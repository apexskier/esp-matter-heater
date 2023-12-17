#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include "driver/gpio.h"
#include <esp_matter.h>
#include <esp_matter_console.h>

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include <app_priv.h>
#include <app_reset.h>
#include <static-supported-temperature-levels.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

static const char *TAG = "app_main";
static uint16_t thermostat_endpoint_id = 0;
static uint32_t system_mode_attribute_id = 0;
static uint32_t occupied_heating_setpoint_id = 0;

typedef enum {
    SYSTEM_MODE_OFF = 0,
    SYSTEM_MODE_COOL = 3,
    SYSTEM_MODE_HEAT = 4,
} system_mode_t;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

static chip::app::Clusters::TemperatureControl::AppSupportedTemperatureLevelsDelegate sAppSupportedTemperatureLevelsDelegate;
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address Changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    default:
        break;
    }
}


// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);

    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(
    callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val,
    void *priv_data
) {
        ESP_LOGI(TAG, "Attribute update callback: type: %d, endpoint: %u, cluster: %lu, attribute: %lu", type, endpoint_id,
                cluster_id, attribute_id);
    if (type == PRE_UPDATE) {
        ESP_LOGI(TAG, "value type: %u", val->type);
        if (occupied_heating_setpoint_id && attribute_id == occupied_heating_setpoint_id) {
            assert(val->type == ESP_MATTER_VAL_TYPE_INT16);
            int16_t temp = val->val.i16; // this is 0.01 degrees C
            ESP_LOGI(TAG, "Occupied heating setpoint update to %d", temp);
        } else if (system_mode_attribute_id && attribute_id == system_mode_attribute_id) {
            assert(val->type == ESP_MATTER_VAL_TYPE_ENUM8);
            uint8_t mode = val->val.u8;
            ESP_LOGI(TAG, "System mode update to %d", mode);
        }
    }

    return ESP_OK;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize driver */
    app_driver_handle_t reset_handle = app_driver_button_init();
    app_reset_button_register(reset_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    thermostat::config_t thermostat_config;
    thermostat_config.thermostat.local_temperature = 2500;
    thermostat_config.thermostat.control_sequence_of_operation = 2; // heating only
    thermostat_config.thermostat.system_mode = SYSTEM_MODE_OFF;
    endpoint_t *thermostat_endpoint = thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, NULL);

    /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
    if (!node || !thermostat_endpoint) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    esp_matter::cluster_t *thermostat_cluster = cluster::get(thermostat_endpoint, chip::app::Clusters::Thermostat::Id);

    thermostat_endpoint_id = endpoint::get_id(thermostat_endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", thermostat_endpoint_id);

    system_mode_attribute_id = attribute::get_id(esp_matter::attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::SystemMode::Id));

    occupied_heating_setpoint_id = attribute::get_id(esp_matter::attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::OccupiedHeatingSetpoint::Id));
    ESP_LOGI(TAG, "OccupiedHeatingSetpoint attribute id %ld", occupied_heating_setpoint_id);


    // Stable readings require a brief period before communication
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_NUM_4);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    ESP_LOGI(TAG, "Find devices:");
    OneWireBus_ROMCode device_rom_codes[1] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(TAG, "Found %d device%s", num_devices, num_devices == 1 ? "" : "s");

    // attribute_t *attribute = attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::FeatureMap::Id);
    // esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    // attribute::get_val(attribute, &val);
    // ESP_LOGI(TAG, "existing feature map is %lu", val.val.u32);
    // val.val.u32 = static_cast<uint32_t>(chip::app::Clusters::Thermostat::Feature::kCooling) &
    //     static_cast<uint32_t>(chip::app::Clusters::Thermostat::Feature::kHeating);// &
    //     // static_cast<uint32_t>(chip::app::Clusters::Thermostat::Feature::kCooling) &
    //     // static_cast<uint32_t>(chip::app::Clusters::Thermostat::Feature::kOccupancy);
    // err = attribute::set_val(attribute, &val);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to update feature map: %d", err);
    // }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
    }
    chip::app::Clusters::TemperatureControl::SetInstance(&sAppSupportedTemperatureLevelsDelegate);

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::init();
#endif
}
