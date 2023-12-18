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
static uint32_t thermostat_cluster_id = 0;

#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (3000)   // milliseconds
#define UPDATE_PERIOD        (10000)   // milliseconds

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
)
{
    ESP_LOGI(TAG, "Attribute update callback: type: %d, endpoint: %u, cluster: %lu, attribute: %lu", type, endpoint_id, cluster_id, attribute_id);
    if (type == PRE_UPDATE) {
        ESP_LOGI(TAG, "value type: %u", val->type);
        if (attribute_id == chip::app::Clusters::Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
            // assert(val->type == ESP_MATTER_VAL_TYPE_INT16);
            // setpoint_value = val->val.i16; // this is 0.01 degrees C
            // ESP_LOGI(TAG, "Occupied heating setpoint update to %d", setpoint_value);
        } else if (attribute_id == chip::app::Clusters::Thermostat::Attributes::SystemMode::Id) {
            assert(val->type == ESP_MATTER_VAL_TYPE_ENUM8);
            uint8_t mode = val->val.u8;
            ESP_LOGI(TAG, "System mode update to %d", mode);
        }
    }

    return ESP_OK;
}

void take_temperature_reading( void *pvParameters )
{
    esp_err_t matter_err = ESP_OK;

    // Stable readings require a brief period before communication
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus *owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_NUM_4);
    owb_use_crc(owb, true);

    OneWireBus_ROMCode device_rom_code;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    if (!found) {
        ESP_LOGE(TAG, "No temperature sensor found");
    } else {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        device_rom_code = search_state.rom_code;
        ESP_LOGI(TAG, "Found temperature sensor: %s", rom_code_s);
    }

    DS18B20_Info* ds18b20_info = ds18b20_malloc();
    ds18b20_init_solo(ds18b20_info, owb);
    ds18b20_use_crc(ds18b20_info, true);
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);

    float reading;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(ds18b20_info);

        // Read the results immediately after conversion otherwise it may fail
        // (using printf before reading may take too long)
        DS18B20_ERROR error = ds18b20_read_temp(ds18b20_info, &reading);
        if (error != DS18B20_OK) {
            ESP_LOGE(TAG, "Temperature reading failed with error: %d", error);
        } else {
            ESP_LOGI(TAG, "Temperature reading: %.1f°C", reading);

            // set temperature attribute
            esp_matter_attr_val_t temp_val = esp_matter_int16(reading * 100);
            matter_err = attribute::update(
                thermostat_endpoint_id,
                thermostat_cluster_id,
                chip::app::Clusters::Thermostat::Attributes::LocalTemperature::Id,
                &temp_val
            );
            if (matter_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to temperature attribute: %d", matter_err);
            }
        }

        xTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void update_heater_state( void *pvParameters )
{
    esp_err_t matter_err = ESP_OK;
    TickType_t last_wake_time = xTaskGetTickCount();

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, thermostat_endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, chip::app::Clusters::Thermostat::Id);
    esp_matter_attr_val_t setpoint_val = esp_matter_invalid(NULL);
    attribute_t *occupied_setpoint_attribute = attribute::get(cluster, chip::app::Clusters::Thermostat::Attributes::OccupiedHeatingSetpoint::Id);
    esp_matter_attr_val_t local_temp_val = esp_matter_invalid(NULL);
    attribute_t *local_temp_attribute = attribute::get(cluster, chip::app::Clusters::Thermostat::Attributes::LocalTemperature::Id);
    esp_matter_attr_val_t system_mode_val = esp_matter_invalid(NULL);
    attribute_t *system_mode_attribute = attribute::get(cluster, chip::app::Clusters::Thermostat::Attributes::SystemMode::Id);
    // TODO
    // attribute_t *thermostat_running_mode_attribute = attribute::get(cluster, chip::app::Clusters::Thermostat::Attributes::ThermostatRunningMode::Id);

    while (true) {
        matter_err = attribute::get_val(occupied_setpoint_attribute, &setpoint_val);
        if (matter_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get setpoint val: %d", matter_err);
        }
        assert(setpoint_val.type == ESP_MATTER_VAL_TYPE_INT16);

        matter_err = attribute::get_val(local_temp_attribute, &local_temp_val);
        if (matter_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get local temp val: %d", matter_err);
        }
        assert(local_temp_val.type == ESP_MATTER_VAL_TYPE_NULLABLE_INT16);

        matter_err = attribute::get_val(system_mode_attribute, &system_mode_val);
        if (matter_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get system mode: %d", matter_err);
        }
        assert(system_mode_val.type == ESP_MATTER_VAL_TYPE_ENUM8);

        ESP_LOGI(TAG, "Occupied heating setpoint is %d, local temp %d, system mode %d", setpoint_val.val.i16, local_temp_val.val.i16, system_mode_val.val.i8);
        esp_matter_attr_val_t state_val;
        // TODO check if local temp is null
        if (local_temp_val.val.i16 + 1000 < setpoint_val.val.i16) {
            gpio_set_level(GPIO_NUM_2, 1);
            gpio_set_level(GPIO_NUM_3, 1);
            state_val = esp_matter_bitmap16(0b001001);
        } else if (local_temp_val.val.i16 < setpoint_val.val.i16) {
            gpio_set_level(GPIO_NUM_2, 1);
            gpio_set_level(GPIO_NUM_3, 0);
            state_val = esp_matter_bitmap16(0b000001);
        } else {
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 0);
            state_val = esp_matter_bitmap16(0);
        }

        // not supported:
        // - chip::app::Clusters::Thermostat::Attributes::ThermostatRunningMode::Id,
        // this is also not supported by esp-matter, I think, but it _should_ work
        matter_err = attribute::update(
            thermostat_endpoint_id,
            thermostat_cluster_id,
            chip::app::Clusters::Thermostat::Attributes::ThermostatRunningState::Id,
            &state_val
        );
        if (matter_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update running state: %d", matter_err);
        }

        xTaskDelayUntil(&last_wake_time, UPDATE_PERIOD / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
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
    thermostat_config.thermostat.control_sequence_of_operation = static_cast<uint8_t>(chip::app::Clusters::Thermostat::ThermostatControlSequence::kHeatingOnly);
    thermostat_config.thermostat.system_mode = SYSTEM_MODE_OFF;
    endpoint_t *thermostat_endpoint = thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, NULL);

    /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
    if (!node || !thermostat_endpoint) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    esp_matter::cluster_t *thermostat_cluster = cluster::get(thermostat_endpoint, chip::app::Clusters::Thermostat::Id);

    thermostat_cluster_id = cluster::get_id(thermostat_cluster);

    thermostat_endpoint_id = endpoint::get_id(thermostat_endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", thermostat_endpoint_id);

    if (xTaskCreate(take_temperature_reading, "take_temperature_reading", 4096, NULL, configMAX_PRIORITIES-2, NULL) == pdFAIL) {
        ESP_LOGE(TAG, "Failed to create take_temperature_reading task");
    }

    if (xTaskCreate(update_heater_state, "update_heater_state", 4096, NULL, configMAX_PRIORITIES-2, NULL) == pdFAIL) {
        ESP_LOGE(TAG, "Failed to create update_heater_state task");
    }

    esp_matter_attr_val_t val = esp_matter_enum8(1); // heating only
    err = attribute::set_val(
        attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::FeatureMap::Id),
        &val
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update FeatureMap: %d", err);
    }

    val = esp_matter_enum8(static_cast<uint8_t>(chip::app::Clusters::Thermostat::ThermostatControlSequence::kHeatingOnly));
    err = attribute::set_val(
        attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::ControlSequenceOfOperation::Id),
        &val
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ControlSequenceOfOperation: %d", err);
    }

    val = esp_matter_enum8(static_cast<uint8_t>(chip::app::Clusters::Thermostat::ThermostatSystemMode::kOff));
    err = attribute::set_val(
        attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::SystemMode::Id),
        &val
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ThermostatSystemMode: %d", err);
    }

    // this appears to be the real value in apple home, but apple doesn't seem to use it
    val = esp_matter_int16(800);
    err = attribute::set_val(
        attribute::get(thermostat_cluster, chip::app::Clusters::Thermostat::Attributes::MinHeatSetpointLimit::Id),
        &val
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MinHeatSetpointLimit: %d", err);
    }

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
