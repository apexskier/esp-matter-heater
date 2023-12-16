#include <device.h>

#include <app_priv.h>
#include <app_reset.h>

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_config_t config = button_driver_get_config();
    button_handle_t handle = iot_button_create(&config);

    return (app_driver_handle_t)handle;
}
