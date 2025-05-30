#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>
#include <time.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <nrfx_gpiote.h>
#include "MAX77658.h"
#include "versa_ble.h"
#include "twim_inst.h"
#include "storage.h"
#include "ADS1298.h"
#include "MAX30001.h"
#include "versa_api.h"
#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <SPI_Heepocrates.h>
#include "app_data.h"
#include <zephyr/drivers/led.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED_PWM_NODE_ID	 DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds)
int main(void)
{
    nrf_gpio_cfg_output(START_PIN);
    nrf_gpio_pin_set(START_PIN);
    
    versa_init();
    // enable_auto_connect();
    versa_config();
    // const struct device *led_pwm;

    // led_pwm = DEVICE_DT_GET(LED_PWM_NODE_ID);
	// if (!device_is_ready(led_pwm)) {
	// 	LOG_ERR("Device %s is not ready", led_pwm->name);
	// }

    // uint8_t i;
    // while(1) {
    //     for(i=0;i<3;i++) {
    //         //led_set_brightness(led_pwm, i, 50);
    //         led_on(led_pwm, i);
    //         k_sleep(K_MSEC(1000));
    //         led_off(led_pwm,i);
    //         k_sleep(K_MSEC(1000));
    //     }
    // }
    // versa_start_led_thread();

    nrf_gpio_cfg_output(GREEN_LED_PIN);
    nrf_gpio_pin_set(GREEN_LED_PIN);

    nrf_gpio_cfg_output(RED_LED_PIN);
    nrf_gpio_pin_set(RED_LED_PIN);

    nrf_gpio_cfg_output(YELLOW_LED_PIN);
    nrf_gpio_pin_set(YELLOW_LED_PIN);
    versa_start_mode_thread();

    SPI_Heepocrates_init();

    while (1)
    {
        // data aquisition example
        k_sleep(K_MSEC(10));
        struct app_data_struct *data = k_malloc(sizeof(*data));
        if (data == NULL)
        {
            LOG_ERR("Failed to allocate memory for new_data\n");
        }
        else
        {
            app_data_get_from_fifo(data);
        }
        
        if (data != NULL)
        {
            LOG_INF("Data received from FIFO: %02hx", data->data[0]);
            k_free(data);
        }
    }
}
