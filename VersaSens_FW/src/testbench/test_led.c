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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


int main(void)
{
    nrf_gpio_cfg_output(START_PIN);
    nrf_gpio_pin_set(START_PIN);
    
    versa_init();
    // enable_auto_connect();
    versa_config();

    versa_start_led_thread();
    versa_start_mode_thread();

    SPI_Heepocrates_init();

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
