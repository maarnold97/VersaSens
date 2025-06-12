/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : versa_api.c                                                     **
** version  : 1                                                            **
** date     : DD/MM/YY                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : DD/MM/YY
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   versa_api.c
* @date   DD/MM/YY
* @brief  This is the main header of versa_api.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _VERSA_API_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

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
#include "versa_config.h"
#include "T5838.h"
#include "MLX90632.h"
#include "MAX86178.h"
#include "sensors_list.h"
#include "versa_time.h"
#include "pin_assignments.h"

#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

LOG_MODULE_REGISTER(versa_api, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/*
 * @brief Function to handle the LED thread
 *
 * @param arg1: Unused
 * @param arg2: Unused
 * @param arg3: Unused
 * @retval None
*/
void LED_thread_func(void *arg1, void *arg2, void *arg3);

/*
 * @brief Function to handle the mode thread
 *
 * @param arg1: Unused
 * @param arg2: Unused
 * @param arg3: Unused
 * @retval None
*/
void mode_thread_func(void *arg1, void *arg2, void *arg3);

/*
 * @brief Function to handle the new module thread
 *
 * @param arg1: Unused
 * @param arg2: Unused
 * @param arg3: Unused
 * @retval None
*/
void new_module_thread_func(void *arg1, void *arg2, void *arg3);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

// Flags to enable or disable the sensors
int vconf_ads1298_en = VCONF_ADS1298_EN;
int vconf_max30001_en = VCONF_MAX30001_EN;
int vconf_bno086_en = VCONF_BNO086_EN;
int vconf_max86178_en = VCONF_MAX86178_EN;
int vconf_mlx90632_en = VCONF_MLX90632_EN;
int vconf_t5838_en = VCONF_T5838_EN;
int vconf_max77658_en = VCONF_MAX77658_EN;

// ADS1298 configuration
int vconf_ads1298_fs = VCONF_ADS1298_FS;
int vconf_ads1298_gain = VCONF_ADS1298_GAIN;

// MAX30001 configuration
int vconf_max30001_mode = VCONF_MAX30001_MODE;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! LED thread stack and instance*/
K_THREAD_STACK_DEFINE(LED_thread_stack, 256);
struct k_thread LED_thread;

/*! Mode thread stack and instance */
K_THREAD_STACK_DEFINE(mode_thread_stack, 2048);
struct k_thread mode_thread;

/*! New module thread stack and instance */
K_THREAD_STACK_DEFINE(new_module_thread_stack, 2048);
struct k_thread new_module_thread;

// Mode of the device
int mode = MODE_IDLE;
// Mutex to handle mode modification
K_MUTEX_DEFINE(mode_mutex);

// Flags 
bool sensor_started = false;
bool new_module_thread_stop = false;
bool auto_disconnect = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int versa_init(void)
{
    nrf_gpio_pin_set(RST_N_PIN);
    //set reset pin to output
    nrf_gpio_cfg_output(RST_N_PIN);

    
    int ret;

    // Initialize the BLE
    start_ble();
    // Initialize the I2C instance
    twim_inst_init();
    // Initialize the MAX77658
    MAX77658_init();
    tlp0102_init();
    // Initialize the storage
    storage_init();
    k_sleep(K_MSEC(500));

    enable_3v3_peripherals();
    enable_5v();
    enable_1v8();

    // Set the start pin to output low
    nrf_gpio_cfg_output(START_PIN);
    nrf_gpio_pin_clear(START_PIN);

    #ifndef BNO086_DISABLE
    // If the BNO086 is enabled, initialize it
    if(vconf_bno086_en)
    {
        nrf_gpio_pin_clear(IMU_ENABLE_N); // enable the power supply
        k_sleep(K_MSEC(10));
        sensors_list[BNO086_ID].init();
    }
    #endif

    nrf_gpio_pin_clear(RST_N_PIN);
    k_sleep(K_MSEC(100));
    //set reset pin to high
    nrf_gpio_pin_set(RST_N_PIN);
    k_sleep(K_MSEC(100));


    #ifndef MAX30001_DISABLE
    // If the MAX30001 is enabled, initialize it
    if(vconf_max30001_en)
    {
        ret = sensors_list[MAX30001_ID].init();
        // If the initialization failed and the auto_disconnect is enabled, disable the module
        if (ret == -2 & auto_disconnect)
        {
            vconf_max30001_en = 0;
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef MAX86178_DISABLE
    // If the MAX77658 is enabled, initialize it
    if(vconf_max86178_en)
    {
        ret = sensors_list[MAX86178_ID].init();
        // If the initialization failed and the auto_disconnect is enabled, disable the module
        if (ret == -2 & auto_disconnect)
        {
            vconf_max86178_en = 0;
            vconf_t5838_en = 0;
            vconf_mlx90632_en = 0;
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef T5838_DISABLE
    // If the T5838 is enabled, initialize it
    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].init();
    }
    #endif
    
    k_sleep(K_MSEC(50));

    #ifndef ADS1298_DISABLE
    // If the ADS1298 is enabled, initialize it
    if(vconf_ads1298_en)
    {
        ret = sensors_list[ADS1298_ID].init();
        // If the initialization failed and the auto_disconnect is enabled, disable the module
        if (ret == -2 & auto_disconnect)
        {
            vconf_ads1298_en = 0;
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef MLX90632_DISABLE
    // If the MLX90632 is enabled, initialize it
    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].init();
    }
    #endif

    // Set the status to idle
    set_status(BLE_STATUS_IDLE);

    LOG_INF("Versa API sensors initialized\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int enable_auto_connect(void)
{
    new_module_thread_stop = false;
    auto_disconnect = true;
    // Start the new module thread
    k_tid_t new_module_thread_id = k_thread_create(&new_module_thread, new_module_thread_stack, K_THREAD_STACK_SIZEOF(new_module_thread_stack),
                                             new_module_thread_func, NULL, NULL, NULL, -1, 0, K_NO_WAIT);
    k_thread_name_set(new_module_thread_id, "New Module Thread");

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int disable_auto_connect(void)
{
    // Stop the new module thread
    new_module_thread_stop = true;
    auto_disconnect = false;
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_config(void)
{
    #ifndef ADS1298_DISABLE
    // If the ADS1298 is enabled, configure it
    if(vconf_ads1298_en)
    {
        if (sensors_list[ADS1298_ID].config != NULL) {
            sensors_list[ADS1298_ID].config();
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef MAX30001_DISABLE
    // If the MAX30001 is enabled, configure it
    if(vconf_max30001_en)
    {
        if (sensors_list[MAX30001_ID].config != NULL) {
            sensors_list[MAX30001_ID].config();
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef MLX90632_DISABLE
    // If the MLX90632 is enabled, configure it
    if(vconf_mlx90632_en)
    {
        if (sensors_list[MLX90632_ID].config != NULL) {
            sensors_list[MLX90632_ID].config();
        }
    }
    #endif

    k_sleep(K_MSEC(50));

    #ifndef MAX86178_DISABLE
    // If the MAX86178 is enabled, configure it
    if(vconf_max86178_en)
    {
        if (sensors_list[MAX86178_ID].config != NULL) {
            sensors_list[MAX86178_ID].config();
        }
    }
    #endif

    LOG_INF("Versa API sensors configured\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_sensor_start(void)
{
    // Open the storage file
    storage_open_file(STORAGE_CREATE);
    // Set the start pin to high
    nrf_gpio_pin_set(START_PIN);

    // Set the start time of the measurements
    sync_time();

    k_sleep(K_MSEC(10));

    // If the MAX77658 is enabled, start the continuous read
    if(vconf_max77658_en)
    {
        MAX77658_start_continuous_read();
    }

    #ifndef ADS1298_DISABLE
    // If the ADS1298 is enabled, start the continuous read
    if(vconf_ads1298_en)
    {
        sensors_list[ADS1298_ID].start_continuous();
    }
    #endif

    #ifndef MAX30001_DISABLE
    // If the MAX30001 is enabled, start the continuous read
    if(vconf_max30001_en)
    {
        sensors_list[MAX30001_ID].start_continuous();
    }
    #endif

    #ifndef BNO086_DISABLE
    // If the BNO086 is enabled, start the continuous read
    if(vconf_bno086_en)
    {
        sensors_list[BNO086_ID].start_continuous();
    }
    #endif

    #ifndef T5838_DISABLE
    // If the T5838 is enabled, start the continuous read
    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].start_continuous();
    }
    #endif

    #ifndef MLX90632_DISABLE
    // If the MLX90632 is enabled, start the continuous read
    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].start_continuous();
    }
    #endif

    #ifndef MAX86178_DISABLE
    // If the MAX86178 is enabled, start the continuous read
    if(vconf_max86178_en)
    {
        sensors_list[MAX86178_ID].start_continuous();
    }
    #endif

    LOG_INF("Versa API sensors started\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_sensor_stop(void)
{
    // Set the start pin to low
    nrf_gpio_pin_clear(START_PIN);

    // If the MAX77658 is enabled, stop the continuous read
    if(vconf_max77658_en)
    {
        MAX77658_stop_continuous_read();
    }

    #ifndef ADS1298_DISABLE
    // If the ADS1298 is enabled, stop the continuous read
    if(vconf_ads1298_en)
    {
        sensors_list[ADS1298_ID].stop_continuous();
    }
    #endif

    #ifndef MAX30001_DISABLE
    // If the MAX30001 is enabled, stop the continuous read
    if(vconf_max30001_en)
    {
        sensors_list[MAX30001_ID].stop_continuous();
    }
    #endif

    #ifndef BNO086_DISABLE
    // If the BNO086 is enabled, stop the continuous read
    if(vconf_bno086_en)
    {
        sensors_list[BNO086_ID].stop_continuous();
    }
    #endif

    #ifndef T5838_DISABLE
    // If the T5838 is enabled, stop the continuous read
    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].stop_continuous();
    }
    #endif

    #ifndef MLX90632_DISABLE
    // If the MLX90632 is enabled, stop the continuous read
    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].stop_continuous();
    }
    #endif

    #ifndef MAX86178_DISABLE
    // If the MAX86178 is enabled, stop the continuous read
    if(vconf_max86178_en)
    {
        sensors_list[MAX86178_ID].stop_continuous();
    }
    #endif

    // Close the storage file
    storage_close_file();

    LOG_INF("Versa API sensors stopped\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_start_led_thread(void)
{
    // Start the LED thread
    k_tid_t LED_thread_id = k_thread_create(&LED_thread, LED_thread_stack, K_THREAD_STACK_SIZEOF(LED_thread_stack),
                                           LED_thread_func, NULL, NULL, NULL, 2, 0, K_NO_WAIT);
    k_thread_name_set(LED_thread_id, "LED Thread");

    LOG_INF("Versa API LED thread started\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_start_mode_thread(void)
{
    // Start the mode thread
    k_tid_t mode_thread_id = k_thread_create(&mode_thread, mode_thread_stack, K_THREAD_STACK_SIZEOF(mode_thread_stack),
                                             mode_thread_func, NULL, NULL, NULL, 1, 0, K_NO_WAIT);
    k_thread_name_set(mode_thread_id, "Mode Thread");

    return 0;
}

/****************************************************************************/
/****************************************************************************/

void versa_set_mode(int new_mode)
{
    // Set the current mode
    k_mutex_lock(&mode_mutex, K_FOREVER);
    mode = new_mode;
    k_mutex_unlock(&mode_mutex);
}

/****************************************************************************/
/****************************************************************************/

int versa_get_mode(void)
{
    // Get the current mode
    k_mutex_lock(&mode_mutex, K_FOREVER);
    int current_mode = mode;
    k_mutex_unlock(&mode_mutex);
    return current_mode;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void LED_thread_func(void *arg1, void *arg2, void *arg3)
{
    // Set the LED pins to output
    nrf_gpio_cfg_output(GREEN_LED);
    nrf_gpio_pin_clear(GREEN_LED);

    nrf_gpio_cfg_output(RED_LED);
    nrf_gpio_pin_clear(RED_LED);

    nrf_gpio_cfg_output(YELLOW_LED);
    nrf_gpio_pin_clear(YELLOW_LED);
    k_sleep(K_MSEC(5000));

    // Update the LED status depending on the mode and events
    while (1)
    {
        if (get_temperature_high())
        {
            nrf_gpio_pin_set(GREEN_LED);
            nrf_gpio_pin_clear(RED_LED);
            nrf_gpio_pin_set(YELLOW_LED);
        }
        else if (get_write_failed())
        {
            nrf_gpio_pin_set(GREEN_LED);
            nrf_gpio_pin_set(RED_LED);
            nrf_gpio_pin_set(YELLOW_LED);
        }
        else if (versa_get_mode() == MODE_STORE)
        {
            nrf_gpio_pin_set(GREEN_LED);
            nrf_gpio_pin_toggle(RED_LED);
            nrf_gpio_pin_set(YELLOW_LED);
        }
        else if (versa_get_mode() == MODE_STREAM)
        {
            nrf_gpio_pin_set(GREEN_LED);
            nrf_gpio_pin_set(RED_LED);
            nrf_gpio_pin_toggle(YELLOW_LED);
        }
        else if (get_battery_charging())
        {
            nrf_gpio_pin_toggle(GREEN_LED);
            nrf_gpio_pin_set(RED_LED);
            nrf_gpio_pin_set(YELLOW_LED);
        }
        else if (get_battery_low())
        {
            nrf_gpio_pin_set(GREEN_LED);
            nrf_gpio_pin_set(RED_LED);
            nrf_gpio_pin_clear(YELLOW_LED);
        }
        else if (versa_get_mode() == MODE_IDLE)
        {
            nrf_gpio_pin_clear(GREEN_LED);
            nrf_gpio_pin_set(RED_LED);
            nrf_gpio_pin_set(YELLOW_LED);
        }


        k_sleep(K_MSEC(500));
    }
}

/****************************************************************************/
/****************************************************************************/

void mode_thread_func(void *arg1, void *arg2, void *arg3)
{
    // // Set the mode switch pins to input
    // nrf_gpio_cfg_input(MODE_IDLE_PIN, GPIO_PIN_CNF_PULL_Pulldown);
    // nrf_gpio_cfg_input(MODE_STORE_PIN, GPIO_PIN_CNF_PULL_Pulldown);
    // nrf_gpio_cfg_input(MODE_STREAM_PIN, GPIO_PIN_CNF_PULL_Pulldown);

    versa_set_mode(MODE_STREAM);
    set_status(BLE_STATUS_STREAM);
    enable_stream_data();
    versa_sensor_start();


    while (1)
    {
    //     // Update the mode according to the switch, BLE command and BLE overwrite
    //     if (((nrf_gpio_pin_read(MODE_IDLE_PIN) > 0 | (nrf_gpio_pin_read(MODE_STORE_PIN)==0 & 
    //           nrf_gpio_pin_read(MODE_STREAM_PIN)==0)) & !BLE_overwrite) | (BLE_overwrite & BLE_cmd == BLE_CMD_MODE_IDLE))
    //     {
    //         // Stop the data stream from the BLE
    //         disable_stream_data();
    //         // Stop the sensor continuous read
    //         if(sensor_started)
    //         {
    //             sensor_started = false;
    //             versa_sensor_stop();
    //         }
    //         // Update mode and status
    //         versa_set_mode(MODE_IDLE);
    //         set_status(BLE_STATUS_IDLE);
    //     }
    //     // Update the mode according to the switch, BLE command and BLE overwrite
    //     else if (((nrf_gpio_pin_read(MODE_STORE_PIN) > 0 | (nrf_gpio_pin_read(MODE_IDLE_PIN)==0 & 
    //                nrf_gpio_pin_read(MODE_STREAM_PIN)==0)) & !BLE_overwrite) | (BLE_overwrite & BLE_cmd == BLE_CMD_MODE_STORE))
    //     {
    //         // Stop the data stream from the BLE
    //         disable_stream_data();
    //         // Start the sensor continuous read
    //         if(!sensor_started)
    //         {
    //             sensor_started = true;
    //             versa_sensor_start();
    //         }
    //         // Update mode and status
    //         versa_set_mode(MODE_STORE);
    //         set_status(BLE_STATUS_STORE);
    //     }
    //     // Update the mode according to the switch, BLE command and BLE overwrite
    //     else if (((nrf_gpio_pin_read(MODE_STREAM_PIN) > 0 | (nrf_gpio_pin_read(MODE_IDLE_PIN)==0 & 
    //                nrf_gpio_pin_read(MODE_STORE_PIN)==0)) & !BLE_overwrite) | (BLE_overwrite & BLE_cmd == BLE_CMD_MODE_STREAM))
    //     {
    //         // Start the data stream from the BLE
    //         enable_stream_data();
    //         // Start the sensor continuous read
    //         if(!sensor_started)
    //         {
    //             sensor_started = true;
    //             versa_sensor_start();
    //         }
    //         // Update mode and status
    //         versa_set_mode(MODE_STREAM);
    //         set_status(BLE_STATUS_STREAM);
    //     }

        k_sleep(K_MSEC(200));
    }
}

/****************************************************************************/
/****************************************************************************/

void new_module_thread_func(void *arg1, void *arg2, void *arg3){
    int ret;
    while(!new_module_thread_stop){
        k_sleep(K_MSEC(3000));

        #ifndef BNO086_DISABLE
        // Try to initialize the BNO086
        if(!vconf_bno086_en){
            nrf_gpio_pin_clear(IMU_ENABLE_N); // enable the power supply
            k_sleep(K_MSEC(10));
            vconf_bno086_en = 1;
            sensors_list[BNO086_ID].init();

            // reset BNO and ADS
            nrf_gpio_pin_clear(RST_N_PIN);
            k_sleep(K_MSEC(100));
            //set reset pin to high
            nrf_gpio_pin_set(RST_N_PIN);
            k_sleep(K_MSEC(100));

            // ADS needs to be reconfigured after reset, if it is enabled
            if(vconf_ads1298_en) {
                ret = sensors_list[ADS1298_ID].init();
                // If initialization succeeded, configure the ADS1298
                if (ret != -2)
                {
                    vconf_ads1298_en = 1;
                    sensors_list[ADS1298_ID].config();
                }
            }
        }
        #endif
    

        #ifndef ADS1298_DISABLE
        // Try to initialize the ADS1298
        if(!vconf_ads1298_en){
            ret = sensors_list[ADS1298_ID].init();
            // If initialization succeeded, configure the ADS1298
            if (ret != -2)
            {
                vconf_ads1298_en = 1;
                sensors_list[ADS1298_ID].config();
            }
        }
        #endif

        #ifndef MAX30001_DISABLE
        // Try to initialize the MAX30001
        if(!vconf_max30001_en){
            ret = sensors_list[MAX30001_ID].init();
            // If initialization succeeded, configure the MAX30001
            if (ret != -2)
            {
                vconf_max30001_en = 1;
                sensors_list[MAX30001_ID].config();
            }
        }
        #endif

        #ifndef MAX86178_DISABLE
        // Try to initialize the MAX86178
        if(!vconf_max86178_en){
            ret = sensors_list[MAX86178_ID].init();
            // If initialization succeeded, configure the MAX86178 and initialize the T5838 and MLX90632 
            if (ret != -2)
            {
                vconf_max86178_en = 1;
                sensors_list[MAX86178_ID].config();

                #ifndef T5838_DISABLE
                vconf_t5838_en = 1;
                sensors_list[T5838_ID].init();
                #endif

                #ifndef MLX90632_DISABLE
                vconf_mlx90632_en = 1;
                sensors_list[MLX90632_ID].init();
                sensors_list[MLX90632_ID].config();
                #endif
            }
        }
        #endif

        // Try to initialize the MAX77658
        if(!vconf_max77658_en){
            vconf_max77658_en = 1;
        }
    }
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/