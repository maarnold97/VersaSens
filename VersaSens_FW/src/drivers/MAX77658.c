/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MAX77658.c                                                   **
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
* @file   MAX77658.c
* @date   DD/MM/YY
* @brief  This is the main header of MAX77658.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _MAX77658_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "MAX77658.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "twim_inst.h"
#include "storage.h"
#include "versa_ble.h"
#include "versa_time.h"
#include "versa_config.h"
#include "SPI_Heepocrates.h"
#include "app_data.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(MAX77658, LOG_LEVEL_INF);

// MAX77658 storage format length
#define MAX77658_STORAGE_LEN 9
// MAX77658 storage format header
#define MAX77658_STORAGE_HEADER 0x8
// MAX77658 mask for the charging flag bit
#define MAX77658_CHARGING_BIT 0b00000010

// MAX77658 temperature high threshold
#define MAX77658_TEMP_HIGH_THR 50
// MAX77658 battery low threshold
#define MAX77658_BATT_LOW_THR 42240

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

/**
 * @brief Function to handle the max77658 thread
 * 
 * @param arg1 A pointer to the first argument passed to the thread.
 * @param arg2 A pointer to the second argument passed to the thread.
 * @param arg3 A pointer to the third argument passed to the thread.
 */
void max77658_thread_func(void *arg1, void *arg2, void *arg3);

/**
 * @brief Function for handling TWIM driver events.
 *
 * @param[in] p_event   Event information structure.
 * @param[in] p_context General purpose parameter set during initialization of the TWIM.
 *                      This parameter can be used to pass additional information to the
 *                      handler function. (Not used in this application.)
 */
static void MAX77658_handler(nrfx_twim_evt_t const * p_event, void * p_context);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;

// TX buffer
uint8_t tx_buffer_pmu[MAX_SIZE_TRANSFER + 1];

// Flag to check if the last transfer was successful
bool MAX77658_last_transfer_succeeded = false;

volatile MAX77658_REG MAX77658_registers;
volatile MAX77658_FG_REG MAX77658_FG_registers;

// Flag to check if continuous read is enabled
bool MAX77658_cont_read = false;

MAX77658_StorageFormat MAX77658_Storage;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(MAX77658_thread_stack, 1024);
struct k_thread MAX77658_thread;

// Battery status flags
bool battery_charging = false;
bool battery_low = false;
bool temperature_high = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int MAX77658_read_8bit(uint8_t addr, uint8_t *data){
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = addr;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_MAIN_ADDR, tx_buffer_pmu, 1, data, 1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_8bit(uint8_t addr, uint8_t data){
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = addr;
    tx_buffer_pmu[1] = data;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_MAIN_ADDR, tx_buffer_pmu, 2);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_read_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes) 
{
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = start_addr;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_MAIN_ADDR, tx_buffer_pmu, 1, data, num_bytes);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes) 
{
    if (num_bytes > MAX_SIZE_TRANSFER) {
        return -1;
    }

    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = start_addr;
    memcpy(&tx_buffer_pmu[1], data, num_bytes);
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_MAIN_ADDR, tx_buffer_pmu, num_bytes+1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_16bit(uint8_t start_address, uint16_t *data, size_t num_words)
{
    if (num_words > MAX_SIZE_TRANSFER/2) {
        return -1;
    }

    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = start_address;

    // Fill the buffer with data, LSB first
    for (size_t i = 0; i < num_words; i++) {
        tx_buffer_pmu[i*2 + 1] = data[i] & 0xFF;  // LSB
        tx_buffer_pmu[i*2 + 2] = data[i] >> 8;    // MSB
    }

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_FUEL_GAUGE_ADDR, tx_buffer_pmu, num_words*2 + 1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_read_16bit(uint8_t start_address, uint16_t *data, size_t num_words)
{
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_pmu[0] = start_address;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_FUEL_GAUGE_ADDR, tx_buffer_pmu, 1, (uint8_t *)data, num_words*2);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_init(void){
    // Get the I2C instance
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    // Initialize the MAX77658 registers
    MAX77658_REG reg;
    MAX77658_FG_REG fg_reg;

    reg.REG_CNFG_SBB2_B.b.OP_MODE = 0b00;
    reg.REG_CNFG_SBB2_B.b.IP_SBB2 = 0b00;
    reg.REG_CNFG_SBB2_B.b.ADE_SBB2 = 0b0;
    reg.REG_CNFG_SBB2_B.b.EN_SBB2 = 0b100;
    MAX77658_write_8bit(REG_CNFG_SBB2_B_ADDR, reg.REG_CNFG_SBB2_B.w);
    reg.REG_CNFG_CHG_E.b.CHG_CC = 0x3F;             //300mA
    reg.REG_CNFG_CHG_E.b.T_FAST_CHG = 0b01;         //3h
    MAX77658_write_8bit(REG_CNFG_CHG_E_ADDR, reg.REG_CNFG_CHG_E.w);
    reg.REG_CNFG_CHG_F.b.CHG_CC_JEITA = 0x3F;       //300mA
    reg.REG_CNFG_CHG_F.b._reserved_1 = 0b0;
    MAX77658_write_8bit(REG_CNFG_CHG_F_ADDR, reg.REG_CNFG_CHG_F.w);
    reg.REG_CNFG_CHG_G.b.CHG_CV = 0x18;             //4.20V
    reg.REG_CNFG_CHG_G.b.USBS = 0b0;                //USBS not suspended
    reg.REG_CNFG_CHG_G.b.FUS_M = 0b1;
    MAX77658_write_8bit(REG_CNFG_CHG_G_ADDR, reg.REG_CNFG_CHG_G.w);
    reg.REG_CNFG_CHG_H.b.CHG_CV_JEITA = 0x18;       //4.20V
    reg.REG_CNFG_CHG_H.b.SYS_BAT_PRT = 0b1;
    reg.REG_CNFG_CHG_H.b.CHR_TH_EN = 0b1;
    MAX77658_write_8bit(REG_CNFG_CHG_H_ADDR, reg.REG_CNFG_CHG_H.w);
    reg.REG_CNFG_CHG_B.b.VCHGIN_MIN = 0b000;        //4.0V
    reg.REG_CNFG_CHG_B.b.ICHGIN_LIM = 0b000;        //475mA
    reg.REG_CNFG_CHG_B.b.I_PQ = 0b0;                //10%
    reg.REG_CNFG_CHG_B.b.CHG_EN = 0b1;              //Enable charger
    MAX77658_write_8bit(REG_CNFG_CHG_B_ADDR, reg.REG_CNFG_CHG_B.w);
    uint16_t capa = 0x01A4;
    MAX77658_write_16bit(REG_DesignCap_ADDR, &capa, 1);
    uint16_t IchgTerm = 0x0070;
    MAX77658_write_16bit(REG_IChgTerm_ADDR, &IchgTerm, 1);
    uint16_t Vempty = 0x9600;
    MAX77658_write_16bit(REG_VEmpty_ADDR, &Vempty, 1);

    // Start the MAX77658 thread
    k_thread_create(&MAX77658_thread, MAX77658_thread_stack, K_THREAD_STACK_SIZEOF(MAX77658_thread_stack),
                    max77658_thread_func, NULL, NULL, NULL, MAX77658_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&MAX77658_thread, "MAX77658_thread");

    printk("MAX77658_init\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_config(void){
    printk("MAX77658_configure\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_start_continuous_read(void)
{
    MAX77658_cont_read = true;

    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_stop_continuous_read(void)
{
    MAX77658_cont_read = false;

    return 0;
}

/*****************************************************************************
*****************************************************************************/

bool get_battery_charging(void)
{
    return battery_charging;
}

/*****************************************************************************
*****************************************************************************/

bool get_temperature_high(void)
{
    return temperature_high;
}

/*****************************************************************************
*****************************************************************************/

bool get_battery_low(void)
{
    return battery_low;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static void MAX77658_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    // printk("TWIM event: %d\n", p_event->type);

    // Handle the event
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            MAX77658_last_transfer_succeeded = true;
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            LOG_ERR("TWIM address NACK\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            LOG_ERR("TWIM data NACK\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_OVERRUN:
            LOG_ERR("TWIM overrun\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_BUS_ERROR:
            LOG_ERR("TWIM bus error\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        default:
            break;
    }
}

/*****************************************************************************
*****************************************************************************/

void max77658_thread_func(void *arg1, void *arg2, void *arg3)
{
    // Initialize the MAX77658 storage format
    MAX77658_Storage.metadata = init_sensor_packet_metadata_with_length(MAX77658_STORAGE_HEADER, MAX77658_STORAGE_LEN);
    uint8_t index = 0;
    uint16_t data_read[4];
    uint8_t data_read_8bit[1];
    while(1)
    {
        // Get the current time
        struct time_values current_time = get_time_values();
        uint32_t time_s_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        // Read the MAX77658 registers
        MAX77658_read_16bit(REG_Temp_ADDR, data_read, 3);
        MAX77658_read_16bit(REG_RepSOC_ADDR, &data_read[3], 1);
        MAX77658_read_8bit(REG_STAT_CHG_B_ADDR, data_read_8bit);

        // Check the battery charging status
        if(data_read_8bit[0] & MAX77658_CHARGING_BIT){
            battery_charging = true;
        }
        else{
            battery_charging = false;
        }

        // Set the battery percentage for the BLE
        set_battery_data(&data_read[1]);

        // Check the temperature status
        uint8_t temp = ((uint8_t*)data_read)[1];
        if(temp > MAX77658_TEMP_HIGH_THR){
            temperature_high = true;
        }
        else{
            temperature_high = false;
        }

        // Check the battery low status
        uint16_t voltage = data_read[1];
        if(voltage < 42240){
            battery_low = true;
        }
        else{
            battery_low = false;
        }

        // Save the measurements
        if(MAX77658_cont_read){
            update_sensor_packet_metadata_without_length(&MAX77658_Storage.metadata, time_s_bin, time_ms_bin, ((index++)%4));
            MAX77658_Storage.temperature = data_read[0];
            MAX77658_Storage.voltage = data_read[1];
            MAX77658_Storage.current = data_read[2];
            MAX77658_Storage.soc = data_read[3];

            storage_add_to_fifo((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));
            ble_add_to_fifo((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));
            if(VCONF_MAX77658_HEEPO)
            {
                SPI_Heep_add_fifo((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));
            }
            if(VCONF_MAX77658_APPDATA)
            {
                app_data_add_to_fifo((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));
            }

        }

        k_sleep(K_MSEC(200));
    }
    k_thread_abort(k_current_get());
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/