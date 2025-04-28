/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MLX90632.c                                                   **
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
* @file   MLX90632.c
* @date   DD/MM/YY
* @brief  This is the main storage_format of MLX90632.c
*
* Here typically goes a more extensive explanation of what the storage_format
* defines.
*/

#define _MLX90632_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "tlp0102.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "versa_time.h"
#include "versa_ble.h"
#include "versa_config.h"
#include "SPI_Heepocrates.h"
#include "app_data.h"



/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(tlp0102, LOG_LEVEL_INF);

#define DIGI_POT_I2C_ADDRESS 0b01010000

#define IV_WRA_ADDRESS 0x00
#define IV_WRB_ADDRESS 0x01
#define ACR_ADDRESS    0x10

typedef struct {  
    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t reserved         :5;     /*!< bits: 0-4  unused, reserved                                 */
        uint8_t wip              :1;     /*!< bit:  5, WIP, 1 if non-volatile write operation is in progress, 0 if not       */
        uint8_t shdn_n           :1;     /*!< bit:  6, SHDWN, set to 0 to put tlp0102 into shutdown mode                                 */ 
        uint8_t vol              :1;     /*<  bit:  7, VOL, if 1, read/write to volatile regs, if 0, read/write to non-volatile regs*/              
        } b;                             /*!< Structure used for bit access                                 */
        uint8_t w;                       /*!< Type used for word access                                     */
    } ACR_REG;
}__attribute__((packed)) TLP0102_REG;

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
 * @brief This function reads a register from the TLP0102 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading from a register
 *          on the TLP0102 device. The address of the register to read from is provided as a parameter,
 *          and the data read from the register is stored in a buffer also provided as a parameter.
 * 
 * @param[in]   addr      The address of the register to read from.
 * @param[out]  data      A pointer to the buffer where the read data should be stored.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int tlp0102_read_reg(uint8_t addr, uint8_t *data);

/**
 * @brief This function writes a byte of data to a register on the TLP0102 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a byte of data to a register
 *          on the TLP0102 device. The address of the register and the data to be written are provided as parameters.
 * 
 * @param[in]   addr  The address of the register to write to.
 * @param[in]   data  The data to write to the register.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int tlp0102_write_reg(uint8_t addr, uint8_t data);


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

/*! I2C tx and rx buffers */
static uint8_t tx_buffer[2];
static uint8_t rx_buffer[1];


/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int tlp0102_set_cgra_res(uint8_t val, bool non_volatile) {
    static TLP0102_REG reg = {0};
    reg.ACR_REG.b.shdn_n = 0b1;
    reg.ACR_REG.b.vol = non_volatile ? 0b0 : 0b1;

    int ret = 0;
    if(tlp0102_write_reg(ACR_ADDRESS, reg.ACR_REG.w) != 0) {
        return -1;
    }
    return tlp0102_write_reg(IV_WRA_ADDRESS, val);
}


/*****************************************************************************
*****************************************************************************/

int tlp0102_set_core_res(uint8_t val, bool non_volatile) {
    static TLP0102_REG reg = {0};
    reg.ACR_REG.b.shdn_n = 0b1;
    reg.ACR_REG.b.vol = non_volatile ? 0b0 : 0b1;

    int ret = 0;
    if(tlp0102_write_reg(ACR_ADDRESS, reg.ACR_REG.w) != 0) {
        return -1;
    }
    return tlp0102_write_reg(IV_WRB_ADDRESS, val);
}


/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

int tlp0102_read_reg(uint8_t addr, uint8_t *data)
{
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    /*! Write the address in the tx buffer */
    tx_buffer[0] = addr;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(DIGI_POT_I2C_ADDRESS, tx_buffer, 1, rx_buffer, 1);
    *data = rx_buffer[0];
    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int tlp0102_write_reg(uint8_t addr, uint8_t data)
{
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    /*! Write the address and data in the tx buffer */
    tx_buffer[0] = addr;
    tx_buffer[1] = data;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(DIGI_POT_I2C_ADDRESS, tx_buffer, 2);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}


/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/
