/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : twim_inst.c                                                   **
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
Date        : 10/02/2021
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   twim_inst.c
* @date   DD/MM/YY
* @brief  This is the main header of twim_inst.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _TWIM_INST_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "twim_inst.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(twim_inst, LOG_LEVEL_INF);

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
 * @brief Function for handling TWIM driver events.
 *
 * @param[in] p_event   Event information structure.
 * @param[in] p_context General purpose parameter set during initialization of the TWIM.
 *                      This parameter can be used to pass additional information to the
 *                      handler function. In this application, it is not used.
 */
static void twim_handler(nrfx_twim_evt_t const * p_event, void * p_context);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

// Semaphore for I2C
struct k_sem I2C_sem = Z_SEM_INITIALIZER(I2C_sem, 1, 1);

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! twim instance */
static nrfx_twim_t twim_inst = NRFX_TWIM_INSTANCE(TWIM_INST_IDX);

// Flag to check if the last transfer was successful
bool twim_last_transfer_succeeded = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

nrfx_twim_t * twim_get_instance(void)
{
    // Return a pointer to the twim instance
    return (nrfx_twim_t *) &twim_inst;
}

/*****************************************************************************
*****************************************************************************/

int twim_inst_init(void)
{
    // Configuration of the SDA pin
    nrf_gpio_cfg(
        TWIM_SDA_PIN,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_E0E1, // High drive for both 0 and 1
        NRF_GPIO_PIN_NOSENSE);

    // Configuration of the SCL pin
    nrf_gpio_cfg(
        TWIM_SCL_PIN,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_E0E1, // High drive for both 0 and 1
        NRF_GPIO_PIN_NOSENSE);

        
    nrfx_twim_config_t twim_config = NRFX_TWIM_DEFAULT_CONFIG(TWIM_SCL_PIN, TWIM_SDA_PIN);
    twim_config.frequency = NRF_TWIM_FREQ_400K;

    // Initialize the TWIM instance
    nrfx_err_t err_code = nrfx_twim_init(&twim_inst, &twim_config, twim_handler, NULL);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_init failed with error code: %d\n", err_code);
        return -1;
    }

    // Enable the interrupt for the TWIM instance
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TWIM_INST_GET(TWIM_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_TWIM_INST_HANDLER_GET(TWIM_INST_IDX), 0);

    // Enable the TWIM instance
    nrfx_twim_enable(&twim_inst);
    return 0;
}

/*****************************************************************************
*****************************************************************************/

bool twim_is_busy(void)
{
    // Check if the TWIM instance is busy
    return nrfx_twim_is_busy(&twim_inst);
}

/*****************************************************************************
*****************************************************************************/

bool twim_transfer_succeeded(void)
{
    // Return the status of the last transfer
    return twim_last_transfer_succeeded;
}

/*****************************************************************************
*****************************************************************************/

void wait_for_twim_transfer(void)
{
    // Wait for the TWIM transfer to finish
    while (twim_is_busy())
    {
        k_sleep(K_MSEC(1));
    }
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static void twim_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    // printk("TWIM event: %d\n", p_event->type);
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            twim_last_transfer_succeeded = true;
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            LOG_ERR("TWIM address NACK\n");
            twim_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            LOG_ERR("TWIM data NACK\n");
            twim_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_OVERRUN:
            LOG_ERR("TWIM overrun\n");
            twim_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_BUS_ERROR:
            LOG_ERR("TWIM bus error\n");
            twim_last_transfer_succeeded = false;
            break;
        default:
            break;
    }
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/