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
#include <nrfx_spis.h>


#define SPIS_INST_IDX 0
#define SCK_PIN_SLAVE 4
#define MOSI_PIN_SLAVE 0
#define MISO_PIN_SLAVE 34
#define CSN_PIN_SLAVE 6


/** @brief Symbol specifying message to be sent via SPIS data transfer. */
#define MSG_TO_SEND_SLAVE "Test message"

/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND_SLAVE ). */
static uint8_t m_tx_buffer_slave[100] = MSG_TO_SEND_SLAVE;

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND_MASTER ). */
static uint8_t m_rx_buffer_slave[100] = MSG_TO_SEND_SLAVE;


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);

/**
 * @brief Function for handling SPIS driver events.
 *
 * @param[in] p_event   Pointer to the SPIS driver event.
 * @param[in] p_context Pointer to the context passed from the driver.
 */
static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context)
{
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE)
    {
        char * p_msg = p_context;
        LOG_INF("SPIS finished. Context passed to the handler: >%s<", p_msg);
        LOG_INF("SPIS rx length: %d", p_event->rx_amount);
        LOG_INF("SPIS rx buffer: %s", m_rx_buffer_slave);
        nrfx_spis_buffers_set(&spis_inst,
                              m_tx_buffer_slave, sizeof(m_tx_buffer_slave),
                              m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    }
}


int main(void)
{
    nrf_gpio_pin_control_select(0, NRF_GPIO_PIN_SEL_APP);
    nrf_gpio_pin_control_select(1, NRF_GPIO_PIN_SEL_APP);
    nrf_gpio_cfg_output(START_PIN);
    nrf_gpio_pin_set(START_PIN);
    
    versa_init();
    // enable_auto_connect();
    versa_config();

    versa_start_led_thread();
    versa_start_mode_thread();

    nrfx_err_t status;
    (void)status;

    // nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);
    nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SCK_PIN_SLAVE,
                                                              MOSI_PIN_SLAVE,
                                                              MISO_PIN_SLAVE,
                                                              CSN_PIN_SLAVE);
    spis_config.mode = NRF_SPIS_MODE_0;

    nrfx_spis_uninit(&spis_inst);

    void * p_context = "Some context";
    status = nrfx_spis_init(&spis_inst, &spis_config, spis_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(SPIS_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_SPIS_INST_HANDLER_GET(SPIS_INST_IDX), 0);

    status = nrfx_spis_buffers_set(&spis_inst,
                                   m_tx_buffer_slave, sizeof(m_tx_buffer_slave),
                                   m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    NRFX_ASSERT(status == NRFX_SUCCESS);

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
