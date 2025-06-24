/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : SPI_Heepocrates.c                                                   **
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
* @file   SPI_Heepocrates.c
* @date   DD/MM/YY
* @brief  This is the main header of SPI_Heepocrates.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _SPI_HEEPOCRATES_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "SPI_Heepocrates.h"
#include <nrfx_spis.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <nrfx_gpiote.h>
#include "thread_config.h"


/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/** @brief Symbol specifying message to be sent via SPIS data transfer. */
#define MSG_TO_SEND_SLAVE "test message"



/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(SPI_Heepocrates, LOG_LEVEL_DBG);

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Function for handling SPIS driver events.
 *
 * @param[in] p_event   Pointer to the SPIS driver event.
 * @param[in] p_context Pointer to the context passed from the driver.
 */
static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context);

// /**
//  * @brief Function for handling the SPI Heepocrates thread.
//  *
//  * @param[in] arg1 Pointer to the first argument.
//  * @param[in] arg2 Pointer to the second argument.
//  * @param[in] arg3 Pointer to the third argument.
//  */
// void HEEPO_thread_func(void *arg1, void *arg2, void *arg3);

void set_spi_buffers_thread_func(void *arg1, void *arg2, void *arg3);
void calculate_latency_thread_func(void *arg1, void *arg2, void *arg3);
void fill_data_buffers_thread_func(void *arg1, void *arg2, void *arg3);
void handle_spi_buffers_set_thread_func(void *arg1, void *arg2, void *arg3);

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

/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND_SLAVE ). */
static uint8_t m_tx_buffer_slave[300] = MSG_TO_SEND_SLAVE;

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND_MASTER ). */
static uint8_t m_rx_buffer_slave[65535] = MSG_TO_SEND_SLAVE;

nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);

K_FIFO_DEFINE(heepo_fifo);
K_FIFO_DEFINE(startTimeFIFO);
K_FIFO_DEFINE(stopTimeFIFO);

typedef struct {
    void *fifo_reserved;
    int64_t time;
} time_item_t;


static uint8_t buffer0[MAX_BYTES_PER_MEASUREMENT*MAX_CHUNK_SIZE + sizeof(uint32_t)];
static uint8_t buffer1[MAX_BYTES_PER_MEASUREMENT*MAX_CHUNK_SIZE + sizeof(uint32_t)];
static uint8_t *spiDataBuffer = NULL;

static uint32_t chunkSize = MIN_CHUNK_SIZE;
static uint32_t maxChunkSize = MAX_CHUNK_SIZE;

static volatile bool bufferReady = false;
static volatile bool intAsserted = false;

static volatile bool calibrationDone = false;

// void (*chunkSizeCalculator)(uint32_t*);

static atomic_t heepo_fifo_counter = ATOMIC_INIT(0);

// bool length_sent = false;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(setSpiBuffersThreadStack, 1024);
K_THREAD_STACK_DEFINE(fillDataBuffersThreadStack, 8192*2);
K_THREAD_STACK_DEFINE(handleSpiBuffersSetThreadStack, 1024);
K_THREAD_STACK_DEFINE(calculateLatencyThreadStack, 1024);

struct k_thread setSpiBuffersThread;
struct k_thread calculateLatencyThread;
struct k_thread fillDataBuffersThread;
struct k_thread handleSpiBuffersSetThread;

/*! Flag to stop the thread */
volatile bool HEEPO_stop_thread_flag = false;

// semaphore for the thread
K_SEM_DEFINE(HEEPO_XFER_DONE, 0, 1);
K_SEM_DEFINE(SPI_BUFFERS_SET, 0, 1);

K_SEM_DEFINE(HEEPO_BUSY, 2,2);


/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

void SPI_Heepocrates_init(void)
{
    /* Assign pin 0 and 1 to the MCUAPP */
    LOG_DBG("start heepo init");
    nrf_gpio_pin_control_select(0, NRF_GPIO_PIN_SEL_APP);
    nrf_gpio_pin_control_select(1, NRF_GPIO_PIN_SEL_APP);

    nrfx_err_t status;
    (void)status;

    nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SCK_PIN_SLAVE,
                                                              MOSI_PIN_SLAVE,
                                                              MISO_PIN_SLAVE,
                                                              CSN_PIN_SLAVE);
    spis_config.mode = NRF_SPIS_MODE_0;
    spis_config.miso_drive = NRF_GPIO_PIN_S0S1;

    // Initialize the SPIS instance
    nrfx_spis_uninit(&spis_inst);
    void * p_context = "Some context";
    status = nrfx_spis_init(&spis_inst, &spis_config, spis_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    LOG_DBG("spis init");

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(SPIS_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_SPIS_INST_HANDLER_GET(SPIS_INST_IDX), 0);

    // Set the ready pin
    nrf_gpio_pin_clear(PIN_HEEPO_RDY);
    nrf_gpio_cfg_output(PIN_HEEPO_RDY);

    // Start the SPI Heepocrates thread
    SPI_Heepocrates_start(m_tx_buffer_slave, sizeof(m_tx_buffer_slave), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));

    return;
}

/*****************************************************************************
*****************************************************************************/

void SPI_Heepocrates_start(uint8_t * p_tx_buffer, uint16_t length_tx, uint8_t * p_rx_buffer, uint16_t length_rx)
{
    nrfx_err_t status;
    (void)status;

    // Assign TX and RX buffers
    if (p_tx_buffer == NULL)
    {
        p_tx_buffer = m_tx_buffer_slave;
        length_tx = 0;
    }

    if (p_rx_buffer == NULL)
    {
        p_rx_buffer = m_rx_buffer_slave;
        length_rx = 0;
    }
 
    k_thread_create(&setSpiBuffersThread, setSpiBuffersThreadStack, K_THREAD_STACK_SIZEOF(setSpiBuffersThreadStack),
                    set_spi_buffers_thread_func, NULL, NULL, NULL, 13, 0, K_NO_WAIT);
    k_thread_name_set(&setSpiBuffersThread, "setSpiBuffersThread");

    k_thread_create(&calculateLatencyThread, calculateLatencyThreadStack, K_THREAD_STACK_SIZEOF(calculateLatencyThreadStack),
                    calculate_latency_thread_func, NULL, NULL, NULL, 10, 0, K_NO_WAIT);
    k_thread_name_set(&calculateLatencyThread, "calculateLatencyThread");

    k_thread_create(&fillDataBuffersThread, fillDataBuffersThreadStack, K_THREAD_STACK_SIZEOF(fillDataBuffersThreadStack),
                    fill_data_buffers_thread_func, NULL, NULL, NULL, 11, 0, K_NO_WAIT);
    k_thread_name_set(&fillDataBuffersThread, "fillDataBuffersThread");

    k_thread_create(&handleSpiBuffersSetThread, handleSpiBuffersSetThreadStack, K_THREAD_STACK_SIZEOF(handleSpiBuffersSetThreadStack),
                    handle_spi_buffers_set_thread_func, NULL, NULL, NULL, 12, 0, K_NO_WAIT);
    k_thread_name_set(&handleSpiBuffersSetThread, "handleSpiBuffersSetThread");

    LOG_DBG("heepo init");
    return;
}

/*****************************************************************************
*****************************************************************************/

void SPI_Heep_add_fifo(uint8_t *data, size_t size)
{
    // Check if the FIFO is full
    // LOG_DBG("SPU_HEEP_ADD_FIFO CALLED. calibrationDone = %d", calibrationDone);
    if(atomic_get(&heepo_fifo_counter) >= MAX_FIFO_SIZE)
    {
        LOG_ERR("HEEPO FIFO FULL");
        return;
    }

    static bool firstMeasurement = true;
    if(firstMeasurement) {
        firstMeasurement = false;
    } else if(!calibrationDone) {
        return;
    }

    // Allocate a new sensor data
    struct sensor_data_heepo *p_data = k_malloc(sizeof(*p_data));

    if (p_data == NULL)
    {
        LOG_ERR("Failed to allocate memory for k_malloc\n");
        while(true);
        return;
    }

    // Add the data to the FIFO
    p_data->size = size;
    memcpy(p_data->data, data, size);
    k_fifo_put(&heepo_fifo, p_data);
    atomic_inc(&heepo_fifo_counter);
    // LOG_DBG("added to FIFO");

    return;
}


/*****************************************************************************
*****************************************************************************/

void fill_data_buffers_thread_func(void *arg1, void *arg2, void *arg3) 
{
    uint32_t i = 0;
    uint32_t bufferIndex = 0;
    uint32_t bufferSize = 0;
    struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_FOREVER);
    uint32_t measurementSize = p_data->size;
    uint8_t *targetWriteBuffer = buffer0;
    uint32_t counter = 0;
    atomic_dec(&heepo_fifo_counter);
    for(i=0;i<MAX_CHUNK_SIZE;i++) { //fill entire buffer0
        memcpy(buffer0+bufferIndex+sizeof(bufferIndex), p_data->data, measurementSize); 
        bufferIndex += measurementSize;
    }
    k_free(p_data);
    // and also buffer1
    memcpy(buffer1, buffer0, bufferIndex+sizeof(bufferIndex));
    LOG_DBG("buffers full, starting calibration");
    while(!calibrationDone) {
        bufferSize = chunkSize*measurementSize;
        memcpy(targetWriteBuffer, &bufferSize, sizeof(bufferSize));
        // k_sleep(K_MSEC(1000));
        targetWriteBuffer = (targetWriteBuffer == buffer0) ? buffer1 : buffer0;
        k_sem_take(&HEEPO_BUSY, K_FOREVER);
        LOG_DBG("heepo_busy semaphore taken");
        // first time we manually set the buffers, then after it is handled by the NRFX_SPIS_XFER_DONE interrupt
        // static bool firstTimeBuffersSet = true;
        // if(firstTimeBuffersSet) {
        bufferReady = true;
        nrfx_spis_buffers_set(&spis_inst, targetWriteBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
            // firstTimeBuffersSet = false;
        //     LOG_DBG("buffers set for first time");
        // }
        if(counter>1) {
            chunkSize = (chunkSize + CHUNK_STEP_SIZE < maxChunkSize) ? chunkSize + CHUNK_STEP_SIZE : maxChunkSize;
            LOG_DBG("new chunkSize calib = %d", chunkSize);
        }
        counter++;
    }
    LOG_DBG("calib done, data_put_in_ready_counter = %d", counter);
    atomic_val_t fifo_counter;
    i = 0;
    bufferSize = 0;
    targetWriteBuffer = buffer0;

    while(!HEEPO_stop_thread_flag) {
        // LOG_DBG("started main loop");
        p_data = k_fifo_get(&heepo_fifo, K_FOREVER);
        atomic_dec(&heepo_fifo_counter);
        memcpy(targetWriteBuffer+bufferSize+sizeof(bufferSize), p_data->data, p_data->size); 
        bufferSize += p_data->size;
        k_free(p_data); 
        i++;

        if(i == chunkSize) {
            // k_sleep(K_MSEC(1000));
            LOG_DBG("reached chunkSize. HEEPO_BUSY sem value = %d", k_sem_count_get(&HEEPO_BUSY));
            int32_t ret = k_sem_take(&HEEPO_BUSY, K_NO_WAIT);
            // LOG_DBG("ret = %d, EBUSY = %d", ret, EBUSY);
            memcpy(targetWriteBuffer, &bufferSize, sizeof(bufferSize));
            if(ret == -EBUSY) {
                k_sem_take(&HEEPO_BUSY, K_FOREVER);
                fifo_counter = atomic_get(&heepo_fifo_counter);
                LOG_DBG("had to wait for sem. fifoCounter = %d", fifo_counter);
                if(((float)fifo_counter/(float)MIN_CHUNK_SIZE) > 1.0F ) {
                    chunkSize = (chunkSize/2 > MIN_CHUNK_SIZE) ? chunkSize/2 : MIN_CHUNK_SIZE;
                } else {
                    chunkSize = (chunkSize - CHUNK_STEP_SIZE > MIN_CHUNK_SIZE) ? chunkSize - CHUNK_STEP_SIZE : MIN_CHUNK_SIZE;
                }
            } else {
                LOG_DBG("got sem immediately");
                chunkSize = (chunkSize + CHUNK_STEP_SIZE < maxChunkSize) ? chunkSize + CHUNK_STEP_SIZE : maxChunkSize;
            }
            LOG_INF("new chunkSize = %d", chunkSize);
            bufferReady = true;
            nrfx_spis_buffers_set(&spis_inst, targetWriteBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
            bufferSize = 0;
            i = 0;
            
            targetWriteBuffer = (targetWriteBuffer == buffer0) ? buffer1 : buffer0;
            
        }
    }
    k_thread_abort(k_current_get());
}

void calculate_latency_thread_func(void *arg1, void *arg2, void *arg3) {
    time_item_t *startTime;
    time_item_t *stopTime;
    int64_t latency = 0;
    uint32_t bestChunkSize = MIN_CHUNK_SIZE;
    uint32_t i;
    for(i=0;i<2;i++) { // we don't care about the first two measurements
        stopTime = k_fifo_get(&stopTimeFIFO, K_FOREVER);
        k_free(stopTime);
        startTime = k_fifo_get(&startTimeFIFO, K_FOREVER);
        k_free(startTime);
    }
    LOG_DBG("finished first two latency measurements");
    while(!calibrationDone) {
        stopTime = k_fifo_get(&stopTimeFIFO, K_FOREVER);
        startTime = k_fifo_get(&startTimeFIFO, K_FOREVER);
        latency = stopTime->time - startTime->time;
        // LOG_DBG("startTime = %lld, stopTime = %lld", startTime->time, stopTime->time);
        LOG_DBG("Latency = %lld", latency);
        k_free(stopTime);
        k_free(startTime);
        if(latency > MAX_LATENCY_MS) {
            if(bestChunkSize == MIN_CHUNK_SIZE) {
                LOG_ERR("MIN_CHUNK_SIZE CANNOT MEET LATENCY REQUIREMENT!!");
            }
            maxChunkSize = bestChunkSize - CHUNK_STEP_SIZE;
            LOG_INF("new maxChunkSize= %d", maxChunkSize);
            calibrationDone = true;
        } else {
            bestChunkSize += CHUNK_STEP_SIZE;
            if(bestChunkSize > MAX_CHUNK_SIZE) {
                calibrationDone = true;
            }
        }
    }
    LOG_INF("calibration done!");

    k_thread_abort(k_current_get());
}


void handle_spi_buffers_set_thread_func(void *arg1, void *arg2, void *arg3) { // very high priority
    LOG_DBG("starting handle_spi_buffers_set_thread_func");
    uint32_t counter = 0;
    while(!calibrationDone) {
        // LOG_DBG("entering first loop in handle_spi_buffers_set_thread_func");
        k_sem_take(&SPI_BUFFERS_SET, K_FOREVER);
        // LOG_DBG("SPI_BUFFERS_SET sem taken. bufferReady = %d, intAsserted = %d", bufferReady, intAsserted);
        if(bufferReady && !intAsserted) {
            LOG_DBG("asserting int");
            time_item_t *startTime = k_malloc(sizeof(time_item_t));
            if(startTime == NULL) {
                LOG_ERR("unable to allocate memory");
            } else {
                startTime->time = k_uptime_get();
                k_fifo_put(&startTimeFIFO, startTime);
            }
            nrf_gpio_pin_set(PIN_HEEPO_RDY);
            counter++;
            intAsserted = true;
            bufferReady = false;
        }
    }
    LOG_DBG("calib done, data_sent_counter = %d", counter);
    while(true) {
        // LOG_DBG("started main loop");
        k_sem_take(&SPI_BUFFERS_SET, K_FOREVER);
        if(bufferReady && !intAsserted) {
            nrf_gpio_pin_set(PIN_HEEPO_RDY);
            LOG_DBG("asserting int");
            intAsserted = true;
            bufferReady = false;
        }
    }
}
void set_spi_buffers_thread_func(void *arg1, void *arg2, void *arg3) // very high priority
{

    nrfx_err_t status;
    spiDataBuffer = buffer0;
    uint32_t bufferSize;
    uint32_t counter = 0;
    while(!calibrationDone) {
        k_sem_take(&HEEPO_XFER_DONE, K_FOREVER);
        switch(m_rx_buffer_slave[0]) {
            case SEND_DATA_CMD:
                nrf_gpio_pin_clear(PIN_HEEPO_RDY); 
                LOG_DBG("deasserting int");
                intAsserted = false; 
                spiDataBuffer = (spiDataBuffer == buffer0) ? buffer1 : buffer0; 
                break;
            case RESULT_CMD:
                time_item_t *stopTime = k_malloc(sizeof(time_item_t));
                if(stopTime == NULL) {
                    LOG_ERR("unable to allocate memory");
                } else {
                    stopTime->time = k_uptime_get();
                    k_fifo_put(&stopTimeFIFO, stopTime);
                }
                LOG_DBG("received result");
                counter++;
                k_sem_give(&HEEPO_BUSY);                    
                break;
            default:
                LOG_ERR("UNKNOWN COMMAND FROM HEEPO");
        }
        memcpy(&bufferSize, spiDataBuffer, sizeof(bufferSize));
        // LOG_DBG("bufferSize set in buffers= %d", bufferSize);
        status = nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
        LOG_DBG("status in set_spi_buffers_thread_func = %d", status);
    }
    LOG_DBG("calib done, result_received_counter = %d", counter);
    // status = nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));

    while(true) {
        LOG_DBG("started main loop");
        k_sem_take(&HEEPO_XFER_DONE, K_FOREVER);
        LOG_DBG("taken HEEPO_XFER_DONE sem");
        switch(m_rx_buffer_slave[0]) {
            case SEND_DATA_CMD:
                nrf_gpio_pin_clear(PIN_HEEPO_RDY); 
                LOG_DBG("deasserting int");
                intAsserted = false; 
                spiDataBuffer = (spiDataBuffer == buffer0) ? buffer1 : buffer0; 
                break;
            case RESULT_CMD:
                LOG_INF("Received Result.");
                uint32_t i;
                for(i=0; i<MAX_RESULT_SIZE;i++) {
                    LOG_DBG("SPIS rx buffer[%d]: 0x%02x",i+4, m_rx_buffer_slave[i+4]);
                }
                k_sem_give(&HEEPO_BUSY);                    
                break;
            default:
                LOG_ERR("UNKNOWN COMMAND FROM HEEPO");
        }
        memcpy(&bufferSize, spiDataBuffer, sizeof(bufferSize));
        nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    }
    
}


void SPI_Heep_stop_thread(void)
{
    HEEPO_stop_thread_flag = true;
    return;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context)
{
    // Handle the SPIS event
    
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE) {
        // LOG_DBG("NRFX_SPIS_XFER_DONE int");
        LOG_DBG("received %d bytes", p_event->rx_amount);
        k_sem_give(&HEEPO_XFER_DONE);
    }
    else if (p_event->evt_type == NRFX_SPIS_BUFFERS_SET_DONE)
    {
        // LOG_DBG("NRFX_SPIS_BUFFERS_SET_DONE int");
        k_sem_give(&SPI_BUFFERS_SET);
    }
    
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/