/****************************************
 * \file     CircularBuffer.h
 * \brief    Circular Buffer header file
 * \date
 * \author
 ****************************************/

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#define CBUFFER_SEMTAKE_TIMEOUT 200 /**< in ms, doesn't need to be big **/

/********* Includes ********************/
#include <stdint.h>

/********* Definitions *****************/

#define CBUFFER_MAX_BUFFER_SIZE 8192
#define CBUFFER_CHUNK_READ_SIZE 256

#define CBUFFER_CONFIG_DISPATCH_YIELD_MS 5
#define CBUFFER_CONFIG_WAIT_PKT_MS       50
#define CBUFFER_CONFIG_RX_QUEUE_LEN      8

#define CBUFFER_SEM_WAIT_MS 100

#ifdef CONFIG_USE_EVENTS
#include "port/events.h"
#define CONFIG_EVENTPOST_WAIT_MS 200

#define CBUFF_EVENT_BASE 0x70

#define CBUFF_EVENT_FULL         (1 << 1)
#define CBUFF_EVENT_NEARFULL     (1 << 2)
#define CBUFF_EVENT_OVERWRITE    (1 << 3)
#define CBUFF_EVENT_EMPTY        (1 << 4)
#define CBUFF_EVENT_TX_TASK_DONE (1 << 5)
#define CBUFF_EVENT_RX_TASK_DONE (1 << 6)

#define CBUFF_EVENTCODE_FULL      (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_FULL)
#define CBUFF_EVENTCODE_NEARFULL  (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_NEARFULL)
#define CBUFF_EVENTCODE_OVERWRITE (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_OVERWRITE)
#define CBUFF_EVENTCODE_EMPTY     (CBUFF_EVENT_BASE << 16 | CBUFF_EVENT_EMPTY)

typedef void (*TRANSLATION_F)(void *buff, void *args);

/**< stuct to define cbuffer event settings **/
typedef struct cbuffer_event_settings {
    /* data */
    event_loop_t loop;     /**< the event loop to post to **/
    uint8_t event_mask;    /**< maask of events defined above **/
    uint32_t nearfull_val; /**< nearly full value to alert on **/
} cbuffer_events_t;

#endif /** CONFIG_USE_EVENTS **/

typedef enum __attribute__((short)) {
    STATUS_OK = 0,
    STATUS_ERR_INVALID_ARG,
    STATUS_ERR_NO_MEM,
    STATUS_ERR_TIMEOUT,

} status_t;

/** the cbuffer handle data structure **/
typedef struct CBuffer_Handle {
    void *buffer_start; /**< buffer start - start of buffer addr **/
    void *buffer_end;   /**< buffer end - end of buffer addr **/

    void *read_ptr;  /**< start of data in buffer **/
    void *write_ptr; /**< end of data in buffer **/

    uint32_t buffer_len;     /**< total length of buffer **/
    uint8_t allow_overwrite; /**< allow overwrite of unread data **/

#ifdef CONFIG_USE_EVENTS
    /** cbuffer events allow for storage/dumping on full/nearfull **/
    uint8_t use_events;              /**< raise events **/
    cbuffer_events_t event_settings; /**< event settings **/
#endif
    void *sem; /**< semaphore handle  **/

} CBuffer_Handle_t;

typedef CBuffer_Handle_t *CBuff;

/** CBuffer init data - keep this simple **/
typedef struct CBuffer_init {
    uint32_t size; /**< buffer size, in bytes **/
    uint8_t *buffer;
    uint8_t allow_ovr;
    void *sem; /** generic semaphore pointer */
} CBuffer_init_t;

/********** Types **********************/

/******** Function Definitions *********/

/**
 * \brief: Allocate a new circular buffer controller
 * \param handle - pointer to a CBuffer_Handle_t struct
 * \param init - pointer to a configured CBuffer_init_t struct
 * \return Pointer to CBuffer control struct (same as supplied pointer) or NULL on error
 **/
CBuff cbuffer_create(CBuffer_Handle_t *handle, CBuffer_init_t *init);

/**
 * \brief: write data to the buffer
 *          If write length would overwrite unread data and
 *          overwrites are not enabled, the write length
 *          will be the available number of bytes
 * \param handle - ptr to the cbuffer handle
 * \param data -  const ptr to the data to write
 * \param wrt_len [in] pointer to length of data (in bytes) to write
 *               [out] length of data written
 * \return ESP_OK or error
 **/
status_t cbuffer_write(CBuff handle, void *const data, uint32_t *wrt_len);

/**
 * \brief: read data from the buffer
 *          if read length greater than number of unread bytes
 *          then read length will be number of unread bytes
 * \param handle - ptr to the cbuffer handle
 * \param buffer - ptr to read data into
 * \param length [in] length of data (in bytes) to read
 *               [out] length of data read (in bytes)
 * \return ESP_OK or error
 **/
status_t cbuffer_read(CBuff handle, void *const buffer, uint32_t *length);

/**
 * \brief: sets buffer contents to 0
 * \param handle see above
 * \return ESP_OK
 **/
status_t cbuffer_clear(CBuff handle);

/***
 * \brief resets the r/w pointer locations to start
 * \param handle see above
 * \return ESP_OK
 **/
status_t cbuffer_reset_pointers(CBuff handle);

/**
 *  \brief does the above 2 functions together
 *  \param handle see above
 *  \return ESP_OK
 **/
status_t cbuffer_reset_buffer(CBuff handle);

uint32_t cbuffer_unread_bytes(CBuff handle);

uint32_t cbuffer_available_space(CBuff handle);

#ifdef CONFIG_USE_EVENTS
/**
 * \brief configure the event settings
 * \param handle see above
 * \param use_events enable events for cbuffers
 * \param loop  the event loop to post to
 * \param event_flags an OR'd byte of CBuffer event flags (see above)
 * \param nearfull_val - the size (bytes) at which nearly full event triggers
 * \return ESP_OK or error
 **/
status_t cbuffer_config_events(
    CBuff handle,
    bool use_events,
    esp_event_loop_handle_t loop,
    uint8_t event_flags,
    uint32_t nearfull_val);

status_t cbuffer_set_event_mask(CBuff handle, uint8_t event_mask);

#endif /** CONFIG_USE_EVENTS **/

#endif /* CIRCULAR_BUFFER_H */
