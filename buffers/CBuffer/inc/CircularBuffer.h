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

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/********* Definitions *****************/

#define CBUFFER_MAX_BUFFER_SIZE 8192
#define CBUFFER_CHUNK_READ_SIZE 256

#define CBUFFER_CONFIG_DISPATCH_YIELD_MS 5
#define CBUFFER_CONFIG_WAIT_PKT_MS       50
#define CBUFFER_CONFIG_RX_QUEUE_LEN      8

#define CBUFFER_SEM_WAIT_MS 100

#ifdef CONFIG_USE_EVENTS
#include "esp_event_base.h"

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
    esp_event_loop_handle_t loop; /**< the event loop to post to **/
    uint8_t event_mask;           /**< maask of events defined above **/
    uint32_t nearfull_val;        /**< nearly full value to alert on **/
} cbuffer_events_t;

#endif /** CONFIG_USE_EVENTS **/

typedef struct CBuffer_tasks {
    bool is_complete;            /**< if task is complete - only used for single-run tasks **/
    TaskHandle_t task_handle;    /**< handle for the task **/
    cbuffer_task_t current_task; /**< the action to complete **/
    cbuffer_data_io_t io_type;   /**< the io type **/
    uint8_t io_bus;              /**< the io bus **/
    uint32_t addr;               /**< an io address (optional) **/
    uint32_t chunk_sz;           /**< size of data to dispatch **/
    bool active;                 /**< the task is currently active **/
    bool continuous;             /**< continuous task **/
} cbuffer_task_settings_t;

/** the cbuffer handle data structure **/
typedef struct CBuffer_Handle {
    void *buffer_start; /**< buffer start - start of buffer addr **/
    void *buffer_end;   /**< buffer end - end of buffer addr **/

    void *read_ptr;  /**< start of data in buffer **/
    void *write_ptr; /**< end of data in buffer **/

    uint32_t buffer_len; /**< total length of buffer **/
    uint16_t data_len;   /**< total length of data **/

    bool allow_overwrite; /**< allow overwrite of unread data **/
    bool is_claimed;      /**< sem is held **/

    /** In order to save structured data implement packet system **/
    bool use_packets;                                     /**< use a packet_style packing **/
    cbuffer_pmember_t packets[CBUFFER_MAX_PACKET_VALUES]; /**< pattern to load **/
    cbuffer_pkt_settings_t pkt_settings;                  /**< settings for pattern builder **/
    uint16_t pkts_available;                              /**< number of packets in buffer
                                                              - read/write packets will increment/decrement this number
                                                              mixing other functions can corrupt **/

#ifdef CONFIG_USE_EVENTS
    /** cbuffer events allow for storage/dumping on full/nearfull **/
    bool use_events;                 /**< raise events **/
    cbuffer_events_t event_settings; /**< event settings **/
#endif
    SemaphoreHandle_t sem; /**< semaphore handle  **/

} CBuffer_Handle_t;

typedef CBuffer_Handle_t *CBuff;

/** CBuffer init data - keep this simple **/
typedef struct CBuffer_init {
    uint32_t size; /**< buffer size, in bytes **/
    uint8_t *buffer;
    bool allow_ovr;
    uint8_t malloc_caps;
} CBuffer_init_t;

/********** Types **********************/

/******** Function Definitions *********/

uint32_t buffer_unread_bytes(CBuff handle);

/**
 * \brief: Allocate a circular buffer of Size
 * \param size - size of buffer requested
 * \param  allow_overrun - allow overwriting unread data
 * \return ptr to buffer or NULL on error
 **/
CBuff cbuffer_create(CBuffer_init_t *init);

status_t cbuffer_set_event_mask(CBuff handle, uint8_t event_mask);

/**
 * \brief: free a cbuffer
 * \param handle - pointer to cbuffer handle
 * \return ESP_OK or error
 **/
status_t cbuffer_destroy(CBuff handle);

/**
 * \brief: write data to the buffer
 * \param handle - ptr to the cbuffer handle
 * \param data - ptr to the data to write
 * \param length - length of data (in bytes) to write
 * \return ESP_OK or error
 **/
status_t cbuffer_write(CBuff handle, void *data, uint32_t length);

/**
 * \brief: read data from the buffer
 * \param handle - ptr to the cbuffer handle
 * \param buffer - ptr to read data into
 * \param length - length of data (in bytes) to read
 * \return ESP_OK or error
 **/
status_t cbuffer_read(CBuff handle, void *buffer, uint32_t length);

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

#endif /** CONFIG_USE_EVENTS **/

#endif /* CIRCULAR_BUFFER_H */
