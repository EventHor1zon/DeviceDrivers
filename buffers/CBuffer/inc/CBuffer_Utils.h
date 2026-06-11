/****************************************
* \file
* \brief
* \date
* \author
****************************************/

#ifndef CBUFF_UTILS_H
#define CBUFF_UTILS_H

/********* Includes ********************/
#define CBUFFER_MAX_PACKET_VALUES           6
#define CBUFFER_MAX_NAME_LENGTH             24
#define CBUFF_MAX_SEP_BYTES                 8
#define CBUFFER_MAX_PACKET_SIZE_BYTES       256
/********* Definitions *****************/

/** the CBuffer Data read task queue **/
extern QueueHandle_t cbuff_read_queue;


/********** Types **********************/

/** cbuffer_pattern_t
 *  ascii codes for the python struct format characters
 *  (reduced set)
 **/
typedef enum {
    CBUFF_VARTYPE_BOOL      = 0x3F,     /**< '?' **/
    CBUFF_VARTYPE_UINT8     = 0x42,     /**< 'B' **/
    CBUFF_VARTYPE_UINT16    = 0x48,     /**< 'H' **/
    CBUFF_VARTYPE_UINT32    = 0x4C,     /**< 'L' **/
    CBUFF_VARTYPE_INT8      = 0x62,     /**< 'i' **/
    CBUFF_VARTYPE_DOUBLE    = 0x64,     /**< 'd' **/
    CBUFF_VARTYPE_FLOAT     = 0x66,     /**< 'f' **/
    CBUFF_VARTYPE_INT16     = 0x68,     /**< 'h' **/
    CBUFF_VARTYPE_INT32     = 0x6C,     /**< 'l' **/
    CBUFF_VARTYPE_PADBYTE   = 0x78,     /**< 'x' **/
} cbuffer_pattern_t;

/**< Timestamp precision enumeration **/
typedef enum {
    CBUFF_TS_PRECISION_SECOND = 1,      /**< 1s precision **/
    CBUFF_TS_PRECISION_MICROSECOND = 2, /**< 1uS precison **/
} timestamp_precision_t;

typedef enum {
    CBUFF_TASK_NONE,
    CBUFF_TASK_DATA_DISPATCH_AVAILABLE,
    CBUFF_TASK_PACKET_DISPATCH_AVAILABLE,
    CBUFF_TASK_DATA_DISPATCH_CONTINUOUS,
    CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS,
    CBUFF_TASK_DATA_READ,
    CBUFF_TASK_PACKET_READ,
    CBUFF_TASK_MAX,
} cbuffer_task_t;

typedef enum {
    CBUFF_IO_TYPE_NONE,
    CBUFF_IO_TYPE_UART,
    CBUFF_IO_TYPE_I2C,
    CBUFF_IO_TYPE_SPI,
    CBUFF_IO_TYPE_CAN,
    CBUFF_IO_TYPE_HTTP,
    CBUFF_IO_TYPE_WEBSOCKET,
    CBUFF_IO_TYPE_TCP,
    CBUFF_IO_TYPE_MAX,
} cbuffer_data_io_t;

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

/**< member of packet structure **/
typedef struct CBuffer_packet_member {
    /* data */
    uint8_t id;                             /**< id of member **/
    char name[CBUFFER_MAX_NAME_LENGTH + 1]; /**< name of member **/
    uint8_t name_length;                    /**< name length (bytes) **/
    cbuffer_pattern_t p;                    /**< pattern character **/
} cbuffer_pmember_t;

/**< cbuffer packet settings **/
typedef struct cbuffer_packet_settings {
    /* data */
    char pattern[CBUFFER_MAX_PACKET_VALUES + 1]; /**< data pattern characters **/
    uint16_t packet_sz_bytes;                    /**< size of packet data in bytes **/
    uint16_t packet_data_sz_bytes;
    uint8_t packet_index;                   /**< current pattern index **/
    uint8_t packet_elements;                /**< pattern length **/
    bool use_sep_bytes;                     /**< add seperation marker bytes @ end of packet **/
    uint8_t sep_bytes[CBUFF_MAX_SEP_BYTES]; /**< seperation bytes **/
    uint8_t sep_length;                     /**< number of seperator bytes to add **/
    bool use_timestamp;                     /**< add a timestamp to the start of the packet **/
    timestamp_precision_t ts_precision; /**< 0 - second precision (32bit) 1 - microsecond precision
                                           (64bit) **/
    uint8_t timestamp_length;           /**< length of timestamp bytes **/
} cbuffer_pkt_settings_t;


    cbuffer_task_settings_t tx_task; /**< settings for dispatch/receive tasks **/
    cbuffer_task_settings_t rx_task;


/** To trigger the cbuffer read, should send a queue request
 *      with an address to read from and the read length
 *  **/
typedef struct CBufferReadTrigger {
    void *read_address;           /** The address to read from    **/
    uint32_t read_len_bytes;      /** Bytes to read               **/
    uint32_t row_size;            /** Row size (TODO: play with)  **/
    SemaphoreHandle_t *data_lock; /** Optional semaphore for data access **/
    CBuff buff;                   /** CBuffer handle              **/
} CBufferReadTrigger_t;


/******** Function Definitions *********/

/**
 * \brief: Configure the packet storage system
 *          (Does not enable, packet storage is enabled with load packet)
 * \param  use_sep - add seperator bytes between packets
 * \param  sep_bytes - a pointer to an array of seperator bytes
 * \param  sep_length - length of seperator bytes (max = 8)
 * \param  use_timestamp - add a timestamp to the packet
 * \return ptr to buffer or NULL on error
 **/
status_t cbuffer_config_packet(
    CBuff handle,
    bool use_sep,
    uint8_t *sep_bytes,
    uint8_t sep_length,
    bool use_timestamp,
    timestamp_precision_t ts_precision);

/**
 *  \brief Gets the length of the full packet including timestamp and
 *          seperation bytes
 *  \param handle handle to the cbuffer
 *  \param len pointer to storage for length
 *  \return ESP_OK or error
 **/
status_t cbuffer_get_full_packet_length(CBuff handle, uint16_t *len);

/**
 * \brief: Load a new packet structure. Resets the CBuffer and
 *          begins recording packets in a structured manner.
 * \param handle cbuffer handle
 * \param values_per_packet - number of measurements making up this packet
 * \param pattern - a string of ascii values found in cbuffer_pattern_t
 *                  must have same len as values_per_packet
 * \param names_array - an array of names for the packet values
 *                      must have array length = values_per_packet
 * \param IDs       - an array of uint8_t ids: must be same length as values_per_packet
 *                      and all values must be greater than zero
 * \return ESP_OK or error
 **/
status_t cbuffer_load_packet(
    CBuff handle,
    uint8_t values_per_packet,
    char *pattern,
    char **names_array,
    uint8_t *ids);

/**
 * \brief: Read from i2c into a pattern Cbuffer - wrote this in order to read directly
 *          from large device fifos directly into cbuffer, from there to be stored to
 *          xfered to another process.
 *          Unfortunately a limitation is conversion not able to be transfered cross-process
 *          without being a massive pita. So data coming out of the fifo is stored as-is, and should
 *          match the loaded packet structure for the cbuffer.
 *          *** This is a bit experimental! ***
 * \param handle - cbuffer handle
 * \param trx - info about i2c transaction
 * \return esp_ok or error
 **/
// status_t cbuffer_read_i2c_pattern_block(CBuff handle, gcd_transaction_t *trx);

/**
 *  \brief: Another experimental implimentation of i2c fifo->cbuffer, this uses lower-level i2c
 *queueing to read bytes directly into the cbuffer and manually increments the pointers. This
 *          _should_ be pretty fast?
 *
 *  \param handle - cbuffer handle
 *  \param bus - i2c bus
 *  \param device_addr - as you might expect
 *  \param reg_addr - ""
 *  \param len ""
 *  \return ESP_OK or error
 **/
status_t cbuffer_read_i2c_block(
    CBuff handle,
    uint8_t bus,
    uint8_t device_addr,
    uint8_t reg_addr,
    uint32_t len);

/**
 * \brief Deletes the packet configuration and deactivates the pattern storage
 * \param handle cbuffer handle
 * \return ESP_OK
 **/
status_t cbuffer_clear_packet(CBuff handle);


/**
 * \brief gets the full packet pattern in string form,
 *          compatible with python struct (minus endian-ness)
 *  \param handle handle
 *  \param pattern a pointer to string storage
 *  \return ESP_OK or error
 **/
status_t cbuffer_get_pattern(CBuff handle, char *pattern);


/**
 * \brief: read a formatted packet from CBuffer
 * \param handle - cbuffer handle
 * \param data - pointer to the data storage
 **/
status_t cbuffer_read_packet(CBuff handle, void *data);

/**
 * \brief: Write a formatted packet to the CBuffer
 * \param handle - cbuffer handle
 * \param data - pointer to the data
 * \param length - length of the data = is this needed?
 **/
status_t cbuffer_write_packet(CBuff handle, void *data, uint32_t length);

/**
 * \brief: Get num unread packets
 * \param handle - cbuffer handle
 * \param data - pointer to the data storage
 **/
status_t cbuffer_get_available_packets(CBuff handle, uint32_t *avail);



////////////////////////////////////////////////////////////
//      Assorted utilities
////////////////////////////////////////////////////////////

/** \brief cbuffer_dump_packets_uart - like it sounds, this function
 *          writes organised packets of data to the uart bus of your choice!
 *          This dumps all the packets available in the buffer
 *          Especially useful for PC processing of sensor data :)
 *  \param handle - handle for the cbuffer
 *  \param uart_bus - the uart bus to write data from
 *
 ***/
status_t cbuff_dump_packets_uart(CBuff handle, uint8_t uart_bus);

/** \brief cbuffer_dump_uart - sends data from the cbuffer to the
 *              uart queue. This uses the non-blocking uart function
 *              so might not write all the data
 *  \param handle - handle for the cbuffer
 *  \param bus - the uart bus to write data to
 *  \param len - length of data to send to queue
 *  \return ESP_OK or ESP_NO_MEM if bytes queued < len
 **/
status_t cbuff_dump_uart(CBuff handle, uint8_t bus, uint32_t len);

/**
 * \brief cbuffer start task - starts a specific i/o task. Task is either contunous tx/rx
 *          or condition specific single-run (such as sending all available data).
 *          If single-run, this task will delete itself on completion
 *          Can also be used to cancel a running task with type == NO_TASK
 *  \param handle - the cbuffer handle
 *  \param io_type - the io type to tx/rx - one of cbuffer_data_io_t
 *  \param io_bus - the io bus to rx/rx - this function does not error check this value
 *  \param addr - an optional address for tx/rx from/to slave - truncated to 8-bit for spi/i2c
 *  \param type - type of task - one of cbuffer_task_t
 *  \param chunk_sz - chunk size to tx/rx - if using packet task type, this is not required
 *
 *  \return ESP_OK or error
 **/
status_t cbuffer_start_task(
    CBuff handle,
    cbuffer_data_io_t io_type,
    uint8_t io_bus,
    uint8_t addr,
    cbuffer_task_t type,
    uint16_t chunk_sz);


#endif /* CBUFF_UTILS_H */
