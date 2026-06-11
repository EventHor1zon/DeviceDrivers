/***************************************
 * \file     CircularBuffer.c
 * \brief    A simple library for creating & using circular buffers
 *           made mainly to entertain myself during this FUCKING LOCKDOWN
 *           Includes functionality to alert tasks when buffer nearing full
 *           to facilitate streaming data.
 *           v1.0  - basic complete
 *           v2.0  - add new functionality: packets
 *                   Allow user to specify a pattern which will be saved. The pattern
 *                   should consist of a string of python-style struct descriptors
 *                   The packet descriptor should be adhered to in order to save data correctly.
 *                   The packet also adds a sequence of separator bytes between packets so that the
 *                   Data can be more easily unpacked externally.
 *                   This will allow high-rate streaming of data inside a CBuffer to a python script
 *                   And also nicely formatted dumping of CBuffer data to console, SD card or file.
 *
 *           Refactor 1:
 *               TODO: Apply lessons from software design patterns -
 *                       have a translator for input/output streams
 *                       think of a smarty-pants way to handle components
 *                       interface with a cbuffer
 *
 *
 * \date     Feb 2021
 * \author   RJAM
 ****************************************/

/********* Includes *******************/

/**
 *  HELLO I.T, HAVE YOU TRIED TURNING IT OFF AND ON AGAIN?
 *
 **/

#include "./inc/CircularBuffer.h"

#ifdef CONFIG_USE_EVENTS
#include "port/event.h"
#endif

/****** Function Prototypes ***********/

TaskHandle_t receiveTaskHandle;

// static bool buffer_is_full(CBuff handle);

// static bool buffer_is_empty(CBuff handle);

/** \brief writing length of data to buffer will overrun the end
 *          of the bufferm circling back to the start
 *  \param handle the cbuffer handle
 *  \param incomming_sz number of bytes to be written
 *  \return boolean
 **/
static uint8_t buffer_will_overrun(CBuff handle, uint32_t incomming_sz);

static uint32_t buffer_free_bytes(CBuff handle);

static uint32_t buffer_bytes_until_end(CBuff handle, bool read);

static uint8_t buffer_will_overwrite(CBuff handle, uint32_t incomming_sz);

static void cbuffer_write_ll(CBuff handle, void *data, uint32_t length);

static void cbuffer_read_ll(CBuff handle, void *data, uint32_t len);

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static void *__memcpy(void *dest, void *source, size_t size) __attribute__((weak))
{
    uint8_t *d = dest;
    const uint8_t *s = source;
    while (size--) {
        *d++ = *s++;
    };
    return dest;
}

/** \return Number of free bytes left in buffer **/
static inline uint32_t buffer_free_bytes(CBuff handle)
{
    uint32_t free = (handle->write_ptr > handle->read_ptr) ?  // if write pointer is ahead of
                                                              // read...
                        (handle->buffer_end - handle->write_ptr)
                            + ((handle->read_ptr - 1) - handle->buffer_start)
                                                           :           // go round the buffer
                        ((handle->read_ptr - 1) - handle->write_ptr);  // else go up to read-1
    return free;
}

/** \return bytes available between read and write pointers **/
inline uint32_t buffer_unread_bytes(CBuff handle)
{
    uint32_t avail =
        (handle->read_ptr > handle->write_ptr ? (
             (handle->buffer_end - handle->read_ptr) + ((handle->write_ptr) - handle->buffer_start))
                                              : (handle->write_ptr - handle->read_ptr));
    return avail;
}

/** \return Non-zero if write will overwrite unread data **/
static inline uint8_t buffer_will_overwrite(CBuff handle, uint32_t incomming_sz)
{
    return incomming_sz > buffer_free_bytes(handle);
}

/** \return Non-zero if buffer write will run past the buffer end**/
static inline uint8_t buffer_will_overrun(CBuff handle, uint32_t incomming_sz)
{
    return ((handle->write_ptr + incomming_sz) > handle->buffer_end);
}

/** bytes from pointer to end of buffer **/
static uint32_t buffer_bytes_until_end(CBuff handle, bool read)
{
    uint32_t bytes = 0;
    if (read) {
        bytes = handle->buffer_end - handle->read_ptr;
    } else {
        bytes = handle->buffer_end - handle->write_ptr;
    }
    return bytes;
}

/** write data of length - will follow circular write & increment the write handle **/
static void cbuffer_write_ll(CBuff handle, void *data, uint32_t length)
{
    if (buffer_will_overrun(handle, length)) {
        uint32_t first_write = buffer_bytes_until_end(handle, 0);
        __memcpy(handle->write_ptr, data, first_write);
        handle->write_ptr = handle->buffer_start;
        _memcpy(handle->write_ptr, (data + first_write), (length - first_write));
        assert(length > first_write);
        handle->write_ptr += (length - first_write);
    } else {
        _memcpy(handle->write_ptr, data, length);
        handle->write_ptr += length;
    }
    return;
}

/** read data of length **/
static void cbuffer_read_ll(CBuff handle, void *data, uint32_t len)
{
    uint32_t bytes_until_end = buffer_bytes_until_end(handle, 1);
    if (len > bytes_until_end) {
        _memcpy(data, handle->read_ptr, bytes_until_end);
        handle->read_ptr = handle->buffer_start;
        _memcpy((data + bytes_until_end), handle->read_ptr, (len - bytes_until_end));
        handle->read_ptr += (len - bytes_until_end);
    } else {
        _memcpy(data, handle->read_ptr, len);
        handle->read_ptr += len;
    }
}

#ifdef CONFIG_USE_EVENTS

/** Emit events for user handling
 *  to prevent numerous event raising, clear the event once raised
 *  User will have to re-implement the event when reading the buffer
 *  is finished
 **/
static status_t emit_nearfull_event(CBuff handle)
{
    status_t err = STATUS_OK;
    err = event_post(
        handle->event_settings.loop,
        PM_EVENT_BASE,
        CBUFF_EVENTCODE_NEARFULL,
        handle,
        sizeof(CBuff),
        pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    log_info("Posting nearful event %u\n\n", err);
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_NEARFULL);
    return err;
}

static status_t emit_full_event(CBuff handle)
{
    status_t err = STATUS_OK;
    err = event_post(
        handle->event_settings.loop,
        PM_EVENT_BASE,
        CBUFF_EVENTCODE_FULL,
        handle,
        sizeof(CBuff),
        pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_FULL);
    return err;
}

static status_t emit_overwrite_event(CBuff handle)
{
    status_t err = STATUS_OK;
    err = event_post(
        handle->event_settings.loop,
        PM_EVENT_BASE,
        CBUFF_EVENTCODE_OVERWRITE,
        handle,
        sizeof(CBuff),
        pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_OVERWRITE);
    return err;
}

static status_t emit_empty_event(CBuff handle)
{
    status_t err = event_post(
        handle->event_settings.loop,
        PM_EVENT_BASE,
        CBUFF_EVENTCODE_EMPTY,
        handle,
        sizeof(CBuff),
        pdMS_TO_TICKS(CONFIG_EVENTPOST_WAIT_MS));
    handle->event_settings.event_mask &= ~(CBUFF_EVENT_EMPTY);
    return err;
}

#endif

/****** Global Data *******************/

/****** Global Functions *************/

CBuff cbuffer_create(CBuffer_Handle_t *handle, CBuffer_init_t *init)
{
    status_t err = STATUS_OK;

    handle->buffer_start = init->buffer;
    handle->buffer_end = (init->buffer + init->size);
    handle->buffer_len = init->size;
    handle->sem = init->sem;

    handle->write_ptr = handle->buffer_start;
    handle->read_ptr = handle->buffer_start;
    handle->allow_overwrite = init->allow_ovr;

    return handle;
}

#ifdef CONFIG_USE_EVENTS

status_t cbuffer_config_events(
    CBuff handle,
    bool use_events,
    event_loop_handle_t loop,
    uint8_t event_flags,
    uint32_t nearfull_val)
{
    status_t err = STATUS_OK;

    if (use_events) {
        if (loop == NULL) {
            err = STATUS_ERR_INVALID_ARG;
        } else if ((event_flags & CBUFF_EVENT_NEARFULL) && nearfull_val > handle->buffer_len) {
            err = STATUS_ERR_INVALID_ARG;
        } else {
            handle->use_events = true;
            handle->event_settings.loop = loop;
            handle->event_settings.event_mask = event_flags;
            handle->event_settings.nearfull_val = (event_flags & CBUFF_EVENT_NEARFULL)
                                                      ? nearfull_val
                                                      : 0;
        }
    } else {
        memset(&handle->event_settings, 0, sizeof(cbuffer_events_t));
        handle->use_events = false;
    }

    return err;
}

status_t cbuffer_set_event_mask(CBuff handle, uint8_t event_mask)
{
    handle->event_settings.event_mask = event_mask;

    if (event_mask) {
        handle->use_events = true;
    } else {
        handle->use_events = false;
        s
    }
    return STATUS_OK;
}

#endif

status_t cbuffer_write(CBuff handle, void *const data, uint32_t *wrt_len)
{
    if (wrt_len > handle->buffer_len) {
        return STATUS_ERR_INVALID_ARG;
    }
    if (!data || !handle) {
        return STATUS_ERR_NO_MEM;
    }
    if (handle->sem && xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        return STATUS_ERR_TIMEOUT;
    }

    uint32_t _len = (buffer_will_overwrite(handle, *wrt_len) && !handle->allow_overwrite)
                        ? buffer_free_bytes(handle)
                        : *wrt_len;

    cbuffer_write_ll(handle, data, _len);

#ifdef CONFIG_USE_EVENTS
    /** emit the overwrite event **/
    if (handle->use_events) {
        if (handle->event_settings.event_mask & CBUFF_EVENT_OVERWRITE && handle->allow_overwrite
            && buffer_will_overwrite(handle, wrt_len))
        {
            emit_overwrite_event(handle);
        }

        uint32_t unread_bytes = buffer_unread_bytes(handle);

        /** emit the 'full' event **/
        if (handle->event_settings.event_mask & CBUFF_EVENT_FULL && buffer_unread_bytes == 0) {
            emit_full_event(handle);
        }

        if ((handle->event_settings.event_mask & CBUFF_EVENT_NEARFULL)
            && (unread_bytes >= handle->event_settings.nearfull_val))
        {
            emit_nearfull_event(handle);
        }
    }
#endif
    if (handle->sem) {
        xSemaphoreGive(handle->sem);
    }
    *wrt_len = _len;
    return STATUS_OK;
}

status_t cbuffer_read(CBuff handle, void *const buffer, uint32_t *length)
{
    if (!buffer) {
        return STATUS_ERR_NO_MEM;
    }
    if (handle->sem && xSemaphoreTake(handle->sem, pdMS_TO_TICKS(CBUFFER_SEM_WAIT_MS)) != pdTRUE) {
        return STATUS_ERR_TIMEOUT;
    }

    uint32_t unread = buffer_unread_bytes(handle);
    uint32_t _len = (*length > unread) ? unread : *length;

    cbuffer_read_ll(handle, buffer, _len);

    if (handle->sem) {
        xSemaphoreGive(handle->sem);
    };

#ifdef CONFIG_USE_EVENTS

    if (!ret && handle->use_events && buffer_unread_bytes(handle) < 1
        && handle->event_settings.event_mask & CBUFF_EVENT_EMPTY)
    {
        emit_empty_event(handle);
    }

#endif
    *length = _len;
    return STATUS_OK;
}

uint32_t cbuffer_unread(CBuff handle)
{
    return buffer_unread_bytes(handle);
}

uint32_t cbuffer_available_space(CBuff handle)
{
    return buffer_free_bytes(handle);
}