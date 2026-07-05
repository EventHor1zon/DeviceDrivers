/***************************************
 * \file .c
 * \brief
 *
 * \date
 * \author
 ****************************************/

/********* Includes *******************/
#include "A4988_Driver.h"
#include "port/error_types.h"
#include "port/port_log.h"
#include "port/port_malloc.h"
#include "port/port_types.h"
#include "port/rtos_primitives/include/port_rtos.h"
#include "string.h"

const char *DEV_TAG = "A4988 Driver";
/** keep the task handle static as we only instatiate it
 * once
 */
static task_type_t a4988_task_handle = NULL;
static QueueHandle_t a4988_cmd_queue = NULL;
static uint8_t driver_running = 0;
static uint8_t num_devices = 0;
stacktype_t task_stack[5012];
stacktype_t queue_stack[1024];

#ifdef CONFIG_USE_PERIPH_MANAGER

const parameter_t a4988_param_map[a4988_param_len] = {
    {"step size",
     1,
     &a4988_get_stepsize,
     &a4988_set_stepsize,
     NULL,
     DATATYPE_UINT8,
     7,
     (GET_FLAG | SET_FLAG)},
    {"step wait",
     2,
     &a4988_get_step_delay,
     &a4988_set_step_delay,
     NULL,
     DATATYPE_UINT16,
     0xFFFF,
     (GET_FLAG | SET_FLAG)},
    {"queued steps",
     3,
     &a4988_get_queued_steps,
     &a4988_set_queued_steps,
     NULL,
     DATATYPE_UINT16,
     0xFFFF,
     (GET_FLAG | SET_FLAG)},
    {"sleep state",
     4,
     &a4988_get_sleepstate,
     &a4988_set_sleepstate,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"enable",
     5,
     &a4988_get_enable,
     &a4988_set_enable,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"direction",
     6,
     &a4988_get_direction,
     &a4988_set_direction,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"clear queue", 7, NULL, NULL, &a4988_clear_step_queue, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"step", 8, NULL, NULL, &a4988_step, DATATYPE_NONE, 0, (ACT_FLAG)},
    {"reset", 9, NULL, NULL, &a4988_reset, DATATYPE_NONE, 0, (ACT_FLAG)},
};

const peripheral_t a4988_periph_template = {
    .handle = NULL,
    .param_len = a4988_param_len,
    .params = a4988_param_map,
    .peripheral_name = "A4988",
    .peripheral_id = 0,
    .periph_type = PTYPE_IO,
};

#endif /** CONFIG_USE_PERIPH_MANAGER **/

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

void pulse_timer_callback(void *args)
{
    A4988_DEV dev = (A4988_DEV)args;
    gpio_set_level(dev->step, 0);
}

void a4988_step_timer_callback(TimerHandle_t tmr)
{
    BaseType_t higherPrioWoken = pdFALSE;
    A4988_DEV dev = pvTimerGetTimerID(tmr);
    a4988_msg_t msg = {.cmd = A4988_CMD_STEP_TMR, .dev = dev};

    xQueueSendFromISR(a4988_cmd_queue, &msg, &higherPrioWoken);

    return;
}

/**
 * @brief driver task. Runs a single instance which controls multiple devices
 *  keep the code here nice and fast, no delays. Just resetting timers etc.
 *  Waits on message from the command queue and takes the appropriate action.
 *
 * @param args - unused
 * @return ** void
 */
static void a4988_driver_task(void *args)
{
    a4988_msg_t msg = {0};
    BaseType_t rx = pdFALSE;

    while (1) {
        rx = port_queue_get(a4988_cmd_queue, &msg, portMAX_DELAY);

        if (rx == pdTRUE) {
            /** retrieve the device handle **/
            A4988_DEV dev = (A4988_DEV)msg.dev;

            switch (msg.cmd) {
                /** step timer expired, take another step **/
                case A4988_CMD_STEP_TMR:
                    if (dev->steps_queued > 0) {
                        /** take the queued step, then start the timer & wait for notify **/
#ifdef DEBUG_MODE
                        log_info(DEV_TAG, "Stepping... (steps in queue: %u)", dev->steps_queued);
#endif
                        a4988_step(dev);
                        if (xTimerStart(dev->step_timer, 0) != pdPASS) {
                            log_error(DEV_TAG, "Error restarting timer");
                        }
                    }
                    break;

                case A4988_CMD_UPDATE_PERIOD:
                    if (xTimerStop(dev->step_timer, A4988_CONFIG_SHORT_WAIT) != pdPASS
                        || xTimerChangePeriod(
                               dev->step_timer,
                               dev->step_wait,
                               A4988_CONFIG_SHORT_WAIT)
                               != pdPASS)
                    {
                        log_error(DEV_TAG, "Error changing timer period");
                    } else {
                        if (dev->steps_queued > 0
                            && xTimerStart(dev->step_timer, A4988_CONFIG_SHORT_WAIT) != pdPASS) {
                            log_error(DEV_TAG, "Error restarting timer");
                        }
                    }

                default: break;
            }
        }
    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/

status_t a4988_start_driver(
    uint32_t task_priority,
    stacktype_t *task_buffer,
    uint32_t stack_size,
    void *const tcb)
{
    /** Create the task and command queue - this should only be called once **/
    if (!err && a4988_task_handle == NULL) {
        if (port_task_create(
                a4988_driver_task,
                "a4988_driver_task",
                stack_size,
                NULL,
                task_priority,
                task_buffer,
                tcb)
            != STATUS_OK)
        {
            log_error(DEV_TAG, "Error starting dev task!");
            err = STATUS_ERR_NO_MEM;
        }
    }

    if (!err && a4988_cmd_queue == NULL) {
        a4988_cmd_queue = port_queue_create(sizeof(a4988_cmd_t), A4988_CONFIG_QUEUE_LEN);
        if (a4988_cmd_queue == NULL) {
            log_error(DEV_TAG, "Error creating queue");
            err = STATUS_ERR_NO_MEM;
        }
    }
}

#ifdef CONFIG_DRIVERS_USE_HEAP
A4988_DEV a4988_init(a4988_init_t *init)
{
#else
A4988_DEV a4988_init(A4988_DEV dev, a4988_init_t *init)
{
#endif

    status_t err = STATUS_OK;
    TimerHandle_t timer = NULL;
    gpio_config_t pins = {0};

    if (num_devices >= A4988_CONFIG_MAX_SUPPORTED_DEVICES) {
        err = STATUS_ERR_NOT_SUPPORTED;
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    /** initialise the handle **/
    A4988_DEV dev = heap_caps_calloc(1, sizeof(a4988_handle_t), MALLOC_CAP_DEFAULT);
    if (dev == NULL) {
        log_error(DEV_TAG, "Error assigning memory for the handle");
        err = STATUS_ERR_NO_MEM;
    }
#else
    memset(dev, 0, sizeof(a4988_handle_t));
#endif
    if (err == STATUS_OK) {
        dev->step_pulse_len = A4988_DEFAULT_STEP_PULSE_LEN;
        dev->step_wait = A4988_DEFAULT_STEP_DELAY;
        dev->is_enabled = true;
        dev->is_sleeping = false;
        dev->direction = false;
        dev->step_size = init->step_size;
        dev->rst = init->rst;
        dev->sleep = init->sleep;
        dev->enable = init->enable;
        dev->step = init->step;
        dev->dir = init->dir;
        dev->ms1 = init->ms1;
        dev->ms2 = init->ms2;
        dev->ms3 = init->ms3;
    }

    /** check & initialise the mandatory pins **/

    if (init->rst == 0 || init->dir == 0 || init->step == 0) {
        log_error(DEV_TAG, "Error - missing important pins!");
        err = STATUS_ERR_INVALID_ARG;
    }

    if (!err) {
        pins.intr_type = GPIO_INTR_DISABLE;
        pins.mode = GPIO_MODE_OUTPUT;
        pins.pin_bit_mask = ((1 << init->rst) | (1 << init->step) | (1 << init->dir));
        err = gpio_config(&pins);

        if (err) {
            log_error(DEV_TAG, "error setting up GPIO pins");
        } else {
            /** set pin states **/
            gpio_set_level(dev->rst, 1);
            gpio_set_level(dev->dir, 0);
            gpio_set_level(dev->step, 0);
        }
    }

    if (!err) {
        uint32_t pinmask = 0;
        /** check if the driver controls the enable pin **/
        if (init->enable) {
            pinmask |= (1 << init->enable);
            dev->_en = true;
        }
        /** check if the driver controls the sleep pin **/
        if (init->sleep) {
            pinmask |= (1 << init->sleep);
            dev->_sleep = true;
        }
        /** check if the driver controls all 3 ms pins **/
        if (init->ms1 && init->ms2 && init->ms3) {
            pinmask |= (1 << init->ms1);
            pinmask |= (1 << init->ms2);
            pinmask |= (1 << init->ms3);
            dev->_ms = true;
        }

        /** initialise these as outputs **/
        if (pinmask > 0) {
            pins.pin_bit_mask = pinmask;
            err = gpio_config(&pins);
            if (err) {
                log_error(DEV_TAG, "Error setting up secondary GPIO pins {%u}", err);
            } else {
                /** set pin levels **/
                if (dev->_en) {
                    gpio_set_level(dev->enable, 0);
                }
                if (dev->_sleep) {
                    gpio_set_level(dev->sleep, 1);
                }
                if (dev->_ms) {
                    gpio_set_level(dev->ms1, 0);
                    gpio_set_level(dev->ms2, 0);
                    gpio_set_level(dev->ms3, 0);
                    dev->step_size = 0;
                }
            }
        }
    }

    if (!err) {
        log_info(DEV_TAG, "Succesfully started the A4988 Driver!");
        dev->index = num_devices;
        num_devices++;
    } else {
        log_error(DEV_TAG, "Error starting the driver (Error: %u)", err);
#ifdef CONFIG_DRIVERS_USE_HEAP
        if (dev != NULL) {
            heap_caps_free(dev);
        }
#endif
    }

    return dev;
}

status_t a4988_step(A4988_DEV dev)
{
    /** TODO: Set a timer to unlock pin so we can control this
     *        entirely through interrupt. Look at using a hardware timer
     *        for this.
     *  **/
    status_t err = STATUS_OK;

    err = gpio_set_level(dev->step, 1);

    if (timer_start(TIMER_GRP_FROM_INDEX(dev->index), TIMER_ID_FROM_INDEX(dev->index)) != STATUS_OK)
    {
        log_error(DEV_TAG, "Error starting pulse timer");
        err = STATUS_ERR_INVALID_STATE;
    }
    /** even if error, we want the step count to decrease **/
    if (dev->steps_queued > 0) {
        dev->steps_queued--;
    }
    return err;
}

status_t a4988_reset(A4988_DEV dev)
{
    status_t err = STATUS_OK;

    err = gpio_set_level(dev->rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    err = gpio_set_level(dev->rst, 1);
    return err;
}

status_t a4988_clear_step_queue(A4988_DEV dev)
{
    dev->steps_queued = 0;
    return STATUS_OK;
}

status_t a4988_set_queued_steps(A4988_DEV dev, uint16_t *steps)
{
    dev->steps_queued = *steps;
    return STATUS_OK;
}

status_t a4988_get_queued_steps(A4988_DEV dev, uint16_t *steps)
{
    *steps = dev->steps_queued;
    return STATUS_OK;
}

status_t a4988_get_stepsize(A4988_DEV dev, uint8_t *sz)
{
    *sz = dev->step_size;
    return STATUS_OK;
}

status_t a4988_set_stepsize(A4988_DEV dev, uint8_t *sz)
{
    uint8_t val = *sz;

    if (!(dev->_ms)) {
        log_error(DEV_TAG, "Driver does not control step size!");
        return STATUS_ERR_NOT_SUPPORTED;
    }

    if (val != FULL_STEP_T && val != HALF_STEP_T && val != QURT_STEP_T && val != EGTH_STEP_T
        && val != SXTN_STEP_T)
    {
        return STATUS_ERR_INVALID_ARG;
    }

    /** write the pins **/
    gpio_set_level(dev->ms1, (val & 0b001));
    gpio_set_level(dev->ms2, (val & 0b010));
    gpio_set_level(dev->ms3, (val & 0b100));

    return STATUS_OK;
}

status_t a4988_get_sleepstate(A4988_DEV dev, bool *slp)
{
    *slp = dev->is_sleeping;
    return STATUS_OK;
}

status_t a4988_set_sleepstate(A4988_DEV dev, bool *slp)
{
    bool sleep = *slp;
    if (!(dev->_sleep)) {
        return STATUS_ERR_NOT_SUPPORTED;
    } else if (sleep && dev->is_sleeping) {
        return STATUS_OK;
    } else if (!sleep && !(dev->is_sleeping)) {
        return STATUS_OK;
    } else {
        gpio_set_level(dev->sleep, (uint32_t)sleep);
    }
    return STATUS_OK;
}

status_t a4988_get_enable(A4988_DEV dev, bool *en)
{
    *en = dev->is_enabled;
    return STATUS_OK;
}

status_t a4988_set_enable(A4988_DEV dev, bool *en)
{
    if (!(dev->_en)) {
        return STATUS_ERR_NOT_SUPPORTED;
    } else {
        uint32_t e = (uint32_t)*en;
        gpio_set_level(dev->enable, e);
    }
    return STATUS_OK;
}

status_t a4988_get_step_delay(A4988_DEV dev, uint16_t *t)
{
    *t = dev->step_wait;
    return STATUS_OK;
}

status_t a4988_set_step_delay(A4988_DEV dev, uint16_t *t)
{
    dev->step_wait = *t;

    a4988_msg_t msg = {.cmd = A4988_CMD_UPDATE_PERIOD, .dev = dev};

    xQueueSend(a4988_cmd_queue, &msg, A4988_CONFIG_SHORT_WAIT);
    return STATUS_OK;
}

status_t a4988_get_direction(A4988_DEV dev, bool *dir)
{
    *dir = dev->direction;
    return STATUS_OK;
}

status_t a4988_set_direction(A4988_DEV dev, bool *dir)
{
    /** TODO: don't check for existing direction here in case
     *  driver ends up in unknown state or something weird
     *  **/
    uint8_t val = *dir;
    if ((val && !(dev->direction)) || (!(val) && dev->direction)) {
        gpio_set_level(dev->dir, (uint32_t)val);
        dev->direction = val;
    }
    return STATUS_OK;
}
