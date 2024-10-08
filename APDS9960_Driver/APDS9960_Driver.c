/***************************************
 * \file     ADPS9960_Driver.c
 * \brief    Driver for the RGB, Gesture & proximity detector
 *           Going to cover most of the device's features
 *           TBH not going to do the offsets unless needed.
 *           (Aint nobody got time for that)
 *           https://ww1.microchip.com/downloads/en/DeviceDoc/21419D.pdf
 *
 * \date     March 2021
 * \author   RJAM
 ****************************************/

/********* Includes *******************/

#include "APDS9960_Driver.h"
#include "Utilities.h"
#include "port/driver/gpio.h"
#include "port/error_type.h"
#include "port/malloc.h"
#include "port/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "GenericCommsDriver.h"

#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#ifdef CONFIG_USE_PERIPH_MANAGER
const parameter_t apds_parameter_map[apds_param_len] = {
    {"Power status",
     1,
     &apds_get_pwr_on_status,
     &apds_set_pwr_on_status,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"Proximity status",
     2,
     &apds_get_proximity_status,
     &apds_set_proximity_status,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"RGB status",
     3,
     &apds_get_als_status,
     &apds_set_als_status,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"Gesture status",
     4,
     &apds_get_gesture_status,
     &apds_set_gesture_status,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"RGB ADC time",
     5,
     &apds_get_adc_time,
     &apds_set_adc_time,
     NULL,
     DATATYPE_UINT8,
     255,
     (GET_FLAG | SET_FLAG)},
    {"Wait time",
     6,
     &apds_get_wait_time,
     &apds_set_wait_time,
     NULL,
     DATATYPE_UINT8,
     255,
     (GET_FLAG | SET_FLAG)},
    {"RGB Low thresh",
     7,
     &apds_get_alsintr_low_thr,
     &apds_set_alsintr_low_thr,
     NULL,
     DATATYPE_UINT16,
     0xffff,
     (GET_FLAG | SET_FLAG)},
    {"RGB High thresh",
     8,
     &apds_get_alsintr_hi_thr,
     &apds_set_alsintr_hi_thr,
     NULL,
     DATATYPE_UINT16,
     0xffff,
     (GET_FLAG | SET_FLAG)},
    {"Prox Low thresh",
     9,
     &apds_get_prxintr_low_thr,
     &apds_set_prxintr_low_thr,
     NULL,
     DATATYPE_UINT8,
     0xff,
     (GET_FLAG | SET_FLAG)},
    {"Prox High thresh",
     10,
     &apds_get_prxintr_high_thr,
     &apds_set_prxintr_high_thr,
     NULL,
     DATATYPE_UINT8,
     0xff,
     (GET_FLAG | SET_FLAG)},
    {"Long Wait",
     11,
     &apds_get_longwait_en,
     &apds_set_longwait_en,
     NULL,
     DATATYPE_BOOL,
     1,
     (GET_FLAG | SET_FLAG)},
    {"Prox Intr perist",
     12,
     &apds_get_prx_intr_persistence,
     &apds_set_prx_intr_persistence,
     NULL,
     DATATYPE_UINT8,
     16,
     (GET_FLAG | SET_FLAG)},
    {"RGB Intr perist",
     13,
     &apds_get_als_intr_persistence,
     &apds_set_als_intr_persistence,
     NULL,
     DATATYPE_UINT8,
     16,
     (GET_FLAG | SET_FLAG)},
    {"Prox Pulse time",
     14,
     &apds_get_prx_ledpulse_t,
     &apds_set_prx_ledpulse_t,
     NULL,
     DATATYPE_UINT8,
     3,
     (GET_FLAG | SET_FLAG)},
    {"Prox Pulse count",
     15,
     &apds_get_prx_pulses,
     &apds_set_prx_pulses,
     NULL,
     DATATYPE_UINT8,
     64,
     (GET_FLAG | SET_FLAG)},
    {"Led Drive str",
     16,
     &apds_get_led_drive,
     &apds_set_led_drive,
     NULL,
     DATATYPE_UINT8,
     3,
     (GET_FLAG | SET_FLAG)},
    {"Prox Gain",
     17,
     &apds_get_prx_gain,
     &apds_set_prx_gain,
     NULL,
     DATATYPE_UINT8,
     4,
     (GET_FLAG | SET_FLAG)},
    {"RGB Gain",
     18,
     &apds_get_als_gain,
     &apds_set_als_gain,
     NULL,
     DATATYPE_UINT8,
     4,
     (GET_FLAG | SET_FLAG)},
};

const peripheral_t apds_periph_template = {
    .handle = NULL,
    .param_len = apds_param_len,
    .params = apds_parameter_map,
    .peripheral_name = "APDS9960",
    .peripheral_id = 0};

#endif

const char *APDS_TAG = "APDS Driver";

/****** Function Prototypes ***********/

static IRAM_ATTR void apds_intr_handler(void *args);

static status_t regSetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask);

static status_t regUnsetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask);

static uint8_t get_interrupt_source_and_clear(APDS_DEV dev);

static bool is_als_valid(APDS_DEV dev);

static bool is_prx_valid(APDS_DEV dev);

/************ ISR *********************/

IRAM_ATTR void apds_intr_handler(void *args)
{
    APDS_DEV dev = (adps_handle_t *)args;

    BaseType_t higherPrio = pdFALSE;
    /* send the device handle as the notify value
       we don't adjust this value if the notify is still pending,
       so it is possible to miss interrupts from multiple devices
       if the task has not yet serviced a notify from another device.
       ISRs could be missed on a single device but they are
       all checked when the task is run.
    */
    if (xTaskNotifyFromISR(
            apds_driver_task_h,
            (uint32_t)dev,
            eSetValueWithoutOverwrite,
            &higherPrio)
        != pdPASS)
    {
        dev->missed_isr++;
    }
    portYIELD_FROM_ISR();
}

/****** Private Data ******************/

/****** Private Functions *************/



/** Sets the BITS in mask **/
static status_t regSetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask)
{
    status_t err = STATUS_OK;
    uint8_t regval = 0;

    err = gcd_i2c_read_address(dev->bus, dev->addr, regaddr, 1, &regval);

    if (!err) {
        // check if mask is already set
        if ((regval & mask) != mask) {
            regval |= mask;
            err = gcd_i2c_write_address(dev->bus, dev->addr, regaddr, 1, &regval);
        }
    }

    return err;
}

/** unset the bits in the mask **/
static status_t regUnsetMask(APDS_DEV dev, uint8_t regaddr, uint8_t mask)
{
    status_t err = STATUS_OK;
    uint8_t regval = 0;

    err = gcd_i2c_read_address(dev->bus, dev->addr, regaddr, 1, &regval);

    if (!err) {
        regval &= ~(mask);
        err = gcd_i2c_write_address(dev->bus, dev->addr, regaddr, 1, &regval);
    }

    return err;
}

static status_t readModWrite(APDS_DEV dev, uint8_t regaddr, uint8_t clear_mask, uint8_t set_mask)
{
    status_t err = STATUS_OK;
    uint8_t regval;

    err = gcd_i2c_read_address(dev->bus, dev->addr, regaddr, 1, &regval);

    if (!err) {
        replaceBits(&regval, clear_mask, set_mask);

        err = gcd_i2c_write_address(dev->bus, dev->addr, regaddr, 1, &regval);
    }

    return err;
}

/** read the status register, clear any outstanding interrupts **/
static uint8_t get_interrupt_source_and_clear(APDS_DEV dev)
{
    status_t err = STATUS_OK;
    uint8_t regval = 0;
    uint8_t retval = 0;
    uint8_t clear = 1;
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &regval);
    if (!err) {
        if (regval & APDS_REGBIT_PRX_INT) {
            retval |= APDS_REGBIT_PRX_INT;
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_ISR_CLR, 1, &clear);
        }
        if (regval & APDS_REGBIT_ALS_INT) {
            retval |= APDS_REGBIT_ALS_INT;
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ALS_ISR_CLR, 1, &clear);
        }
        // if(regval & APDS_REGBIT_GST_INT) {
        //     retval |= APDS_REGBIT_GST_INT;
        //     gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_ISR_CLR,
        //     1, &clear);
        // }
    }

    return retval;
}

static apds_direction_t apds_gst_max_at_index(APDS_DEV dev, uint8_t index)
{
    if (index > 31) {
        return APDS_DIRECTION_INVALID;
    }

    apds_direction_t dir = APDS_DIRECTION_UP;
    uint8_t max = 0;
    uint8_t max_index = 0;
    bool max_tie = false;

    uint8_t data[4] = {
        dev->data.gst_fifo_u_data[index],
        dev->data.gst_fifo_d_data[index],
        dev->data.gst_fifo_l_data[index],
        dev->data.gst_fifo_r_data[index],
    };

    for (uint8_t i = 0; i < 4; i++) {
        if (max < data[i]) {
            max = data[i];
            max_index = i;
            if (max_tie) {
                /** there was maybe a previous tie **/
                max_tie = false;
            }
        } else if (max == data[i]) {
            max_tie = true;
        }
    }

    if (max_tie) {
        dir = APDS_DIRECTION_UNK;
    } else {
        /** this works because of order of enum & u/d/l/r data align **/
        dir = max_index;
    }

    return dir;
}

/** super simple - assume half entries are first swipe and half entries
 *                  are swipe exit. How to deal with small #'s of values?
 *                  why even process swipe with small number?
 *                  Assume minumum 4 measurements
 **/
status_t apds_detect_swipe_dir(APDS_DEV dev)
{
    status_t err = STATUS_OK;
    uint8_t highest = 0;
    uint8_t lowest = UINT8_MAX;
    apds_direction_t direction_bias[32] = {0};
    uint8_t num_measures = dev->gst_settings.fifo_pkts_read;
    uint8_t offset = 0;
    uint8_t weights[4] = {0};
    apds_direction_t entry_direction = 0;
    apds_direction_t exit_direction = 0;
    apds_swipe_t swipe = APDS_SWIPE_UNCERTAIN;

    if (num_measures < 1) {
        log_error(APDS_TAG, "Error: No fifo data to process");
    }

    else if (num_measures < 4)
    {
        log_info(APDS_TAG, "Cannot get reliable swipe from less than 4 samples!");
    }

    else
    {
        for (uint8_t i = 0; i < num_measures; i++) {
            /** get the highest and lowest per set **/
            direction_bias[i] = apds_gst_max_at_index(dev, i);
        }

        log_info(APDS_TAG, "Processing %u measurements to determine swipe direction", num_measures);

        for (uint8_t i = 0; i < 2; i++) {
            if (i == 0) {
                /** examine the first half **/
                for (uint8_t j = 0; j < num_measures / 2; j++) {
                    switch (direction_bias[j]) {
                        case APDS_DIRECTION_UP: weights[APDS_DIRECTION_UP]++; break;
                        case APDS_DIRECTION_DOWN: weights[APDS_DIRECTION_DOWN]++; break;
                        case APDS_DIRECTION_LEFT: weights[APDS_DIRECTION_LEFT]++; break;
                        case APDS_DIRECTION_RIGHT: weights[APDS_DIRECTION_RIGHT]++; break;
                        default: break;
                    }
                }
                /** get the highest weighted direction & clear the array **/
                entry_direction = index_of_largest(weights, 4);
                log_info(
                    "Sums: %u %u %u %u: Entry dir: %u\n ",
                    weights[0],
                    weights[1],
                    weights[2],
                    weights[3],
                    entry_direction);
                memset(weights, 0, sizeof(weights));
            }
            if (i == 1) {
                /** examine the last half **/
                for (uint8_t j = num_measures / 2; j < num_measures; j++) {
                    switch (direction_bias[j]) {
                        case APDS_DIRECTION_UP: weights[APDS_DIRECTION_UP]++; break;
                        case APDS_DIRECTION_DOWN: weights[APDS_DIRECTION_DOWN]++; break;
                        case APDS_DIRECTION_LEFT: weights[APDS_DIRECTION_LEFT]++; break;
                        case APDS_DIRECTION_RIGHT: weights[APDS_DIRECTION_RIGHT]++; break;
                        default: break;
                    }
                }
                exit_direction = index_of_largest(weights, 4);
                log_info(
                    "Sums: %u %u %u %u: Exit dir: %u\n ",
                    weights[0],
                    weights[1],
                    weights[2],
                    weights[3],
                    exit_direction);
            }
        }

        /** Get swipe direction from entry/exit bias - 'diagonal' swipes are not
         * handled **/
        if (entry_direction == APDS_DIRECTION_UP && exit_direction == APDS_DIRECTION_DOWN) {
            swipe = APDS_SWIPE_UP_TO_DOWN;
        } else if (entry_direction == APDS_DIRECTION_DOWN && exit_direction == APDS_DIRECTION_UP) {
            swipe = APDS_SWIPE_DOWN_TO_UP;
        } else if (entry_direction == APDS_DIRECTION_LEFT && exit_direction == APDS_DIRECTION_RIGHT)
        {
            swipe = APDS_SWIPE_LEFT_TO_RIGHT;
        } else if (entry_direction == APDS_DIRECTION_RIGHT && exit_direction == APDS_DIRECTION_LEFT)
        {
            swipe = APDS_SWIPE_RIGHT_TO_LEFT;
        } else {
            swipe = APDS_SWIPE_UNCERTAIN;
        }

        log_info(
            APDS_TAG,
            "Done Processing: Entry bias %u Exit bias %u. Packets processed: "
            "%u Calculated swipe direction: %u",
            entry_direction,
            exit_direction,
            num_measures,
            swipe);
    }

    return swipe;
}

static bool is_als_valid(APDS_DEV dev)
{
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &val);
    return ((val & APDS_REGBIT_ALS_VALID) ? true : false);
}

static bool is_prx_valid(APDS_DEV dev)
{
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_STATUS, 1, &val);
    return ((val & APDS_REGBIT_PRX_VALID) ? true : false);
}

static status_t apds_initialise_device(APDS_DEV dev)
{
    uint8_t pwr_on = 1;
    uint8_t id = 0;

    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &pwr_on);

    if (!err) {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ID, 1, &id);
        if (!err) {
            log_info(APDS_TAG, "Got ID: 0x%02x", id);
        }
    }

    /** set some default values - looks like rest are 0 default? **/

    dev->als_settings.adc_intg_time = 0xff;
    dev->gen_settings.wait_time = 0xff;
    return err;
}

static status_t apds_config_intr_pin(APDS_DEV dev)
{
    gpio_config_t conf = {0};

    conf.pin_bit_mask = (1 << dev->intr_pin);
    conf.intr_type = GPIO_INTR_NEGEDGE;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = 0;

    status_t err = gpio_config(&conf);
    if (err) {
        log_error(APDS_TAG, "Error configuring GPIO pin! [%u]", err);
    } else {
        err = gpio_isr_handler_add(dev->intr_pin, (gpio_isr_t)apds_intr_handler, dev);
    }

    return err;
}

static status_t apds_gst_clr_fifo(APDS_DEV dev)
{
    status_t err = regSetMask(dev, APDS_REGADDR_GST_CONFIG_3, APDS_REGBIT_GST_FIFO_CLR);
    return err;
}

static status_t testmode(APDS_DEV dev)
{
    return STATUS_OK;
}

static uint8_t get_gst_fifo_pkts(APDS_DEV dev)
{
    uint8_t val = 0;
    gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_LVL, 1, &val);
    return val;
}

static void apds_driver_task(void *args)
{
    APDS_DEV dev;
    uint32_t notify = 0;
    uint8_t byte = 0;
    status_t err = STATUS_OK;

    while (1) {
        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(1000)) == pdPASS) {
            /* retrieve the driver handle from the notification value */
            dev = (adps_handle_t *)notify;

            log_info(APDS_TAG, "Received interrupt");

            /** get the isr source **/
            uint8_t interrupt_source = get_interrupt_source_and_clear(dev);
            log_info(APDS_TAG, "Got interrupt value 0x%02x", interrupt_source);

            /** TODO: Take actions!
             *  if data valid, set the bit in the struct so user can poll easily
             *  if saturation occured, mark in struct too
             *  **/

            if (interrupt_source & APDS_REGBIT_CP_SAT) {
                log_info(APDS_TAG, "Clear LED Channel ADC was Saturated!");
            }
            if (interrupt_source & APDS_REGBIT_PRX_GST_SAT) {
                log_info(APDS_TAG, "Clear Gesture ADC was Saturated!");
            }
            if (interrupt_source & APDS_REGBIT_PRX_INT) {
                log_info(APDS_TAG, "Proximity Interrupt threshold interrupt!");
            }
            if (interrupt_source & APDS_REGBIT_ALS_INT) {
                log_info(APDS_TAG, "ALS Interrupt threshold interrupt!");
            }
            if (interrupt_source & APDS_REGBIT_GST_INT) {
                log_info(APDS_TAG, "Gesture Interrupt threshold interrupt!");
            }
            if (interrupt_source & APDS_REGBIT_PRX_VALID) {
                log_info(APDS_TAG, "Proximity is valid!");
            }
            if (interrupt_source & APDS_REGBIT_ALS_VALID) {
                log_info(APDS_TAG, "ALS is valid!");
            }
            if (dev->gst_settings.gmode > 0) {
                apds_swipe_t swipe_dir;
                apds_get_fifo_valid(dev, (bool *)&byte);
                if (byte == 0) {
                    log_info(APDS_TAG, "FIFO Data is not valid! Reading anyway...");
                }

                err = apds_read_fifo_full(dev);
                if (!err) {
                    log_info(APDS_TAG, "Fifo read complete");
                    swipe_dir = apds_detect_swipe_dir(dev);
                    dev->gst_settings.last_swipe = swipe_dir;
                }
                apds_gst_clr_fifo(dev);
            }
        }
    }
    /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
APDS_DEV apds_init(apds_init_t *init)
#else
APDS_DEV apds_init(APDS_DEV dev, apds_init_t *init)
#endif
{
    status_t err = STATUS_OK;

    if (!gcd_i2c_check_bus(init->i2c_bus)) {
        err = STATUS_ERR_INVALID_ARG;
        log_error(APDS_TAG, "Error - invalid i2c bus");
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    APDS_DEV dev;

    if (!err) {
        dev = heap_caps_calloc(1, sizeof(adps_handle_t), MALLOC_CAP_DEFAULT);
        if (dev == NULL) {
            log_error(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
            err = STATUS_ERR_NO_MEM;
        }
    }
#else
    memset(dev, 0, sizeof(adps_handle_t));
#endif

    if (!err) {
        dev->addr = APDS_I2C_ADDRESS;
        dev->bus = init->i2c_bus;
    }

    if (init->intr_pin != 0) {
        dev->intr_pin = init->intr_pin;
        err = apds_config_intr_pin(dev);
        if (!err) {
            dev->intr_en = true;
        }
    }

    if (!err && apds_initialise_device(dev)) {
        log_error(APDS_TAG, "Error: Error assigning heap mem [%u]", err);
        err = STATUS_ERR_TIMEOUT;
    }

    if (!err
        && xTaskCreate(
               apds_driver_task,
               "apds_driver_task",
               CONFIG_APDS_TASK_STACK_SZ,
               NULL,
               3,
               &apds_driver_task_h)
               != pdTRUE)
    {
        log_error(APDS_TAG, "Error creating driver task");
        err = STATUS_ERR_NO_MEM;
    }

#if APDS_DEBUG_MODE
    if (!err) {
        log_info(APDS_TAG, "Entering test mode!");
        testmode(dev);
    }
#endif

    if (err && dev != NULL) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        heap_caps_free(dev);
#endif
    }

    return dev;
}

/*************** ENABLE SETTINGS **************************/

status_t apds_get_pwr_on_status(APDS_DEV dev, uint8_t *on)
{
    *on = dev->gen_settings.pwr_on;
    return STATUS_OK;
}

status_t apds_set_pwr_on_status(APDS_DEV dev, uint8_t *on)
{
    status_t err = STATUS_OK;
    uint8_t val = *on;

    if (val) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PWR_ON);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PWR_ON);
    }

    if (!err) {
        dev->gen_settings.pwr_on = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_proximity_status(APDS_DEV dev, uint8_t *on)
{
    *on = dev->prx_settings.prx_en;
    return STATUS_OK;
}

status_t apds_set_proximity_status(APDS_DEV dev, uint8_t *on)
{
    status_t err = STATUS_OK;
    uint8_t val = *on;
    if (val) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_EN);
    }

    if (!err) {
        dev->prx_settings.prx_en = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_als_status(APDS_DEV dev, uint8_t *on)
{
    *on = dev->als_settings.als_en;
    return STATUS_OK;
}

status_t apds_set_als_status(APDS_DEV dev, uint8_t *on)
{
    status_t err = STATUS_OK;
    uint8_t val = *on;
    if (val) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_EN);
    }

    if (!err) {
        dev->als_settings.als_en = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_gesture_status(APDS_DEV dev, uint8_t *on)
{
    *on = dev->gst_settings.gst_en;
    return STATUS_OK;
}

status_t apds_set_gesture_status(APDS_DEV dev, uint8_t *on)
{
    uint8_t state = *on;
    uint8_t regval = 0;
    status_t err = STATUS_OK;

    if (state && dev->gst_settings.gst_en) {
        log_info(APDS_TAG, "Gesture engine alreay enabled");
        return STATUS_OK;
    } else if (!state && !(dev->gst_settings.gst_en)) {
        log_info(APDS_TAG, "Gesture engine alreay disabled");
        return STATUS_OK;
    } else {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
        if (state) {
            regval |= APDS_REGBIT_GST_EN;
        } else {
            regval &= ~APDS_REGBIT_GST_EN;
        }
        if (!err) {
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
            if (!err) {
                dev->gst_settings.gst_en = (state ? 1 : 0);
            }
        }
    }

    return STATUS_OK;
}

/**************** GENERAL SETTINGS ***************/

status_t apds_set_wait_enable(APDS_DEV dev, uint8_t *val)
{
    uint8_t state = *val;
    uint8_t regval = 0;
    status_t err = STATUS_OK;

    if (state && dev->gen_settings.wait_en) {
        log_info(APDS_TAG, "Gesture engine alreay enabled");
        return STATUS_OK;
    } else if (!state && !(dev->gen_settings.wait_en)) {
        log_info(APDS_TAG, "Gesture engine alreay disabled");
        return STATUS_OK;
    } else {
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
        if (state) {
            regval |= APDS_REGBIT_WAIT_EN;
        } else {
            regval &= ~APDS_REGBIT_WAIT_EN;
        }
        if (!err) {
            gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ENABLE, 1, &regval);
            if (!err) {
                dev->gen_settings.wait_en = (state ? 1 : 0);
            }
        }
    }

    return STATUS_OK;
}

status_t apds_get_wait_enable(APDS_DEV dev, uint8_t *val)
{
    *val = dev->gen_settings.wait_en;
    return STATUS_OK;
}

status_t apds_get_wait_time(APDS_DEV dev, uint8_t *wait)
{
    *wait = dev->gen_settings.wait_time;
    return STATUS_OK;
}

status_t apds_set_wait_time(APDS_DEV dev, uint8_t *wait)
{
    uint8_t byte = *wait;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_WAITTIME, 1, &byte);
    if (!err) {
        dev->gen_settings.wait_time = byte;
    }
    return err;
}

status_t apds_get_longwait_en(APDS_DEV dev, uint8_t *en)
{
    *en = dev->gen_settings.wait_long_en;
    return STATUS_OK;
}

status_t apds_set_longwait_en(APDS_DEV dev, uint8_t *en)
{
    uint8_t val = *en;
    status_t err = STATUS_OK;
    if (val) {
        err = regSetMask(dev, APDS_REGADDR_CONFIG_1, APDS_REGBITS_WLON_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_CONFIG_1, APDS_CONFIG_1_MASK);
    }

    if (!err) {
        dev->gen_settings.wait_long_en = val;
    }

    return STATUS_OK;
}

status_t apds_set_led_drive(APDS_DEV dev, apds_led_drive_t *drive)
{
    uint8_t d = *drive;
    status_t err = STATUS_OK;
    if (d >= APDS_LEDDRIVE_MAX) {
        err = STATUS_ERR_INVALID_ARG;
        log_error(APDS_TAG, "Invalid args");
    } else {
        d = (d << 6);
        err = readModWrite(dev, APDS_REGADDR_GAIN_CTRL, (3 << 6), d);
    }
    if (!err) {
        dev->gst_settings.gst_led_drive_str = d;
    }
    return err;
}

status_t apds_get_led_drive(APDS_DEV dev, apds_led_drive_t *drive)
{
    *drive = dev->gst_settings.gst_led_drive_str;
    return STATUS_OK;
}

/***************  ALS Settings ***********************/

status_t apds_get_adc_time(APDS_DEV dev, uint8_t *adct)
{
    *adct = dev->als_settings.adc_intg_time;
    return STATUS_OK;
}

status_t apds_set_adc_time(APDS_DEV dev, uint8_t *adct)
{
    uint8_t byte = *adct;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_ADCTIME, 1, &byte);
    if (!err) {
        dev->als_settings.adc_intg_time = byte;
    }
    return err;
}

status_t apds_get_alsintr_low_thr(APDS_DEV dev, uint16_t *thr)
{
    *thr = dev->als_settings.als_thresh_l;
    return STATUS_OK;
}

status_t apds_set_alsintr_low_thr(APDS_DEV dev, uint16_t *thr)
{
    uint16_t val = *thr;
    uint8_t bytes[2] = {
        (uint8_t)val,
        ((uint8_t)val >> 8),
    };
    status_t err = gcd_i2c_write_address(
        dev->bus,
        dev->addr,
        APDS_REGADDR_ALS_THR_LOW_LSB,
        2,
        bytes);
    if (!err) {
        dev->als_settings.als_thresh_l = val;
    }
    return err;
}

status_t apds_get_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr)
{
    *thr = dev->als_settings.als_thresh_h;
    return STATUS_OK;
}

status_t apds_set_alsintr_hi_thr(APDS_DEV dev, uint16_t *thr)
{
    uint16_t val = *thr;

    status_t err = gcd_i2c_write_address(
        dev->bus,
        dev->addr,
        APDS_REGADDR_ALS_THR_HIGH_LSB,
        2,
        &val);
    if (!err) {
        dev->als_settings.als_thresh_h = val;
    }
    return err;
}

status_t apds_get_als_intr_persistence(APDS_DEV dev, uint8_t *cnt)
{
    *cnt = dev->als_settings.als_persist;
    return STATUS_OK;
}

status_t apds_set_als_intr_persistence(APDS_DEV dev, uint8_t *cnt)
{
    status_t err = STATUS_OK;
    uint8_t val = *cnt;
    uint8_t regval = 0;
    if (val > 15) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_ISR_PERSIST_FLTR, APDS_ALS_INT_PERSIST_MASK, val);
    }

    if (!err) {
        dev->als_settings.als_persist = val;
    }

    return err;
}

status_t apds_get_als_gain(APDS_DEV dev, als_gain_t *g)
{
    *g = dev->als_settings.als_gain;
    return STATUS_OK;
}

status_t apds_set_als_gain(APDS_DEV dev, als_gain_t *g)
{
    uint8_t val = *g;
    status_t err = STATUS_OK;
    if (val > APDS_GST_GAIN_MAX) {
        err = STATUS_ERR_INVALID_ARG;
        log_error(APDS_TAG, "Invalid args");
    } else {
        err = readModWrite(dev, APDS_REGADDR_GAIN_CTRL, 3, val);
    }

    if (!err) {
        dev->als_settings.als_gain = val;
    }

    return err;
}

/***************** Proximity Settings **********************/

status_t apds_get_prxintr_low_thr(APDS_DEV dev, uint8_t *thr)
{
    *thr = dev->prx_settings.prox_thresh_l;
    return STATUS_OK;
}

status_t apds_set_prxintr_low_thr(APDS_DEV dev, uint8_t *thr)
{
    uint8_t val = *thr;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_THR_LOW, 1, &val);
    if (!err) {
        dev->prx_settings.prox_thresh_l = val;
    }
    return err;
}

status_t apds_get_prxintr_high_thr(APDS_DEV dev, uint8_t *thr)
{
    *thr = dev->prx_settings.prox_thresh_h;
    return STATUS_OK;
}

status_t apds_set_prxintr_high_thr(APDS_DEV dev, uint8_t *thr)
{
    uint8_t val = *thr;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_THR_HIGH, 1, &val);
    if (!err) {
        dev->prx_settings.prox_thresh_h = val;
    }
    return err;
}

status_t apds_get_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt)
{
    *cnt = dev->prx_settings.prox_perist_cycles;
    return STATUS_OK;
}

status_t apds_set_prx_intr_persistence(APDS_DEV dev, uint8_t *cnt)
{
    status_t err = STATUS_OK;
    uint8_t val = *cnt;
    if (val > 15) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_ISR_PERSIST_FLTR, 0xf0, (val << 4));
    }

    if (!err) {
        dev->prx_settings.prox_perist_cycles = val;
    }

    return err;
}

status_t apds_get_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t)
{
    *t = dev->prx_settings.ledtime;
    return STATUS_OK;
}

status_t apds_set_prx_ledpulse_t(APDS_DEV dev, prx_ledtime_t *t)
{
    uint8_t val = *t;
    status_t err = STATUS_OK;

    if (val >= APDS_PRX_LEDTIME_MAX) {
        log_error(APDS_TAG, "Error: Invalid value [%u]", val);
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_PRX_PULSE_LEN, 0xC0, (val << 6));
    }

    if (!err) {
        dev->prx_settings.ledtime = val;
    }

    return err;
}

status_t apds_get_prx_pulses(APDS_DEV dev, uint8_t *cnt)
{
    *cnt = dev->prx_settings.led_pulse_n + 1;
    return STATUS_OK;
}

status_t apds_set_prx_pulses(APDS_DEV dev, uint8_t *cnt)
{
    uint8_t val = *cnt;
    status_t err = STATUS_OK;

    if (val > 63) {
        err = STATUS_ERR_INVALID_ARG;
        log_error(APDS_TAG, "Invalid value");
    } else {
        uint8_t regval = val | (dev->prx_settings.ledtime << 6);
        err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_PRX_PULSE_LEN, 1, &regval);
    }
    if (!err) {
        dev->prx_settings.led_pulse_n = val;
    }

    return err;
}

status_t apds_get_prx_gain(APDS_DEV dev, gst_gain_t *g)
{
    *g = dev->prx_settings.prox_gain;
    return STATUS_OK;
}

status_t apds_set_prx_gain(APDS_DEV dev, gst_gain_t *g)
{
    uint8_t val = *g;
    status_t err = STATUS_OK;
    if (val > APDS_PRX_GAIN_MAX) {
        err = STATUS_ERR_INVALID_ARG;
        log_error(APDS_TAG, "Invalid args");
    } else {
        err = readModWrite(
            dev,
            APDS_REGADDR_GAIN_CTRL,
            (3 << APDS_REG_OFFSET_PGAIN),
            (val << APDS_REG_OFFSET_PGAIN));
    }

    if (!err) {
        dev->prx_settings.prox_gain = val;
    }
    return err;
}

/******************** Gesture Settings ***********************/

status_t apds_get_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d)
{
    *d = dev->gst_settings.gst_thresh_entr;
    return STATUS_OK;
}

status_t apds_set_gst_proximity_ent_thr(APDS_DEV dev, uint8_t *d)
{
    uint8_t val = *d;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_ENTR_THR, 1, &val);
    if (!err) {
        dev->gst_settings.gst_thresh_entr = val;
    }

    return err;
}

status_t apds_get_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d)
{
    *d = dev->gst_settings.gst_thresh_exit;
    return STATUS_OK;
}

status_t apds_set_gst_proximity_ext_thr(APDS_DEV dev, uint8_t *d)
{
    uint8_t val = *d;
    status_t err = gcd_i2c_write_address(dev->bus, dev->addr, APDS_REGADDR_GST_EXT_THR, 1, &val);
    if (!err) {
        dev->gst_settings.gst_thresh_exit = val;
    }

    return err;
}

status_t apds_set_gst_direction_mode(APDS_DEV dev, uint8_t *mode)
{
    uint8_t regval = *mode;
    if (regval > APDS_DIR_LEFTRIGHT_ONLY) {
        return STATUS_ERR_INVALID_ARG;
    }
    status_t err = gcd_i2c_write_address(
        dev->bus,
        dev->addr,
        APDS_REGADDR_GST_CONFIG_3,
        1,
        &regval);
    if (!err) {
        dev->gst_settings.gst_dir_select = regval;
    }
    return err;
}

status_t apds_get_gst_direction_mode(APDS_DEV dev, uint8_t *mode)
{
    *mode = dev->gst_settings.gst_dir_select;
    return STATUS_OK;
}

status_t apds_get_gst_pulse_len(APDS_DEV dev, uint8_t *cnt)
{
    status_t status = STATUS_OK;
    *cnt = dev->gst_settings.gst_pulse_len;
    return status;
}

status_t apds_set_gst_pulse_len(APDS_DEV dev, uint8_t *cnt)
{
    status_t err = STATUS_OK;
    uint8_t val = *cnt;
    uint8_t regval = 0;
    if (val > 3) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_GST_PULSE_LEN, 0b11000000, (val << 6));
    }

    if (!err) {
        dev->gst_settings.gst_pulse_len = val;
    }

    return err;
}

status_t apds_get_gst_pulse_cnt(APDS_DEV dev, uint8_t *cnt)
{
    status_t status = STATUS_OK;
    *cnt = dev->gst_settings.gst_pulse_len;
    return status;
}

status_t apds_set_gst_pulse_cnt(APDS_DEV dev, uint8_t *cnt)
{
    status_t err = STATUS_OK;
    uint8_t val = *cnt;
    uint8_t regval = 0;
    if (val > 64) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_GST_PULSE_LEN, 0b11111, val);
    }

    if (!err) {
        dev->gst_settings.gst_pulse_cnt = val;
    }

    return err;
}

status_t apds_set_gst_gmode(APDS_DEV dev, uint8_t *val)
{
    uint8_t en = *val;
    status_t err = STATUS_OK;

    if (en) {
        err = regSetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_MODE);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_MODE);
    }

    if (!err) {
        dev->gst_settings.gmode = (en > 0) ? true : false;
    }

    return err;
}

status_t apds_get_gst_gmode(APDS_DEV dev, uint8_t *val)
{
    *val = dev->gst_settings.gmode;
    return STATUS_OK;
}

status_t apds_set_gst_gain(APDS_DEV dev, uint8_t *gain)
{
    status_t err = STATUS_OK;
    uint8_t val = *gain;

    if (val >= APDS_GST_GAIN_MAX) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(
            dev,
            APDS_REGADDR_GST_CONFIG_2,
            0b11100000,
            (val << APDS_REGOFFSET_GST_GAIN));
    }

    if (!err) {
        dev->gst_settings.gst_gain_ctrl = val;
    }

    return err;
}

status_t apds_get_gst_wait(APDS_DEV dev, uint8_t *wait)
{
    *wait = dev->gst_settings.gst_wait_time;
    return STATUS_OK;
}

status_t apds_set_gst_wait(APDS_DEV dev, uint8_t *wait)
{
    status_t err = STATUS_OK;
    uint8_t val = *wait;

    if (val >= APDS_GST_WAIT_T_MAX) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_GST_CONFIG_2, 0b111, val);
    }

    if (!err) {
        dev->gst_settings.gst_wait_time = val;
    }

    return err;
}

status_t apds_get_gst_gain(APDS_DEV dev, uint8_t *gain)
{
    *gain = dev->gst_settings.gst_gain_ctrl;
    return STATUS_OK;
}

status_t apds_set_gst_led_drive(APDS_DEV dev, apds_led_drive_t *drive)
{
    uint8_t d = *drive;
    status_t err = STATUS_OK;

    if (d > APDS_LEDDRIVE_MAX) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_GST_CONFIG_2, (3 << 6), (d << 6));
    }

    if (!err) {
        dev->gst_settings.gst_led_drive_str = d;
    }
    return err;
}

status_t apds_get_gst_led_drive(APDS_DEV dev, apds_led_drive_t *drive)
{
    *drive = dev->gst_settings.gst_led_drive_str;
    return STATUS_OK;
}

/******************** INTERRUPT SETTINGS *********************/

status_t apds_get_sleep_after_intr(APDS_DEV dev, uint8_t *en)
{
    *en = dev->gen_settings.sleep_after_intr;
    return STATUS_OK;
}

status_t apds_set_sleep_after_intr(APDS_DEV dev, uint8_t *en)
{
    status_t err = STATUS_OK;
    uint8_t val = *en;

    if (val) {
        err = regSetMask(dev, APDS_REGADDR_CONFIG_3, APDS_REGBIT_SLP_POST_INT);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_CONFIG_3, APDS_REGBIT_SLP_POST_INT);
    }

    if (!err) {
        dev->gen_settings.sleep_after_intr = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_als_intr(APDS_DEV dev, uint8_t *en)
{
    *en = dev->als_settings.als_intr_en;
    return STATUS_OK;
}

status_t apds_set_als_intr(APDS_DEV dev, uint8_t *en)
{
    status_t err = STATUS_OK;
    uint8_t val = *en;

    if (val) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_INT_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_ALS_INT_EN);
    }

    if (!err) {
        dev->als_settings.als_intr_en = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_prox_intr(APDS_DEV dev, uint8_t *en)
{
    *en = dev->prx_settings.prox_intr_en;
    return STATUS_OK;
}

status_t apds_set_prox_intr(APDS_DEV dev, uint8_t *en)
{
    status_t err = STATUS_OK;
    uint8_t val = *en;

    if (val) {
        err = regSetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_INT_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_ENABLE, APDS_REGBIT_PRX_INT_EN);
    }

    if (!err) {
        dev->prx_settings.prox_intr_en = (val) ? 1 : 0;
    }
    return err;
}

status_t apds_get_gst_intr(APDS_DEV dev, uint8_t *en)
{
    *en = dev->gst_settings.gst_int_en;
    return STATUS_OK;
}

status_t apds_set_gst_intr(APDS_DEV dev, uint8_t *en)
{
    status_t err = STATUS_OK;
    uint8_t val = *en;

    if (val) {
        err = regSetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_INT_EN);
    } else {
        err = regUnsetMask(dev, APDS_REGADDR_GST_CONFIG_4, APDS_REGBIT_GST_INT_EN);
    }

    if (!err) {
        dev->prx_settings.prox_intr_en = (val) ? 1 : 0;
    }
    return err;
}

/*************** FIFO Settings **********************/

status_t apds_get_gst_fifo_lvl(APDS_DEV dev, uint8_t *level)
{
    status_t status = STATUS_OK;
    uint8_t regval = 0;
    status = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_LVL, 1, &regval);
    *level = regval;
    return status;
}

status_t apds_read_fifo_data_set(APDS_DEV dev, uint8_t *index)
{
    status_t err = STATUS_OK;
    uint8_t data[4] = {0};
    uint8_t pkt_index = *index;

    if (pkt_index > 8) {
        return STATUS_ERR_INVALID_ARG;
    }

    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_U, 4, data);
    if (!err) {
        dev->data.gst_fifo_u_data[pkt_index] = data[0];
        dev->data.gst_fifo_d_data[pkt_index] = data[1];
        dev->data.gst_fifo_l_data[pkt_index] = data[2];
        dev->data.gst_fifo_r_data[pkt_index] = data[3];
    }

    return err;
}

status_t apds_read_fifo_full(APDS_DEV dev)
{
    status_t err = STATUS_OK;
    uint8_t data[APDS_FIFO_LEN_BYTES] = {0};
    uint8_t num_bytes = 0;
    uint8_t num_rows = 0;
    uint8_t j = 0;

    err = apds_get_gst_fifo_lvl(dev, &num_rows);

    if (num_rows < 1) {
        log_info(APDS_TAG, "Fifo is empty!");
        return STATUS_OK;
    }

    if (!err) {
        num_bytes = num_rows * 4;
        log_info("Reading %u bytes / %u available pkts from the fifo!\n", num_bytes, num_rows);
        err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_FIFO_U, num_bytes, data);
    }

    if (!err) {
        memset(&dev->data, 0, sizeof(apds_data_t));

        for (uint8_t i = 0; i < num_bytes; i++) {
            switch (i % 4) {
                case 0: dev->data.gst_fifo_u_data[j] = data[i]; break;
                case 1: dev->data.gst_fifo_d_data[j] = data[i]; break;
                case 2: dev->data.gst_fifo_l_data[j] = data[i]; break;
                case 3:
                    dev->data.gst_fifo_r_data[j] = data[i];
                    j++;
                    break;
                default: break;
            }
        }

        /** store the number of rows read for processing **/
        dev->gst_settings.fifo_pkts_read = num_rows;
    }

    // apds_gst_clr_fifo(dev);
    // apds_set_gst_intr(dev, &en);
    return err;
}

status_t apds_get_fifo_valid(APDS_DEV dev, bool *valid)
{
    status_t err = STATUS_OK;
    uint8_t regval = 0;
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_STATUS, 1, &regval);

    if (!err) {
        if (regval & APDS_REGBIT_GST_DATA_VALID) {
            *valid = true;
        } else {
            *valid = false;
        }
    }
    return err;
}

status_t apds_get_fifo_overflow(APDS_DEV dev, bool *ov)
{
    status_t err = STATUS_OK;
    uint8_t regval = 0;
    err = gcd_i2c_read_address(dev->bus, dev->addr, APDS_REGADDR_GST_STATUS, 1, &regval);

    if (!err) {
        if (regval & APDS_REGBIT_GST_FIFO_OVR) {
            *ov = true;
        } else {
            *ov = false;
        }
    }
    return err;
}

status_t apds_get_fifo_thresh(APDS_DEV dev, uint8_t *thr)
{
    *thr = dev->gst_settings.gst_fifo_thresh;
    return STATUS_OK;
}

status_t apds_set_fifo_thresh(APDS_DEV dev, uint8_t *thr)
{
    uint8_t thresh = *thr;
    status_t err = STATUS_OK;

    if (thresh >= APDS_FIFO_THRESH_MAX) {
        err = STATUS_ERR_INVALID_ARG;
    } else {
        err = readModWrite(dev, APDS_REGADDR_GST_CONFIG_1, 0b11000000, (thresh << 6));
    }

    if (!err) {
        dev->gst_settings.gst_fifo_thresh = thresh;
    }

    return err;
}
