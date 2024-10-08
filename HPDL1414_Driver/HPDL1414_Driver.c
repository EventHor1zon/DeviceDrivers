/***************************************
* \file     HPDL1414_Driver.c
* \brief    A simple driver for the HPDL1414 
*           micro led matrix display
*           Uses the provided character map to imply 
*           data pin levels.
* \date     Dec 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <string.h>

#include "port/error_type.h"
#include "port/log.h"
#include "esp32/rom/ets_sys.h"
#include "port/malloc.h"
#include "port/driver/gpio.h"

#include "./HPDL1414_Driver.h"

/****** Function Prototypes ***********/


/****** Global Data *******************/

const char *HPDL_TAG = "HPDL Driver";

/** the device character map **/
const char *charmap[4][16] = {
    {" ", "!", "\"", "#", "$", "%", "&", "'", "<", ">", "*", "+", ",", "-", ".", "/"},
    {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "¬", "|", "{", "=", "}", "?" },
    {"a", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "N" },
    {"P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "[", "\\" "]", "^", "_" },
};

/****** Private Functions *************/

/** get character location & infer data pin levels **/
static status_t get_pinset_from_char(char *c, uint8_t *rowmask, uint8_t *colmask) {

    /** set staus to OK if char found **/
    status_t status = STATUS_FAIL;
    uint8_t row_val = 0;
    uint8_t colval = 0;
    bool brk = false;

    /** search the char map for the character **/
    for(uint8_t  i=0; i<4; i++) {
        for(uint8_t j=0; j<16; j++) {
            if(c == charmap[i][j]) {
                row_val = i;
                colval = j;
                status = STATUS_OK;
                brk = true; /** ESCAPE!! **/
                log_info(HPDL_TAG, "Found %s at %u:%u", c, row_val, colval);
            }
            if(brk) {
                break;
            }
        }
        if(brk) {
            break;
        }
    }

    /** because of value offset, add 2 to the colmask **/
    *rowmask = row_val;
    *colmask = colval + 2;

    return status;
}


/** set a LED to specific character **/
static status_t set_char_data(hpdl_driver_t *dev, char *c) {

    status_t status = STATUS_OK;

    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t v0, v1, v2, v3, v4, v5, v6, a0, a1; 


    if(get_pinset_from_char(c, &row, &col) != STATUS_OK) {
        status= STATUS_FAIL;
    } else {
        v0 = (row & DATA_ROW_MASK_0) ? 1 : 0;
        v1 = (row & DATA_ROW_MASK_1) ? 1 : 0;
        v2 = (row & DATA_ROW_MASK_2) ? 1 : 0;
        v3 = (row & DATA_ROW_MASK_3) ? 1 : 0;
        v4 = (col & DATA_COL_MASK_0) ? 1 : 0;
        v5 = (col & DATA_COL_MASK_1) ? 1 : 0;
        v6 = (col & DATA_COL_MASK_2) ? 1 : 0;

        a0 = (dev->current_led & 1);
        a1 = (dev->current_led & 2) >> 1;
        /** set the address lines **/
        log_info(HPDL_TAG, "Setting address lines: 0:%u 1:%u", a0, a1);
        STATUS_ERROR_CHECK(gpio_set_level(dev->adrpins.A0, a0));
        STATUS_ERROR_CHECK(gpio_set_level(dev->adrpins.A1, a1));
        /** short delay, then set write low **/
        ets_delay_us(1);
        STATUS_ERROR_CHECK(gpio_set_level(dev->write_pin, 0));
        ets_delay_us(1);

        /** short wait then set data pins **/
        log_info(HPDL_TAG, "Setting data lines: %u %u %u %u : %u %u %u", v0, v1, v2, v3, v4, v5, v6);
        STATUS_ERROR_CHECK(gpio_set_level(dev->rowpins.DR0, v0));
        STATUS_ERROR_CHECK(gpio_set_level(dev->rowpins.DR1, v1));
        STATUS_ERROR_CHECK(gpio_set_level(dev->rowpins.DR2, v2));
        STATUS_ERROR_CHECK(gpio_set_level(dev->rowpins.DR3, v3));
        STATUS_ERROR_CHECK(gpio_set_level(dev->colpins.DC4, v4));
        STATUS_ERROR_CHECK(gpio_set_level(dev->colpins.DC5, v5));
        STATUS_ERROR_CHECK(gpio_set_level(dev->colpins.DC6, v6));

        /** short delay, then write high **/
        ets_delay_us(1);
        STATUS_ERROR_CHECK(gpio_set_level(dev->write_pin, 1));
    }

    return status;
}


/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
hpdl_driver_t *hpdl_init(hpdl_initdata_t *init) 
#else
hpdl_driver_t *hpdl_init(hpdl_driver_t *handle, hpdl_initdata_t *init)
#endif
{

    status_t status = STATUS_OK;

    /** make a pinmask of the outputs **/
    uint32_t pinmask;

#ifdef CONFIG_DRIVERS_USE_HEAP
    hpdl_driver_t *handle = (hpdl_driver_t *)heap_caps_calloc(1, sizeof(hpdl_driver_t), MALLOC_CAP_8BIT);
    if(handle == NULL) {
        log_error(HPDL_TAG, "Error assigning memory for driver handle");
        status = STATUS_ERR_NO_MEM;
    }
#else
    memset(handle, 0, sizeof(hpdl_driver_t));
#endif

    if(status == STATUS_OK) {
        pinmask = 
            (1 << (init->D0)) |
            (1 << (init->D1)) |
            (1 << (init->D2)) |
            (1 << (init->D3)) |
            (1 << (init->D4)) |
            (1 << (init->D5)) |
            (1 << (init->D6)) |
            (1 << (init->A0)) |
            (1 << (init->A1)) 
        ;
    }

    if(status == STATUS_OK) {
        handle->rowpins.DR0 = init->D0;
        handle->rowpins.DR1 = init->D1;
        handle->rowpins.DR2 = init->D2;
        handle->rowpins.DR3 = init->D3;
        handle->colpins.DC4 = init->D4;
        handle->colpins.DC5 = init->D5;
        handle->colpins.DC6 = init->D6;
        handle->adrpins.A0 = init->A0;
        handle->adrpins.A1 = init->A1;
        handle->write_pin = init->write;
        handle->current_led = 0;
    }

    gpio_config_t io_cfg = {0};
    io_cfg.intr_type = 0;
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pull_down_en = 0;
    io_cfg.pull_up_en = 0;
    io_cfg.pin_bit_mask = pinmask;

    if(status == STATUS_OK && gpio_config(&io_cfg) != STATUS_OK) {
        log_error(HPDL_TAG, "Error configuring the GPIO pins");
        status = STATUS_FAIL;
    }
    /** write all pins high **/
    status += gpio_set_level(handle->rowpins.DR0, 1);
    status += gpio_set_level(handle->rowpins.DR1, 1);
    status += gpio_set_level(handle->rowpins.DR2, 1);
    status += gpio_set_level(handle->rowpins.DR3, 1);
    status += gpio_set_level(handle->colpins.DC4, 1);
    status += gpio_set_level(handle->colpins.DC5, 1);
    status += gpio_set_level(handle->colpins.DC6, 1);
    status += gpio_set_level(handle->adrpins.A0, 1);
    status += gpio_set_level(handle->adrpins.A1, 1);
    status += gpio_set_level(handle->write_pin, 1);

    if(status == STATUS_OK) {
        log_info(HPDL_TAG, "Succesfully init the driver!");
        //test(handle);
    }
    else {
        log_error(HPDL_TAG, "Failed to init the driver (Error %u)", status); 
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(handle != NULL) {
            heap_caps_free(handle);
        }
#endif
    }


    return handle;
}


status_t hpdl_set_led(hpdl_driver_t *dev, uint8_t *led) {
    
    status_t status = STATUS_OK;

    if(*led > HPDL_MAX_LED_INDEX) {
        log_error(HPDL_TAG, "Invalid led number");
        status = STATUS_ERR_INVALID_ARG;
    } else {
        dev->current_led = *led;
    }

    return status;
}


status_t hpdl_set_char(hpdl_driver_t *dev, uint8_t *c) {

    status_t status = set_char_data(dev, (char *)c);

    return status;
}


status_t hpdl_set_chars(hpdl_driver_t *dev, uint32_t *var) {
    status_t status = STATUS_OK;

    uint8_t c[HPDL_NUMLEDS] = {0};
    c[0] = (uint8_t )*var;
    c[1] = (uint8_t )*var >> 8;
    c[2] = (uint8_t )*var >> 16;
    c[3] = (uint8_t )*var >> 24;

    for(uint8_t i=0; i < HPDL_NUMLEDS; i++) {
        dev->current_led = i;
        if(set_char_data(dev, (char *)&c[i]) != STATUS_OK){
            /** if char fails, return err but dont stop **/
            status = STATUS_ERR_INVALID_ARG;
        }
    }

    return status;
}
