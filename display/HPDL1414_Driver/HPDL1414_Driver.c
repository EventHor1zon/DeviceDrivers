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
#include "HPDL1414_Driver.h"
#include "stm32f401xc.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#include <stdarg.h>
#include <string.h>

/****** Function Prototypes ***********/

/****** Global Data *******************/
#define HPDL_GPIO_DATA_PORT GPIOA
#define HPDL_GPIO_UTIL_PORT GPIOB

UART_HandleTypeDef huart1;

static void simple_print(char *msg, uint8_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100000);
}

/** the device character map **/
const unsigned char charmap[4][16] = {
    {' ', '!', '\'', '#', '$', '%', '&', '\'', '<', '>', '*', '+', ',', '-', '.', '/'},
    {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '{', '=', '}', '?'},
    {'a', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'N'},
    {'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', '\\', ']', '^', '_'},
};

/****** Private Functions *************/

/** get character location & infer data pin levels **/
static uint8_t get_pinset_from_char(uint8_t c, uint8_t *rowmask, uint8_t *colmask)
{
    /** set staus to OK if char found **/
    uint8_t status = 1;
    uint8_t row_val = 0xFF;
    uint8_t colval = 0xFF;
    uint8_t brk = 0;

    if (c > 95) {
        return 1;
    }
    /** search the char map for the character **/
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 16; j++) {
            if (c == (uint8_t)charmap[i][j]) {
                row_val = i;
                colval = j;
                status = 0;
                brk = 1; /** ESCAPE!! **/
                break;
            }
        }
        if (brk) {
            break;
        }
    }

    /** because of value offset, add 2 to the colmask **/
    *rowmask = row_val + 2;
    *colmask = colval;

    return status;
}

/** set a LED to specific character **/
static return_t set_char_data(hpdl_driver_t *dev, uint8_t c)
{
    return_t status = 0;

    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t v0, v1, v2, v3, v4, v5, v6, a0, a1;

    if (get_pinset_from_char(c, &row, &col) != 0) {
        simple_print("lookup failed\n", 15);
        status = 1;
    } else {
        simple_print("lookup gooded\n", 15);
        v0 = (col & DATA_COL_MASK_0) ? 1 : 0;
        v1 = (col & DATA_COL_MASK_1) ? 1 : 0;
        v2 = (col & DATA_COL_MASK_2) ? 1 : 0;
        v3 = (col & DATA_COL_MASK_3) ? 1 : 0;
        v4 = (row & DATA_ROW_MASK_0) ? 1 : 0;
        v5 = (row & DATA_ROW_MASK_1) ? 1 : 0;
        v6 = (row & DATA_ROW_MASK_2) ? 1 : 0;

        a0 = (dev->current_led & 1) ? 1 : 0;
        a1 = (dev->current_led & 2) ? 1 : 0;
        /** set the address lines **/
        // log_thingI(HPDL_TAG, "Setting address lines: 0:%u 1:%u", a0, a1);
        HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->adrpins.A0, a0);
        HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->adrpins.A1, a1);

        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC0, v0);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC1, v1);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC2, v2);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC3, v3);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR4, v4);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR5, v5);
        HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR6, v6);

        HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->write_pin, 0);
        for (uint32_t i = 0; i < 32000; i++) {
            __ASM volatile("nop");
        }
        HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->write_pin, 1);
    }

    return status;
}

static void test_mode(hpdl_driver_t *dev)
{
    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->write_pin, 0);
    /** set the address lines **/
    // log_thingI(HPDL_TAG, "Setting address lines: 0:%u 1:%u", a0, a1);
    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->adrpins.A0, 0);
    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->adrpins.A1, 0);

    /** short wait then set data pins **/
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC0, 0);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC1, 1);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC2, 0);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->colpins.DC3, 1);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR4, 0);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR5, 1);
    HAL_GPIO_WritePin(HPDL_GPIO_DATA_PORT, dev->rowpins.DR6, 0);

    for (uint32_t i = 0; i < 32000; i++) {
        __ASM volatile("nop");
    }

    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, dev->write_pin, 1);
}

/****** Global Functions *************/

hpdl_driver_t *hpdl_init(hpdl_driver_t *handle, hpdl_initdata_t *init)
{
    uint8_t status = 0;

    memset(handle, 0, sizeof(hpdl_driver_t));

    if (status == 0) {
        handle->colpins.DC0 = init->D0;
        handle->colpins.DC1 = init->D1;
        handle->colpins.DC2 = init->D2;
        handle->colpins.DC3 = init->D3;
        handle->rowpins.DR4 = init->D4;
        handle->rowpins.DR5 = init->D5;
        handle->rowpins.DR6 = init->D6;
        handle->adrpins.A0 = init->A0;
        handle->adrpins.A1 = init->A1;
        handle->write_pin = init->write;
        handle->current_led = 0;
    }

    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, handle->adrpins.A0, 1);
    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, handle->adrpins.A1, 1);
    HAL_GPIO_WritePin(HPDL_GPIO_UTIL_PORT, handle->write_pin, 1);

    // if (status == 0) {
    //     log_thingI(HPDL_TAG, "Succesfully init the driver!");
    //     // test(handle);
    // } else {
    //     log_thingE(HPDL_TAG, "Failed to init the driver (Error %u)", status);
    // }

    test_mode(handle);

    return handle;
}

return_t hpdl_set_led(hpdl_driver_t *dev, uint8_t *led)
{
    return_t status = 0;

    if (*led > HPDL_MAX_LED_INDEX) {
        status = 1;
    } else {
        dev->current_led = *led;
    }

    return status;
}

return_t hpdl_set_char(hpdl_driver_t *dev, uint8_t *c)
{
    return_t status = set_char_data(dev, *c);

    return status;
}

return_t hpdl_set_chars(hpdl_driver_t *dev, uint32_t *var)
{
    return_t status = 0;

    uint8_t c[HPDL_NUMLEDS] = {0};
    c[0] = (uint8_t)*var;
    c[1] = (uint8_t)*var >> 8;
    c[2] = (uint8_t)*var >> 16;
    c[3] = (uint8_t)*var >> 24;

    for (uint8_t i = 0; i < HPDL_NUMLEDS; i++) {
        dev->current_led = i;
        if (set_char_data(dev, c[i]) != 0) {
            /** if char fails, return err but dont stop **/
            status = 1;
        }
    }

    return status;
}
