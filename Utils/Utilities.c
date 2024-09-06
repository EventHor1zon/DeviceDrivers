/***************************************
* \file .c
* \brief
*
* \date
* \author
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "port/types.h"
#include "port/error_type.h"
#include "port/log.h"

#include "Utilities.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/
void showmem(uint8_t *memptr, uint16_t len)
{
    for (int i = 0; i < len; i++)
    {
        if (i % 8 == 0)
        {
            log_info("[%p] %02x", memptr, *memptr);
        }
        else if (i % 8 == 7)
        {
            log_info(" %02x\n", *memptr);
        }
        else
        {
            log_info(" %02x", *memptr);
        }
        memptr++;
    }
}

void printByteBits(uint8_t num)
{
    log_info("[");
    for (int bit = 0; bit < 8; bit++)
    {
        log_info("%d", num & 0x01);
        num = num >> 1;
    }
    log_info("]\n");
}

void printBytesOrderExplicit(uint8_t val) {
    uint8_t set = 0;
    log_info("MSB ------> LSB\n");
    log_info("7 6 5 4 3 2 1 0\n");
    for(int bit = 0; bit < 8; bit++) {
        set = (val & (1 << (7 - bit))) > 0 ? 1 : 0;
        log_info("%u ", set);
    }
    log_info("\n");
}


uint8_t set_bits(uint8_t byte, uint8_t set) {
    return (byte | set);
}

uint8_t unset_bits(uint8_t byte, uint8_t unset) {
    return (byte & ~(unset));
}


void replaceBits(uint8_t *data, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t a = *data;
    a &= ~(clear_mask);
    a |= set_mask;
    *data = a;
}

uint8_t largest_from_array(uint8_t *array, uint8_t len) {

    uint8_t max = array[0];
    for(uint8_t i=1; i < len; i++) {
        if(array[i] > max) {
            max = array[i];
        }
    }
    return max;
}

uint8_t index_of_largest(uint8_t *array, uint8_t len) {
    uint8_t max = array[0];
    uint8_t max_index = 0;
    for(uint8_t i=0; i < len; i++) {
        if(array[i] > max) {
            max = array[i];
            max_index = i;
        }
    }
    return max_index;
}