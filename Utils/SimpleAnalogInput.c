/***************************************
* \file     .c
* \brief    
* \date
* \author
****************************************/

/********* Includes *******************/
#include "port/error_type.h"
#include "port/types.h"
#include "port/log.h"
#include "port/malloc.h"
#include "port/interfaces/adc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "SimpleAnalogInput.h"

const char *RGB_TAG = "RGB_Driver";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/


SAI_HANDLE sai_init(sai_init_t *init) {


    SAI_HANDLE handle = heap_caps_calloc(1, sizeof(sai_driver_t), MALLOC_CAP_DEFAULT);
    status_t err = STATUS_OK;


    if(handle == NULL) {
        err = STATUS_ERR_NO_MEM;
        log_error("SAI", "Error assigning handle memory");
    }
    else {
        handle->adc_channel_a = init->a_chan;
        handle->adc_channel_b = init->b_chan;
        handle->adc_channel_c = init->c_chan;
        handle->width = init->width;
    }

    if(!err) {
        err = adc1_config_width(handle->width);
    }

    if(!err) {
        err = adc1_config_channel_atten(handle->adc_channel_a, handle->atten);
        err = adc1_config_channel_atten(handle->adc_channel_b, handle->atten);
        err = adc1_config_channel_atten(handle->adc_channel_c, handle->atten);
    }

    return handle;

}


static status_t sai_update_channel(SAI_HANDLE handle, uint8_t *chan) {
    
}

status_t sai_update_all_channels(SAI_HANDLE handle);

status_t sai_get_delta_thresh(SAI_HANDLE handle, uint8_t *thresh);

status_t sai_set_delta_thresh(SAI_HANDLE handle, uint8_t *thresh);

status_t sai_enable_delta_thresh_en(SAI_HANDLE handle);