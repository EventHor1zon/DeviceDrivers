/***************************************
* \file   CCS811.c
* \brief  Driver for the CCS811 VOC/CO2 
*          gas sensor
*
* \date     Dec 2020
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "port/error_type.h"
#include "port/log.h"
#include "port/malloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GenericCommsDriver.h"
#include "CCS811_Driver.h"

const char *CCS_TAG = "CSS DRIVER";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

void ccs_interrupt_handler(void *args) {
    return;
}

static status_t ccs_is_new_data(ccs811_Device_t *dev, bool *data) {

    return STATUS_ERR_NOT_SUPPORTED;

}

static status_t ccs_config_pins(gpio_num_t rst, gpio_num_t wake) {

    status_t status = STATUS_OK;

    uint32_t gpio_mask = ((1 << rst) | (1 << wake));

    gpio_config_t io_init = {0};

    io_init.intr_type = 0;
    io_init.mode = GPIO_MODE_OUTPUT;
    io_init.pin_bit_mask = gpio_mask;
    io_init.pull_up_en = GPIO_PULLUP_DISABLE;
    io_init.pull_up_en = GPIO_PULLDOWN_DISABLE;

    if(gpio_config(&io_init) != STATUS_OK) {
        log_error(CCS_TAG, "Error configuring GPIO");
        status = STATUS_FAIL;
    } 

    if(gpio_set_level(rst, 1) != STATUS_OK || gpio_set_level(wake, 1) != STATUS_OK) {
        log_error(CCS_TAG, "Error setting gpio level");
        status = STATUS_FAIL;
    }

    return status;

}

static status_t ccs_config_intr_pins(gpio_num_t intr_gpio, ccs_intr_t type, bool init_interrupts) {
    
    status_t status = STATUS_OK;

    uint32_t pin_mask = (1 << intr_gpio);

    gpio_config_t io_init = {0};

    io_init.intr_type = GPIO_INTR_NEGEDGE;
    io_init.mode = GPIO_MODE_INPUT;
    io_init.pin_bit_mask = pin_mask;
    io_init.pull_up_en = GPIO_PULLUP_ENABLE;
    io_init.pull_up_en = GPIO_PULLDOWN_DISABLE;

    if(gpio_config(&io_init) != STATUS_OK) {
        log_error(CCS_TAG, "Error configuring Intr GPIO");
        status = STATUS_FAIL;
    } 
    
    if(init_interrupts && status == STATUS_OK) {
        status = gpio_install_isr_service(STATUS_INTR_FLAG_LOWMED);
    }
    
    if(status == STATUS_OK) {
        status = gpio_isr_handler_add(intr_gpio, ccs_interrupt_handler, NULL);
    }

    return status;
}


static void ccs811_driver_task(void *args) {
 
    ccs811_Device_t *dev = (ccs811_Device_t *)args;

   while(1) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   /** here be dragons **/
}


/****** Private Functions *************/


/****** Global Data *******************/

/****** Global Functions *************/


ccs811_Device_t *css811_init(ccs811_init_t *init) {

    status_t status = STATUS_OK;

    ccs811_Device_t *handle = heap_caps_calloc(1, sizeof(ccs811_Device_t), MALLOC_CAP_8BIT); 
    if(handle == NULL) {
        log_info(CCS_TAG, "Error initialising struct memory");
        status = STATUS_ERR_NO_MEM;
    }
    else {
        handle->comms_channel = init->i2c_channel;
        handle->dev_addr = init->addr_pin_lvl > 0 ? CCS_DEVICE_ADDRESS_HIGH : CCS_DEVICE_ADDRESS_LOW;
        handle->mode = CCS_MODE_IDLE;
    }

    if(status == STATUS_OK && init->intr_type) {
        status = ccs_config_intr_pins(init->gpio_intr, init->intr_type, false);
    }

    if(status == STATUS_OK) {
        status = ccs_config_pins(init->gpio_nreset, init->gpio_nwake);        
    }

    if(status == STATUS_OK && xTaskCreate(ccs811_driver_task, "ccs811_driver_task", configMINIMAL_STACK_SIZE, (void *)handle, 3, handle->t_handle) != pdTRUE) {
        log_error(CCS_TAG, "Error initialising the driver task!");
        status = STATUS_ERR_NO_MEM;
    }

    if(status != STATUS_OK) {
        log_info(CCS_TAG, "Error starting the CCS driver :(");
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(handle != NULL ) {
            heap_caps_free(handle);
        }
#endif
    }

    return handle;
}