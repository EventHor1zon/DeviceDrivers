/****************************************
 * \file     DeviceRegistry.h
 * \brief    Header file for the Peripheral manager.c
 *
 *   DeviceRegistry is one of the main components of ESP_Home system
 *   It acts as a bridge btween the connected peripherals and the API manager
 *   It creates the main command queue and the control task is blocked waiting for
 *   an incomming command from the API manager
 *
 *
 *
 * \date     Sept 2020
 * \author   RJAM
 ****************************************/

#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

/********* Includes ********************/
#include "DeviceRegistryAPI.h"
#include "error_types.h"
#include "port/driver/include/port_gpio.h"
#include "port/interfaces/include/port_i2c.h"
#include "port/rtos_primitives/include/port_rtos.h"
#include "port_log.h"
#include "port_types.h"

/********* Definitions *****************/

#define PM_MAX_PERIPHERALS    0x0F
#define PM_QUEUE_SEND_TIMEOUT 1000

typedef enum {
    DR_ERR_INVALID_ID = 0x80,
    DR_ERR_INVALID_ARG = 0x81,
    DR_ERR_INVALID_TYPE = 0x82,
    DR_ERR_INVALID_CMD = 0x83,
    DR_ERR_INVALID_PERIPH_ID = 0x84,
    DR_ERR_INVALID_PARAM_ID = 0x85,
    DR_ERR_INVALID_CMD_ARGS = 0x86,
    DR_ERR_INVALID_METHOD = 0x87,
    DR_ERR_SET_OUT_OF_BOUNDS = 0x88
} dev_reg_err_t;

#define DR_ERR_GET_FAILED_BASE 0x90
#define DR_ERR_SET_FAILED_BASE 0xA0
#define DR_ERR_ACT_FAILED_BASE 0xB0

typedef struct peripheral_summary {
    peripheral_t *peripherals;
    uint8_t perip_num;
} peripheral_summary_t;

typedef struct dev_reg_init {
    queuetype_t request_queue;   // the command input queue, expecting items of 'cmd_request_t'
    queuetype_t response_queue;  // command response queue, outputting items of 'cmd_rsp_t'
} dev_reg_init_t;

/********** Types **********************/

/******** Function Definitions *********/

/** peripheral_manager_init()
 *
 *  initialises the peripheral manager
 *
 *  \return error code
 **/
status_t device_registry_init(dev_reg_init_t *init_data);

/** dev_reg_add_new_peripheral();
 *  \brief register a new peripheral with the PM
 *  \param template - a pointer to the Peripheral_t template
 *  \param id - the peripheral ID
 *  \param handle - a pointer to the periph handle
 *  \return ESP_OK or error
 **/
status_t dev_reg_add_new_peripheral(peripheral_t *template, uint8_t id, void *handle);

/** dev_reg_handle_parameter_request
 *  \brief - returns a cmd_rsp+t response to a command request
 *  \param - pointer to a request to process
 **/
cmd_rsp_t dev_reg_handle_parameter_request(cmd_request_t *request);

/**
 *  \brief: Returns pointer to a peripheral from the peripheral id
 *  \param periph_id - the peripheral id to look up
 *  \return peripheral_t * or NULL
 *
 **/
peripheral_t *get_peripheral_from_id(uint32_t periph_id);

/**
 *  \brief: returns a pointer to a parameter_t struct from given peripheral and parameter id
 *  \param periph - pointer to the peripheral to search
 *  \param param_id - the parameter id to look up
 *  \return parameter pointer or NULL
 **/
parameter_t *get_parameter_from_id(peripheral_t *periph, uint8_t param_id);

#ifdef CONFIG_USE_EVENTS

/** \brief returns the peripheral managers' event loop
 *  \return handle to event loop
 */
esp_event_loop_handle_t dev_reg_get_event_loop(void);

/** DEBUG: remove **/
int dev_reg_test_print(void *args);

/** \brief adds an event to the linked list of events
 *  \param map_init a pointer to an event_map_init_t struct
 *  \return ESP_OK or ERROR
 */
status_t add_event_map(event_map_init_t *map_init);

#endif

#endif /* PERIPHERAL_MANAGER_H */
