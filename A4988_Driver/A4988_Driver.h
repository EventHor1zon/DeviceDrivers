/****************************************
* @file     A4988_Driver.h
* @brief    Header file
* @date     May 22
* @author   RJAM
****************************************/

#ifndef A4988_DRIVER_H
#define A4988_DRIVER_H

/********* Includes ********************/


#include "esp_err.h"
#include "esp_types.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "sdkconfig.h"

/********* Definitions *****************/

#ifdef CONFIG_USE_PERIPH_MANAGER

#include "CommandAPI.h"
#define a4988_param_len 9

const parameter_t a4988_param_map[a4988_param_len];
const peripheral_t a4988_periph_template;

#endif

/**
 * @defgroup A4988_definitions
 * @brief    A4988 definitions
 * @{
 * 
 **/

#define A4988_DEFAULT_STEP_PULSE_LEN 1
#define A4988_DEFAULT_STEP_DELAY 100
#define A4988_CONFIG_PTMR_DIV 80    /** 80MHz apb clock / 80 = 1MHz, microsecond timer **/

#define A4988_CONFIG_MAX_SUPPORTED_DEVICES  4
#define A4988_CONFIG_QUEUE_LEN  8
#define A4988_CONFIG_PULSE_LEN  100 /** pulse length in microseconds **/
#define A4988_CONFIG_SHORT_WAIT pdMS_TO_TICKS(10)

#define TIMER_GRP_FROM_INDEX(i) (i >> 1)
#define TIMER_ID_FROM_INDEX(i) (i & 1)

/** @enum step_size_t 
 *  @brief Defines the step size types **/
typedef enum {
    FULL_STEP_T = 0,
    HALF_STEP_T = 1,
    QURT_STEP_T = 2,
    EGTH_STEP_T = 3,
    SXTN_STEP_T = 7,
} step_size_t;


/** @enum a4988_cmd_t
 *  @brief command types for the command queue
 * 
 */
typedef enum {
    A4988_CMD_STEP_TMR,
    A4988_CMD_PULSE_TMR,
    A4988_CMD_UPDATE_PERIOD
} a4988_cmd_t;

/** @struct a4988_init_t 
 *  @brief Defines the initialisation data 
 */

typedef struct A4988_Init
{
    
    gpio_num_t sleep;       /*!< sleep pin - optional **/
    gpio_num_t enable;      /*!< enable pin - optional **/
    gpio_num_t step;        /*!< step pin - must be set **/
    gpio_num_t rst;         /*!< rst pin  - must be set **/
    gpio_num_t dir;         /*!< dir pin    must be set **/
    gpio_num_t ms1;         /*!< ms1 pin  - optional **/
    gpio_num_t ms2;         /*!< ms2 pin  - optional **/
    gpio_num_t ms3;         /*!< ms3 pin  - optional **/
    step_size_t step_size;  /*!< step size - if driver controls 
                                ms pins, leave at zero, else describes 
                                the ms pin states
                            **/
} a4988_init_t;

/**  
 * @struct Structure containing the driver information
 * @brief The driver handle structure
 */
typedef struct A4988_Driver
{
    gpio_num_t sleep;       /**< sleep pin - optional **/
    gpio_num_t enable;      /**< enable pin - optional **/
    gpio_num_t step;        /**< step pin - must be set **/
    gpio_num_t rst;         /**< rst pin  - must be set **/
    gpio_num_t dir;         /**< dir pin    must be set **/
    gpio_num_t ms1;         /**< ms1 pin  - optional **/
    gpio_num_t ms2;         /**< ms2 pin  - optional **/
    gpio_num_t ms3;         /**< ms3 pin  - optional **/

    uint32_t step_pulse_len;    /**< step pulse hold time **/
    uint32_t steps_queued;      /**< number of steps queued  **/
    uint16_t step_wait;         /**< delay between executing queued steps **/


    bool is_sleeping;       /**< is device sleeping **/
    bool is_enabled;        /**< is device enabled **/
    bool direction;         /**< current direction 
                                (depends on wiring, dir is "true/false" for
                                software purposes ) 
                            **/
    bool _en;               /**< driver manages enabled pin **/
    bool _sleep;            /**< driver manages sleep pin **/
    bool _ms;               /**< driver manages 3 microstep select pins **/

    step_size_t step_size;  /**< step size setting **/
    TimerHandle_t step_timer;    /**< timer handle **/
    uint8_t pulse_timer;    /**< timer id **/
    uint8_t index;

} a4988_handle_t;


/** @brief Device handle type **/
typedef a4988_handle_t * A4988_DEV;

typedef struct {
    A4988_DEV dev;
    a4988_cmd_t cmd;
} a4988_msg_t;


/** @} A4988_definitions */


/******** Function Definitions *********/
/** @defgroup A4988_Driver_functions
 *  @brief    A4988 Driver function definitions
 *  @{
 */

/**
 * @brief Initialises the A4988 Driver and returns the driver handle
 * @param   dev     Pointer to device handle 
 * @param   init    Pointer to a populated A4988_init_t
 * @return  Device handle or NULL on error
 */
#ifdef CONFIG_DRIVERS_USE_HEAP
A4988_DEV a4988_init(a4988_init_t *init);
#else
A4988_DEV a4988_init(A4988_DEV dev, a4988_init_t *init);
#endif

/**
 * @brief Take a single step in the configured direction
 *
 * @param   dev - the A4988 driver device handle
 * @return  ESP_OK or Error
 */
status_t a4988_step(A4988_DEV dev);

/**
 * @brief Resets the IC
 *
 * @param   dev - the A4988 driver device handle
 * @return  ESP_OK or Error
 */
status_t a4988_reset(A4988_DEV dev);

/**
 * @brief  Clears currently queued steps
 *
 * @param   dev - the A4988 driver device handle
 * @return  ESP_OK or Error
 */
status_t a4988_clear_step_queue(A4988_DEV dev);

/**
 * @brief Get the number of steps in the queue
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_queued_steps(A4988_DEV dev, uint16_t *steps);

/**
 * @brief Set the number of steps in the queue
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value
 * @return  ESP_OK or Error
 */
status_t a4988_set_queued_steps(A4988_DEV dev, uint16_t *steps);

/**
 * @brief Get the currently configured step size
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_stepsize(A4988_DEV dev, uint8_t *sz);

/**
 * @brief Set the step size - the driver must control the step
 *          size pins
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_set_stepsize(A4988_DEV dev, uint8_t *sz);

/**
 * @brief Get the sleep state
 *
 * @param   dev - the A4988 driver device handle
 * @param   slp - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_sleepstate(A4988_DEV dev, bool *slp);

/**
 * @brief Set the device sleep state. The driver must control the 
 *          sleep pin in order to set this value
 *
 * @param   dev - the A4988 driver device handle
 * @param   slp - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_set_sleepstate(A4988_DEV dev, bool *slp);

/**
 * @brief Get the device's enabled status
 *
 * @param   dev - the A4988 driver device handle
 * @param   en - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_enable(A4988_DEV dev, bool *en);

/**
 * @brief Set the device's enable state - the driver must control the 
 *          enable pin in order to set this value
 *
 * @param   dev - the A4988 driver device handle
 * @param   steps - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_set_enable(A4988_DEV dev, bool *en);

/**
 * @brief Get the currently configured delay between steps
 *
 * @param   dev - the A4988 driver device handle
 * @param   sz - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_step_delay(A4988_DEV dev, uint16_t *sz);

/**
 * @brief Set the delay between steps
 *
 * @param   dev - the A4988 driver device handle
 * @param   sz - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_set_step_delay(A4988_DEV dev, uint16_t *sz);

/**
 * @brief Get the currently configured direction
 *
 * @param   dev - the A4988 driver device handle
 * @param   dir - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_get_direction(A4988_DEV dev, bool *dir);

/**
 * @brief Set the direction
 *
 * @param   dev - the A4988 driver device handle
 * @param   dir - pointer to value storage
 * @return  ESP_OK or Error
 */
status_t a4988_set_direction(A4988_DEV dev, bool *dir);

/** @} A4988_Driver_functions */


#endif /* A4988_DRIVER_H */
