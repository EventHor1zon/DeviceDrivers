/**
 *    @file    port_gpio.h
 *
 *    @brief    header file for port_gpio
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:43:16 BST 2026
 */

#ifndef PORT_GPIO_H
#define PORT_GPIO_H

/** Includes **/

#include "port/error_types.h"
#include "stdint.h"

/** Defines **/

/** Typedefs **/
typedef uint32_t gpio_port_t;
typedef uint32_t gpio_pin_t;

typedef uint32_t gpio_port_pin_t;

/** Function Declarations **/

/** @brief set the gpio output level
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @param level [in] Level to set pin (0 - low, >0 - high)
 *  @return status_t status
 */
status_t gpio_set_level(const uint32_t gpio_port, const uint32_t gpio_pin, const uint32_t level);

/** @brief toggle (flip) the gpio output level
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @return status_t status
 */
status_t gpio_toggle_level(const uint32_t gpio_port, const uint32_t gpio_pin);

/** @brief enable gpio to open drain
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @return status_t status
 */
status_t gpio_set_open_drain(const uint32_t gpio_port, const uint32_t gpio_pin);

/** @brief enable gpio to push-pull
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @return status_t status
 */
status_t gpio_set_push_pull(const uint32_t gpio_port, const uint32_t gpio_pin);

/** @brief set gpio to high impedence (High Z)
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @return status_t status
 */
status_t gpio_set_high_impedance(const uint32_t gpio_port, const uint32_t gpio_pin);

/** @brief set the gpio direction
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @param direction [in] Direction to set pin (0 - Input, 1 Ouput, 1>Implementation dependent)
 *  @return status_t status
 */
status_t gpio_set_direction(
    const uint32_t gpio_port,
    const uint32_t gpio_pin,
    const uint32_t direction);

/** @brief Get the gpio input level
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @param level [in] Pointer to level storage (level: 0 = low, >0 = high)
 *  @return status_t status
 */
status_t gpio_get_level(const uint32_t gpio_port, const uint32_t gpio_pin, uint32_t *const level);

/** @brief set the gpio pull direction
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @param level [in] Pull direction for pin (pull: 0 = down, 1 = up, >1 implementation dependent)
 *  @return status_t status
 */
status_t gpio_set_pull_direction(
    const uint32_t gpio_port,
    const uint32_t gpio_pin,
    const uint32_t pull_dir);

/** @brief init a gpio port/pin
 *  @param gpio_port [in] GPIO port index (zero if not required)
 *  @param gpio_pin [in]  GPIO pin number
 *  @param args [in] pointer to init arguments
 *  @return status_t status
 */
status_t gpio_init(const uint32_t gpio_port, const uint32_t gpio_pin, const void *const args);

/** END **/
#endif /** PORT_GPIO_H **/
