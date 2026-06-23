/**
 *    @file    port_gpio.c
 *
 *    @brief    header file for port_gpio
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:43:02 BST 2026
 */

/** Includes **/
#include "./include/port_gpio.h"

/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t __attribute__((weak))
gpio_set_level(const uint32_t gpio_port, const uint32_t gpio_pin, const uint32_t level)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) gpio_toggle_level(const uint32_t gpio_port, const uint32_t gpio_pin)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_set_open_drain(const uint32_t gpio_port, const uint32_t gpio_pin)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) gpio_set_push_pull(const uint32_t gpio_port, const uint32_t gpio_pin)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_set_direction(const uint32_t gpio_port, const uint32_t gpio_pin, const uint32_t direction)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_set_high_impedance(const uint32_t gpio_port, const uint32_t gpio_pin)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_get_level(const uint32_t gpio_port, const uint32_t gpio_pin, uint32_t *const level)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_set_pull_direction(const uint32_t gpio_port, const uint32_t gpio_pin, const uint32_t pull_dir)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
gpio_init(const uint32_t gpio_port, const uint32_t gpio_pin, const void *const args)
{
    return STATUS_NOT_IMPLEMENTED;
}

/** END **/
