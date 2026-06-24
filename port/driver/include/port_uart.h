/**
 *    @file    port_uart.c
 *
 *    @brief    source file for port_uart
 *
 *
 *
 *    @author    RJAM
 *    @created   Wed 24 Jun 00:00:00 BST 2026
 */

/** Includes **/
#include "./include/port_uart.h"

/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t __attribute__((weak)) uart_write(const uint32_t uart_port, const uint8_t data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) uart_read(const uint32_t uart_port, uint8_t *const data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
uart_burst_write(const uint32_t uart_port, const uint8_t *const data, const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
uart_burst_read(const uint32_t uart_port, uint8_t *const data, const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) uart_available(const uint32_t uart_port, uint32_t *const available)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) uart_init(const uint32_t uart_port, const void *const args)
{
    return STATUS_NOT_IMPLEMENTED;
}

/** END **/