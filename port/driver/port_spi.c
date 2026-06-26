/**
 *    @file    port_spi.c
 *
 *    @brief    source file for port_spi
 *
 *
 *
 *    @author    RJAM
 *    @created   Wed 24 Jun 00:00:00 BST 2026
 */

/** Includes **/
#include "./include/port_spi.h"

/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t __attribute__((weak)) spi_write(const uint32_t spi_port, const uint8_t data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) spi_read(const uint32_t spi_port, uint8_t *const data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
spi_transfer(const uint32_t spi_port, const uint8_t tx_data, uint8_t *const rx_data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
spi_transaction(const uint32_t spi_port, const void *const args, const uint32_t timeout)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
spi_burst_write(const uint32_t spi_port, const uint8_t *const data, const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
spi_burst_read(const uint32_t spi_port, uint8_t *const data, const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) spi_init(const uint32_t spi_port, const void *const args)
{
    return STATUS_NOT_IMPLEMENTED;
}

/** END **/