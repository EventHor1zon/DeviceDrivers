/**
 *    @file    port_i2c.c
 *
 *    @brief    source file for port_i2c
 *
 *
 *
 *    @author    RJAM
 *    @created   Wed 24 Jun 00:00:00 BST 2026
 */

/** Includes **/
#include "./include/port_i2c.h"

/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t __attribute__((weak))
i2c_write(const uint32_t i2c_port, const uint32_t address, const uint8_t data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
i2c_read(const uint32_t i2c_port, const uint32_t address, uint8_t *const data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_burst_write(
    const uint32_t i2c_port,
    const uint32_t address,
    const uint8_t *const data,
    const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_burst_read(
    const uint32_t i2c_port,
    const uint32_t address,
    uint8_t *const data,
    const uint32_t len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_register_write(
    const uint32_t i2c_port,
    const uint8_t ic_address,
    const uint8_t reg,
    uint32_t *const len,
    const uint8_t *const data)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_register_read(
    const uint32_t i2c_port,
    const uint8_t ic_address,
    const uint8_t reg,
    uint32_t *const len,
    uint8_t *const buffer)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_transact(
    const uint32_t i2c_port,
    const uint32_t address,
    void *const transaction,
    void *const args)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) i2c_init(const uint32_t i2c_port, const void *const args)
{
    return STATUS_NOT_IMPLEMENTED;
}

/** END **/