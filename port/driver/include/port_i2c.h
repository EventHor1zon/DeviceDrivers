/**
 *    @file    port_i2c.h
 *
 *    @brief    header file for port_i2c
 *
 *
 *
 *    @author    RJAM
 *    @created   Wed 24 Jun 00:00:00 BST 2026
 */

#ifndef PORT_I2C_H
#define PORT_I2C_H

/** Includes **/

#include "port/error_types.h"
#include "stdint.h"

/** Defines **/

/** Typedefs **/

/** Function Declarations **/

/** @brief write a byte to i2c device
 *  @param i2c_port [in] I2C port index
 *  @param address [in] slave address
 *  @param data [in] byte to transmit
 *  @return status_t status
 */
status_t i2c_write(const uint32_t i2c_port, const uint32_t address, const uint8_t data);

/** @brief read a byte from i2c device
 *  @param i2c_port [in] I2C port index
 *  @param address [in] slave address
 *  @param data [out] pointer to received byte
 *  @return status_t status
 */
status_t i2c_read(const uint32_t i2c_port, const uint32_t address, uint8_t *const data);

/** @brief write multiple bytes to i2c device
 *  @param i2c_port [in] I2C port index
 *  @param address [in] slave address
 *  @param data [in] pointer to source data
 *  @param len [in] number of bytes to write
 *  @return status_t status
 */
status_t i2c_burst_write(
    const uint32_t i2c_port,
    const uint32_t address,
    const uint8_t *const data,
    const uint32_t len);

/** @brief read multiple bytes from i2c device
 *  @param i2c_port [in] I2C port index
 *  @param address [in] slave address
 *  @param data [out] pointer to destination buffer
 *  @param len [in] number of bytes to read
 *  @return status_t status
 */
status_t i2c_burst_read(
    const uint32_t i2c_port,
    const uint32_t address,
    uint8_t *const data,
    const uint32_t len);

/** @brief read/write multiple bytes from i2c device
 *  @param i2c_port [in] I2C port index
 *  @param address [in] slave address
 *  @param transaction [in] pointer to a transaction descriptor struct
 *  @param args [in] pointer to port args
 *  @return status_t status
 */
status_t i2c_transact(
    const uint32_t i2c_port,
    const uint32_t address,
    void *const transaction,
    void *const args);

/** @brief init i2c port
 *  @param i2c_port [in] I2C port index
 *  @param args [in] pointer to init arguments
 *  @return status_t status
 */
status_t i2c_init(const uint32_t i2c_port, const void *const args);

/** END **/
#endif /** PORT_I2C_H **/