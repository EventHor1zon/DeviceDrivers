/**
 *    @file    port_spi.h
 *
 *    @brief    header file for port_spi
 *
 *
 *
 *    @author    RJAM
 *    @created   Wed 24 Jun 00:00:00 BST 2026
 */

#ifndef PORT_SPI_H
#define PORT_SPI_H

/** Includes **/

#include "port/error_types.h"
#include "stdint.h"

/** Defines **/

/** Typedefs **/

/** Function Declarations **/

/** @brief write a byte to spi
 *  @param spi_port [in] SPI port index
 *  @param data [in] byte to transmit
 *  @return status_t status
 */
status_t spi_write(const uint32_t spi_port, const uint8_t data);

/** @brief read a byte from spi
 *  @param spi_port [in] SPI port index
 *  @param data [out] pointer to received byte
 *  @return status_t status
 */
status_t spi_read(const uint32_t spi_port, uint8_t *const data);

/** @brief transfer a byte on spi
 *  @param spi_port [in] SPI port index
 *  @param tx_data [in] transmit byte
 *  @param rx_data [out] received byte
 *  @return status_t status
 */
status_t spi_transfer(const uint32_t spi_port, const uint8_t tx_data, uint8_t *const rx_data);

/** @brief perform a full spi transaction with user args
 *  @param spi_port [in] SPI port index
 *  @param args [in] pointer to args
 *  @param timeout [in] timeout for operation
 *  @return status_t status
 */
status_t spi_transaction(const uint32_t spi_port, const void *const args, const uint32_t timeout);

/** @brief write multiple bytes to spi
 *  @param spi_port [in] SPI port index
 *  @param data [in] pointer to source data
 *  @param len [in] number of bytes to write
 *  @return status_t status
 */
status_t spi_burst_write(const uint32_t spi_port, const uint8_t *const data, const uint32_t len);

/** @brief read multiple bytes from spi
 *  @param spi_port [in] SPI port index
 *  @param data [out] pointer to destination buffer
 *  @param len [in] number of bytes to read
 *  @return status_t status
 */
status_t spi_burst_read(const uint32_t spi_port, uint8_t *const data, const uint32_t len);

/** @brief init spi port
 *  @param spi_port [in] SPI port index
 *  @param args [in] pointer to init arguments
 *  @return status_t status
 */
status_t spi_init(const uint32_t spi_port, const void *const args);

/** @brief initialise a device on an spi port
 *  @param spi_port [in] SPI port index
 *  @param args [in] pointer to init arguments
 *  @param returned [out] pointer to return data
 *  @return status_t status
 */
status_t spi_init_device(const uint32_t spi_port, const void *const args, void *const returned);

/** END **/
#endif /** PORT_SPI_H **/