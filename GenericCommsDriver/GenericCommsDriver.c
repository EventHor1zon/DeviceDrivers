/***************************************
* \file     GenericCommsDriver.c
* \brief    Some generic i2c/spi/other comms setup and 
*           read/write functions
*
* \date     Aug 2020
* \author   RJAM
****************************************/

/********* Includes *******************/

#include "GenericCommsDriver.h"

#include "port/error_type.h"
#include "port/log.h"

#include "port/interfaces/uart.h"
#include "port/interfaces/spi.h"
#include "port/interfaces/spi.h"
#include "port/interfaces/i2c.h"


#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/****** Function Prototypes ***********/

/************ ISR *********************/

const char *COMMS_TAG = "GEN_COMMS";

/****** Private Data ******************/

static gcd_status_t gcd = {0};
QueueHandle_t uart_queue = NULL;

/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

bool gcd_i2c_check_bus(uint8_t bus) {
    if(bus >= I2C_NUM_MAX) {
        return false;
    } 
    return true;
}


bool gcd_spi_check_bus(uint8_t spi_bus) {

    bool result = false;
    if (spi_bus == HSPI_HOST || spi_bus == VSPI_HOST) {
        result = true;
    }
    return result;
}


status_t gcd_i2c_bus_claim(uint8_t i2c_channel) {
    status_t lock_status = STATUS_OK;
    SemaphoreHandle_t sempr = NULL;

    if(!gcd_i2c_check_bus(i2c_channel)) {
        log_error(COMMS_TAG, "Invalid i2c bus %d", i2c_channel);
        lock_status = STATUS_ERR_INVALID_ARG;
    }
    else if((i2c_channel == I2C_NUM_0) && gcd.i2c0_sem != NULL) {
        sempr = gcd.i2c0_sem;
    }
    else if((i2c_channel == I2C_NUM_1) && gcd.i2c1_sem != NULL) {
        sempr = gcd.i2c1_sem;
    }

    if(sempr != NULL) {
        if(xSemaphoreTake(sempr, pdMS_TO_TICKS(GCD_SEMAPHORE_TIMEOUT)) != pdTRUE) {
            log_error(COMMS_TAG, "Could not get i2c %u semaphore in time", i2c_channel);
            lock_status = STATUS_ERR_TIMEOUT;
        }
    }

    return lock_status;
}


status_t gcd_i2c_bus_unclaim(uint8_t i2c_channel) {
    status_t lock_status = STATUS_OK;
    SemaphoreHandle_t sempr = NULL;

    if(!gcd_i2c_check_bus(i2c_channel)) {
        log_error(COMMS_TAG, "Invalid i2c bus %d", i2c_channel);
        lock_status = STATUS_ERR_INVALID_ARG;
    }
    else if((i2c_channel == I2C_NUM_0) && gcd.i2c0_sem != NULL) {
        sempr = gcd.i2c0_sem;
    }
    else if((i2c_channel == I2C_NUM_1) && gcd.i2c1_sem != NULL) {
        sempr = gcd.i2c1_sem;
    }

    if(sempr != NULL) {
        xSemaphoreGive(sempr);
    }

    return lock_status;
}


status_t gcd_i2c_read_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t readLen, uint8_t *rxBuffer)
{
    status_t txStatus = STATUS_OK;
    SemaphoreHandle_t sempr = NULL;

    /** if bus using a semaphore, take it first **/
    if(!gcd_i2c_check_bus(i2cChannel)) {
        log_error(COMMS_TAG, "Invalid i2c bus %d", i2cChannel);
        txStatus = STATUS_ERR_INVALID_ARG;
    }
    else if((i2cChannel == I2C_NUM_0) && gcd.i2c0_sem != NULL) {
        sempr = gcd.i2c0_sem;
    }
    else if((i2cChannel == I2C_NUM_1) && gcd.i2c1_sem != NULL) {
        sempr = gcd.i2c1_sem;
    }

    if(sempr != NULL) {
        if(xSemaphoreTake(sempr, pdMS_TO_TICKS(GCD_SEMAPHORE_TIMEOUT)) != pdTRUE) {
            log_info(COMMS_TAG, "Could not get i2c %u semaphore in time", i2cChannel);
            txStatus = STATUS_ERR_TIMEOUT;
        }
    }
    
    if(txStatus == STATUS_OK) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, deviceAddr << 1 | 0, 1);
        i2c_master_write_byte(cmd, regAddr, 1);
        i2c_master_start(cmd);
        
        i2c_master_write_byte(cmd, deviceAddr << 1 | 1, 1);
        if (readLen > 1)
        {
            i2c_master_read(cmd, rxBuffer, readLen - 1, 0);
        }
        i2c_master_read_byte(cmd, rxBuffer + readLen - 1, 1);
        i2c_master_stop(cmd);
        txStatus = i2c_master_cmd_begin(i2cChannel, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (txStatus == STATUS_ERR_TIMEOUT)
        {
            log_warn("GenericI2C Read", "I2C Timeout error");
        } 
        if (txStatus != STATUS_OK)
        {
            log_warn("GenericI2C Read", "I2C error");
        }

        /** give back the sem **/
        if(sempr != NULL) {
            xSemaphoreGive(sempr);
        }
    }

    return txStatus;
}


status_t gcd_i2c_write_block(uint8_t i2cChannel, uint8_t deviceAddr, uint16_t writeLen, uint8_t *txBuffer) {

    status_t txStatus = STATUS_OK;
    SemaphoreHandle_t sempr = NULL;
    

    // /** if bus using a semaphore, take it first **/
    if(!gcd_i2c_check_bus(i2cChannel)) {
        log_error(COMMS_TAG, "Invalid i2c bus %d", i2cChannel);
        txStatus = STATUS_ERR_INVALID_ARG;
    }
    else if((i2cChannel == I2C_NUM_0) && gcd.i2c0_sem != NULL) {
        sempr = gcd.i2c0_sem;
    }
    else if((i2cChannel == I2C_NUM_1) && gcd.i2c1_sem != NULL) {
        sempr = gcd.i2c1_sem;
    }

    if(sempr != NULL) {
        if(xSemaphoreTake(sempr, pdMS_TO_TICKS(GCD_SEMAPHORE_TIMEOUT)) != pdTRUE) {
            log_info(COMMS_TAG, "Could not get i2c %u semaphore in time", i2cChannel);
            txStatus = STATUS_ERR_TIMEOUT;
        }
    }
    
    if(txStatus == STATUS_OK) {
        /** perform the transaction **/
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddr << 1 | I2C_MASTER_WRITE), 1);
        i2c_master_write(cmd, txBuffer, writeLen, 1);
        i2c_master_stop(cmd);

        txStatus = i2c_master_cmd_begin(i2cChannel, cmd, pdMS_TO_TICKS(GENERIC_I2C_COMMS_TIMEOUT_MS));
        if (txStatus != STATUS_OK)
        {
            log_error("gcd_i2c_write_address", "Error during transmission [%u]", txStatus);
        }
        i2c_cmd_link_delete(cmd);

        /** give back the sem **/
        if(sempr != NULL) {
            xSemaphoreGive(sempr);
        }
    }

    return txStatus;   
}


status_t gcd_i2c_write_address(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint16_t writeLen, uint8_t *txBuffer)
{

    status_t txStatus = STATUS_OK;
    SemaphoreHandle_t sempr = NULL;
    
    /** if bus using a semaphore, take it first **/
    if(!gcd_i2c_check_bus(i2cChannel)) {
        log_error(COMMS_TAG, "Invalid i2c bus %d", i2cChannel);
        txStatus = STATUS_ERR_INVALID_ARG;
    }
    else if((i2cChannel == I2C_NUM_0) && gcd.i2c0_sem != NULL) {
        sempr = gcd.i2c0_sem;
    }
    else if((i2cChannel == I2C_NUM_1) && gcd.i2c1_sem != NULL) {
        sempr = gcd.i2c1_sem;
    }

    if(sempr != NULL) {
        if(xSemaphoreTake(sempr, pdMS_TO_TICKS(GCD_SEMAPHORE_TIMEOUT)) != pdTRUE) {
            log_info(COMMS_TAG, "Could not get i2c %u semaphore in time", i2cChannel);
            txStatus = STATUS_ERR_TIMEOUT;
        }
    }

    if(txStatus == STATUS_OK) {
        /** perform the transaction **/
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddr << 1 | I2C_MASTER_WRITE), 1);
        i2c_master_write_byte(cmd, regAddr, 1);
        if(writeLen > 0) {
            i2c_master_write(cmd, txBuffer, writeLen, 1);
        }
        i2c_master_stop(cmd);

        txStatus = i2c_master_cmd_begin(i2cChannel, cmd, pdMS_TO_TICKS(GENERIC_I2C_COMMS_TIMEOUT_MS));
        if (txStatus != STATUS_OK)
        {
            log_error("gcd_i2c_write_address", "Error during transmission [%u]", txStatus);
        }
        i2c_cmd_link_delete(cmd);

        /** give back the sem **/
        if(sempr != NULL) {
            xSemaphoreGive(sempr);
        }
    }
    
    return txStatus;
}


status_t gcd_i2c_read_mod_write(uint8_t i2cChannel, uint8_t deviceAddr, uint8_t regAddr, uint8_t clear_bits, uint8_t set_bits) {

    uint8_t regval = 0;
    status_t err = gcd_i2c_read_address(i2cChannel, deviceAddr, regAddr, 1, &regval);
    if(!err) {
        regval &= ~clear_bits;
        regval |= set_bits;
        err = gcd_i2c_write_address(i2cChannel, deviceAddr, regAddr, 1, &regval);
    }
    return err;
}


status_t gcd_i2c_reg_set_bits(uint8_t channel, uint8_t dev_addr, uint8_t reg_addr, uint8_t mask) {

    status_t err = STATUS_OK;
    uint8_t regval = 0; 

    err = gcd_i2c_read_address(channel, dev_addr, reg_addr, 1, &regval);

    if(!err) {
        // check if mask is already set
        if((regval & mask) != mask) {
            regval |= mask;
            err = gcd_i2c_write_address(channel, dev_addr, reg_addr, 1, &regval);
        }
    }

    return err;
}


status_t gcd_i2c_reg_unset_bits(uint8_t channel, uint8_t dev_addr, uint8_t reg_addr, uint8_t mask) {
    status_t err = STATUS_OK;
    uint8_t regval = 0; 

    err = gcd_i2c_read_address(channel, dev_addr, reg_addr, 1, &regval);

    if(!err) {
        // check if mask is already unset
        if((regval & mask) > 0) {
            regval &= ~(mask);
            err = gcd_i2c_write_address(channel, dev_addr, reg_addr, 1, &regval);
        }
    }

    return err;   
}


status_t gcd_i2c_address_access(uint8_t channel, uint8_t dev_addr, uint8_t reg_addr) {
    // perform an address access (zero length write)
    return gcd_i2c_write_address(channel, dev_addr, reg_addr, 0, NULL);
}



status_t gcd_i2c_init(int16_t dataPin, int16_t clockPin, uint32_t clockSpeed, uint8_t busNum, bool use_smphr)
{
    log_info("GenericI2C Init", "Initialsing i2c bus");
    status_t status = STATUS_OK;

    if (clockSpeed > 1000000)
    {
        log_error("I2C Init", "Error, the clock speed too damn high!");
        status = STATUS_ERR_INVALID_ARG;
    }
    else if (busNum != I2C_NUM_0 && busNum != I2C_NUM_1)
    {
        log_error("I2C Init", "Error, invalid i2c Bus number");
        status = STATUS_ERR_INVALID_ARG;
    }
    else
    {
        log_info("GenericI2C Init", "starting driver %d %d %lu %u", dataPin, clockPin, clockSpeed, busNum);
        status = i2c_driver_install(busNum, I2C_MODE_MASTER, 0, 0, 0);
    }

    if (status == STATUS_OK)
    {
        i2c_config_t i2cConf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = dataPin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = clockPin,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = clockSpeed
            };
        status = i2c_param_config(busNum, &i2cConf);
    }
    if (status == STATUS_OK)
    {
        log_info("GenericI2C Init", "I2C driver started on bus %d", busNum);
        if(busNum == I2C_NUM_0) {
            gcd.i2c0_is_init = true;
        } 
        else if(busNum == I2C_NUM_1) {
            gcd.i2c1_is_init = true;
        }
    }

    if(use_smphr) {
        SemaphoreHandle_t sem = xSemaphoreCreateMutex();
        if(sem == NULL) {
            log_error(COMMS_TAG, "Error creating semaphore!");
            status = STATUS_ERR_NO_MEM;
        }
        else {
            if(busNum == I2C_NUM_0) {
                gcd.i2c0_sem = sem;
            }
            else if(busNum == I2C_NUM_1) {
                gcd.i2c1_sem = sem;
            }
        }
    }
    return status;
}


status_t gcd_spi_init(int16_t clk_pin, int16_t mosi_pin, int16_t miso_pin, uint8_t spi_bus, bool use_smphr) {

    status_t status = STATUS_OK;
    log_info("SPI_SETUP", "[+] Setting up SPI bus");

    status = gcd_spi_check_bus(spi_bus) ? 0 : STATUS_ERR_INVALID_ARG;
    if(status == STATUS_OK) {
        spi_bus_config_t buscfg = {0};
        buscfg.mosi_io_num = mosi_pin;
        buscfg.miso_io_num = miso_pin;
        buscfg.sclk_io_num = clk_pin;
        buscfg.max_transfer_sz = 8192; // led data + up to 10 frames of start & end
        buscfg.quadhd_io_num = -1;
        buscfg.quadwp_io_num = -1;
        buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
        buscfg.intr_flags = 0;
        status = spi_bus_initialize(spi_bus, &buscfg, 0);
        if(status) {
            log_error("GCD", "Error initialising spi bus {%u}", status);
        }
    }
    else {
        log_error("GCD", "Error invalid spi bus %u", spi_bus);
    }


    if(status == STATUS_OK && use_smphr) {
        SemaphoreHandle_t sem = xSemaphoreCreateMutex();
        if(sem == NULL) {
            log_error(COMMS_TAG, "Error creating semaphore!");
            status = STATUS_ERR_NO_MEM;
        }
        else {
            if(spi_bus == SPI2_HOST) {
                gcd.hspi_sem = sem;
            }
            else if(spi_bus == SPI3_HOST) {
                gcd.vspi_sem = sem;
            }
        }
    }

    if(status == STATUS_OK) {
        if(spi_bus == SPI2_HOST) {
            gcd.hspi_is_init = true;
        }
        else if(spi_bus == SPI3_HOST) {
            gcd.vspi_is_init = true;
        }
    }

    return status;
}



/** \brief : Provides a simple uart interface without hardware control
 * \return : STATUS_OK or error
 **/
status_t gcd_uart_init(uint8_t bus, int16_t tx_pin, int16_t rx_pin, int16_t cts_pin, int16_t rts_pin, uint32_t baud, uint8_t stop_bits, uint8_t parity) {

    status_t err = STATUS_OK;

    log_info("GCD", "Initialising UART driver");

    /** set the HW flow control from pins supplied **/
    uart_hw_flowcontrol_t flow_ctrl = 0;
    if(cts_pin > 0) {
        flow_ctrl |= (1 << 1);
    }
    if(rts_pin > 0) {
        flow_ctrl |= (1 << 0);
    }

    uart_config_t config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = flow_ctrl,
        .source_clk = UART_SCLK_APB,
    };

    err = uart_driver_install(bus, GCD_CONFIG_UART_RX_BUFF_SIZE, GCD_CONFIG_UART_TX_BUFF_SIZE, 20, &uart_queue, STATUS_INTR_FLAG_LOWMED);
    if(!err) {
        gcd.uart_queue = uart_queue;
    }
    else {
        log_error("GCD", "Error installing uart driver [%u]", err);
    }

    if(!err) {
        err = uart_param_config(bus, &config);
        if(err) {
            log_error("GCD", "Error initialising uart params on bus %u [%u]", bus, err);
        }
    }

    if(!err) {
        /** set the uart pins **/
        err = uart_set_pin(bus, tx_pin, rx_pin, rts_pin, cts_pin);
        if(err) {
            log_error("GCD", "Error configuring pins [%u]", err);
        }
    }

    if(err) {
        log_error("GCD", "Error Initialising UART driver [%u]", err);
    }
    else {
        log_info("GCD", "Initialising UART driver was a success");
    }

    return err;
}


status_t gcd_uart_transmit_blocking(uint8_t bus, uint8_t *data, uint16_t len) {

    status_t err = STATUS_OK;
    uart_write_bytes(bus, (const char *)data, (size_t)len);
    return err;
}


status_t gcd_uart_transmit_non_blocking(uint8_t bus, uint8_t *data, uint32_t len) {

    status_t err = STATUS_OK;

    if(uart_tx_chars(bus, (const char *)data, len) < len) {
        err = STATUS_ERR_NO_MEM;
    }

    return err;
}
