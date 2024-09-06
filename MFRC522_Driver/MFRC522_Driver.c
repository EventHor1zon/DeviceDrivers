/***************************************
* \file     MFRC522_Driver.c
* \brief    A driver for the MFRC RFID read/write IC
*
*           For ease of repeatetive programming, this driver only works with spi/i2c and expects valid 
*           spi/i2c driver specs. This driver does not start the i2c/spi devices so the user should configure
*           the peripherals themselves.
*
*           Thoughts:
*           To communicate with picc need to load data to send into fifo, then 
*           transceive. 
*           From arduino library (https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp)
*           - set command = Idle
*           - clear all comms irq bits
*           - flush fifo 
*           - write Tx data to fifo
*           - set bitFraming
*           - execute transceive command
*           - set the startSend bit in BitFramingReg
*           - wait for done interrupt or timeout
*           - check the error register
*           - read fifo level
*           - read fifo data
*           - get valid bits in last received byte
*           - if check crc, check crc
*
*           After that, read the card serial, etc. Add more as the 
*           driver progresses. Get it working first!
*
* \date     November 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "port/types.h"
#include "port/error_type.h"
#include "port/log.h"
#include "Utilities.h"
#include "GenericCommsDriver.h"
#include "port/malloc.h"
#include "port/interfaces/spi.h"
#include "port/interfaces/spi.h"

#include "MFRC522_Driver.h"

/****** Global Data *******************/


const char *MFRC_TAG = "MFRC Driver";

/****** Function Prototypes ***********/

/** Read the status register bytes **/
static status_t mfrc_read_status1_register(MFRC_DEV dev, uint8_t *regvals);

/** Read the modem state from the status register **/
static status_t mfrc_read_modem_state(MFRC_DEV dev, mfrc_modemstate_t *modemstate);

/** Read the two crc result bytes and returns them **/
static uint16_t mfrc_read_crc_result(MFRC_DEV dev);


static status_t mfrc_read_error_register(MFRC_DEV dev, uint8_t *errors);

/************ ISR *********************/

void mfrc_irq_function(void *args) {
    MFRC_DEV dev = (MFRC_DEV)args;
    log_info("ISR\n");
    BaseType_t higherPrioWoken = pdFALSE;
    vTaskNotifyGiveFromISR(dev->isr_task, &higherPrioWoken);

    
}

/****** Private Data ******************/

/****** Private Functions *************/


static status_t mfrc_read_byte_from_address(MFRC_DEV dev, uint8_t address, uint8_t *buffer) {

    status_t err = STATUS_OK;

    if(dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        spi_transaction_t trx = {0};
        trx.tx_data[0] = (address << 1 | (1 << 7));
        trx.tx_data[1] = 0;

        trx.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);
        trx.length = 16;
        trx.rxlength = 16;

        err = spi_device_polling_transmit(dev->comms_handle, &trx);
#if DEBUG_MODE 
        log_info(MFRC_TAG, "Sent: %02x to address %02x Received: %02x %02x ", trx.tx_data[1] >> 1, trx.tx_data[0], trx.rx_data[0], trx.rx_data[1]);
#endif 
        if(!err) {
            *buffer = trx.rx_data[1];
        }
    }
    else if (dev->comms_mode == MFRC_I2C_COMMS_MODE) {
        err = gcd_i2c_read_address(dev->comms_bus, MFRC_I2C_ADDRESS, address, 1, buffer);
    }
    else {
        err = STATUS_ERR_INVALID_ARG;
    }

    return err;
}


static status_t mfrc_write_to_fifo(MFRC_DEV dev, uint8_t length, uint8_t *data) {

    status_t err = STATUS_OK;

    if (length > MFRC_FIFO_SIZE_BYTES) {
        err = STATUS_ERR_INVALID_ARG;
    }

    if(!err && dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        uint8_t buffer[MFRC_FIFO_SIZE_BYTES + 1] = {0};
        memset(buffer, ((MRFC_REGADDR_FIFO_DATA << 1) & ~(1 << 7)), 1);
        memcpy(&buffer[1], data, sizeof(uint8_t) * length);

        spi_transaction_t trx = {0};
        trx.tx_buffer = buffer;
        trx.rx_buffer = NULL;
        trx.flags = 0;
        trx.length = length * 8;

        err = spi_device_polling_transmit(dev->comms_handle, &trx);

#if DEBUG_MODE 
        log_info(MFRC_TAG, "Wrote %u bytes from the fifo", length);
#endif 

    }
    else if (!err && dev->comms_mode == MFRC_I2C_COMMS_MODE) {
        err = gcd_i2c_write_address(dev->comms_bus, MFRC_I2C_ADDRESS, MRFC_REGADDR_FIFO_DATA, length, data);
    }
    else {
        err = STATUS_ERR_INVALID_ARG;
    }
    return err;

}


static status_t mfrc_read_from_fifo(MFRC_DEV dev, uint8_t length) {

    status_t err = STATUS_OK;
    memset(dev->raw_data, 0, sizeof(dev->raw_data));
    
    if (length > MFRC_FIFO_SIZE_BYTES) {
        err = STATUS_ERR_INVALID_ARG;
    }

    if(!err && dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        uint8_t dummy_buffer[MFRC_FIFO_SIZE_BYTES + 1] = {0};
        /** write the fifo address into the buffer buffer repeatedly **/
        memset(dummy_buffer, ((MRFC_REGADDR_FIFO_DATA << 1) | (1 << 7)), sizeof(dummy_buffer));
        
        spi_transaction_t trx = {0};
        trx.tx_buffer = dummy_buffer;
        trx.rx_buffer = dev->raw_data;
        trx.flags = 0;
        trx.length = length * 8;

        err = spi_device_polling_transmit(dev->comms_handle, &trx);

#if DEBUG_MODE 
        log_info(MFRC_TAG, "Read %u bytes from the fifo ", length);
#endif 
    }
    else if (!err && dev->comms_mode == MFRC_I2C_COMMS_MODE) {
        err = gcd_i2c_read_address(dev->comms_bus, MFRC_I2C_ADDRESS, MRFC_REGADDR_FIFO_DATA, length, dev->raw_data);
    }
    else {
        err = STATUS_ERR_INVALID_ARG;
    }

    return err;
}


static status_t mfrc_write_byte_to_address(MFRC_DEV dev, uint8_t address, uint8_t value) {

    status_t err = STATUS_OK;

    if(dev->comms_mode == MFRC_SPI_COMMS_MODE) {
        spi_transaction_t trx = {0};
        trx.tx_data[0] = ((address << 1) & ~(1 << 7));
        trx.tx_data[1] = value;

        trx.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA);
        trx.length = 16;
        trx.rxlength = 0;

        err = spi_device_polling_transmit(dev->comms_handle, &trx);
#if DEBUG_MODE 
        log_info(MFRC_TAG, "Wrote: %02x to address %02x", trx.tx_data[0], trx.tx_data[1] >> 1);
#endif 
    }
    else if (dev->comms_mode == MFRC_I2C_COMMS_MODE) {
        err = gcd_i2c_write_address(dev->comms_bus, MFRC_I2C_ADDRESS, address, 1, &value);
    }
    else {
        err = STATUS_ERR_INVALID_ARG;
    }

    return err;
}


static status_t mfrc_read_mod_write_register(MFRC_DEV dev, uint8_t address, uint8_t clear_bits, uint8_t set_bits, uint8_t *newval) {
    uint8_t read_byte = 0;
    uint8_t write_byte = 0;
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, address, &read_byte);

    if(!err) {
        write_byte = read_byte & ~(clear_bits);
        write_byte |= set_bits;
    }

    if(!err) {
        if(write_byte != read_byte) {
            err = mfrc_write_byte_to_address(dev, address, write_byte);
        }
#ifdef DEBUG_MODE
        else {
            log_info(MFRC_TAG, "Not writing, write byte is equal to read byte (r:%02x, w:%02x)", read_byte, write_byte);
        }
#endif /** DEBUG_MODE **/
    }

    if(newval != NULL) {
        *newval = write_byte;
    }

    return err;
}


static status_t mfrc_read_error_register(MFRC_DEV dev, uint8_t *errors) {

    status_t err = STATUS_OK;
    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_ERRORS, errors);
    return err;
}


static status_t mfrc_read_comm_irq_register(MFRC_DEV dev, uint8_t *isr) {
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_COMM_IRQ, isr);

    return err;
}


static status_t mfrc_read_div_irq_register(MFRC_DEV dev, uint8_t *isr) {
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_DIV_IRQ, isr);

    return err;
}


static mfrc_modemstate_t mfrc_get_modemstate(MFRC_DEV dev) {
    uint8_t regval = 0;
    status_t err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_STATUS_2, &regval);

    return MFRC_MODEMSTATE_FROM_REG(regval);
}


static void reset_device(MFRC_DEV dev) {
    if(dev->rst_en) {
        gpio_set_level(dev->rst_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(dev->rst_pin, 1);
    }
}

static status_t mfrc_read_status1_register(MFRC_DEV dev, uint8_t *regvals) {
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_STATUS_1, regvals);

    return err;
}

static status_t mfrc_read_status2_register(MFRC_DEV dev, uint8_t *regvals) {
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_STATUS_2, regvals);

    return err;
}

static uint8_t mfrc_get_fifo_byte_count(MFRC_DEV dev) {
    uint8_t count = 0;
    mfrc_read_byte_from_address(dev, MRFC_REGADDR_FIFO_LEVEL, &count);
    count &= ~(0b10000000);
    return count;
}

static status_t mfrc_flush_fifo(MFRC_DEV dev) {
    uint8_t regval = (1 << 7);
    status_t err = mfrc_write_byte_to_address(dev, MRFC_REGADDR_FIFO_LEVEL, regval);
    return err;
}


static status_t mfrc_read_fifo(MFRC_DEV dev) {

    status_t err = STATUS_OK;

    uint8_t count = 0;

    count = mfrc_get_fifo_byte_count(dev);

    if(count > 0) {

        err = mfrc_read_from_fifo(dev, count);
    }

    return err;
}

static status_t process_div_interrupts(MFRC_DEV dev, uint8_t byteval) {
    return STATUS_ERR_NOT_SUPPORTED;
}

static status_t clear_comm_interrupts(MFRC_DEV dev) {
    uint8_t val = 0x7F;
    status_t err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMM_IRQ, 0x7F, val, NULL);
    return err;
}

static status_t start_transceive_send(MFRC_DEV dev) {
    uint8_t val = (1 << 7);
    status_t err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_BIT_FRAMING, (1 << 7), val, &dev->map.bitframe_reg.regByte);
    return err;
}


status_t transceive_with_target(MFRC_DEV dev) {
    /** largely inspired by the arduino library **/
    
    status_t err = STATUS_OK;
    mfrc_cmd_t cmd = MFRC_COMMAND_IDLE;
    uint8_t valid_bits = 7;
    PICC_Command_t command = PICC_CMD_REQA;
    uint8_t interrupts = (MFRC_ISR_RX_DONE | MFRC_ISR_TX_DONE | MFRC_ISR_IDLE | MFRC_ISR_TMR);
    bool en = true;
    uint16_t pre = 0xA9;
    uint16_t tmr = 0x3E8;
    uint8_t isr_read = 0;
    mfrc_crc_preset_t crc = MFRC_CRC_PRESET_6363;

    log_info(MFRC_TAG, "Transceiving with target - sending %02x", command);

    err = mfrc_set_command(dev, &cmd);

    if(!err) {
        err = mfrc_set_tmr_auto_en(dev, &en);
    }

    if(!err) {
        err = clear_comm_interrupts(dev);
    }

    if(!err) {
        err = mfrc_flush_fifo(dev);
    }

    if(!err) {
        err = mfrc_set_ask(dev, &en);
    }

    if(!err) {
        err = mfrc_set_crc_base(dev, &crc);
    }

    if(!err) {
        /** set timer to a 100ms timeout **/
        err = mfrc_set_tmr_prescaler(dev, &pre);
    }

    if(!err) {
        err = mfrc_set_tmr_reload(dev, &tmr);
    }

    if(!err) {
        err = mfrc_write_to_fifo(dev, 1, &command);
    }

    if(!err) {
        err = mfrc_set_tx_lastbits(dev, &valid_bits);
    }

    if(!err && dev->intr_en) {
        err = mfrc_set_comm_interrupt_mask(dev, &interrupts);
    }

    if(!err) {
        cmd = MFRC_COMMAND_TRANSCEIVE;
        err = mfrc_set_command(dev, &cmd);
    }

    if(!err) {
        err = start_transceive_send(dev);
    }
    
    if(!err) {
        dev->tx_in_progress = true;
        dev->rx_in_progress = true;
        dev->trx_in_progress = true;
    }

    log_info(MFRC_TAG, "Started Tx'ing data...");

    for(uint8_t i=0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        err = mfrc_read_comm_irq_register(dev, &isr_read);

        if(!err) {
            log_info(MFRC_TAG, "Interrupt register value: %02x", isr_read);
            if((isr_read & MFRC_ISR_TMR)) {
                log_info(MFRC_TAG, "Timer isr read");
            }
            else if((isr_read & MFRC_ISR_IDLE)) {
                log_info(MFRC_TAG, "IDLE isr read");
            }
            else if((isr_read & MFRC_ISR_TX_DONE)) {
                log_info(MFRC_TAG, "TxDone isr read");
            }
            else if((isr_read & MFRC_ISR_RX_DONE)) {
                log_info(MFRC_TAG, "RxDone isr read");
            }
            else {
                ;
            }

            if(isr_read > 0) {
                log_info(MFRC_TAG, "Clearing interrupts");
                err = clear_comm_interrupts(dev);
            }
        }
    }

    return err;
}


/** \brief This task runs to process interrupts 
 *          For an interrupt based driver, this task can unblock other tasks
 *          and update the device model.
 * 
 *          Block forever waiting on interrupt pin
 *  
 *          Should check the comms and driver irq masks to decide if
 *          we need to read the interrupt registers. 
 * 
 * **/
static void mfrc_irq_task(void *args) {

    MFRC_DEV dev = (MFRC_DEV)args;
    uint32_t notify = 0;
    status_t err = STATUS_OK;
    uint8_t comm_isr = 0, dev_isr = 0;
    uint8_t errors = 0;
    while(1) {

        notify = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        log_info(MFRC_TAG, "Received an interrupt [%u]", notify);

        if(notify) {
            /** check the interrupt registers
             **/
            if(dev->comms_intr_mask > 0) {
                err = mfrc_read_comm_irq_register(dev, &comm_isr);
                if(comm_isr > 0) {

                    if(comm_isr & MFRC_ISR_TMR) {
                        err = STATUS_ERR_NOT_SUPPORTED;
                    }
                    else if (comm_isr & MFRC_ISR_ERR) {
                        err = mfrc_read_error_register(dev, &errors);
                    }
                    else if (comm_isr & MFRC_ISR_FIFO_LO) {
                        err = STATUS_ERR_NOT_SUPPORTED;
                    }
                    else if (comm_isr & MFRC_ISR_FIFO_HI) {
                        dev->rx_available = true;
                    }
                    else if (comm_isr & MFRC_ISR_IDLE) {
                        log_info(MFRC_TAG, "Idle interrupt");
                        dev->rx_in_progress = false;
                        dev->tx_in_progress = false;
                        dev->trx_in_progress = false;
                        dev->current_cmd = MFRC_CMD_IDLE;
                        xTaskNotifyGive(dev->task);
                    }
                    else if (comm_isr & MFRC_ISR_RX_DONE) {
                        log_info(MFRC_TAG, "Rx complete interrupt");
                        dev->rx_in_progress = false;
                        xTaskNotifyGive(dev->task);
                    }
                    else if (comm_isr & MFRC_ISR_TX_DONE) {
                        log_info(MFRC_TAG, "Tx complete interrupt");
                        dev->tx_in_progress = false;
                        xTaskNotifyGive(dev->task);
                    }
                    else {
                        ;
                    }
                }
            }
        }
    }

}



static void mrfc_driver_task(void *args) {
 
    MFRC_DEV dev = (MFRC_DEV)args;
    uint32_t notify = 0;
    status_t err = STATUS_OK;
    uint8_t byteval = 0;

    while(1) {

        if(dev->intr_en) {
            if(dev->trx_in_progress) {
                /** wait to be unblocked **/
                notify = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(MFRC_DEFAULT_TIMEOUT_MS));
                if(notify) {
                    log_info(MFRC_TAG, "TRX complete!");
                }
                else {
                    log_error(MFRC_TAG, "TRX Timeout");
                }
            }
            
            else if(dev->rx_in_progress) {
                /** wait to be unblocked **/
                notify = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(MFRC_DEFAULT_TIMEOUT_MS));
                if(notify) {
                    log_info(MFRC_TAG, "Rx complete!");
                }
                else {
                    log_error(MFRC_TAG, "Rx Timeout");
                }
            }
            
            else if (dev->tx_in_progress) {
                /** wait to be unblocked **/
                notify = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(MFRC_DEFAULT_TIMEOUT_MS));
                if(notify) {
                    log_info(MFRC_TAG, "Tx complete!");
                }
                else {
                    log_error(MFRC_TAG, "Tx Timeout");
                }
            }

            else {
                /** nothing to be done **/
                ;
            }
        }

        else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    /** here be dragons **/
}


/****** Global Functions *************/

#ifdef CONFIG_DRIVERS_USE_HEAP
MFRC_DEV mfrc_init(mfrc_init_t *init) 
#else
MFRC_DEV mfrc_init(MFRC_DEV handle, mfrc_init_t *init) 
#endif

{

    MFRC_DEV handle = NULL;
    gpio_config_t pin_config = {0};
    status_t err = STATUS_OK;

    if(init->comms_mode >= MFRC_COMMS_MODE_INVALID) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else if(init->comms_mode == MFRC_SPI_COMMS_MODE && init->ss_pin == 0) {
        log_error(MFRC_TAG, "Error, no slave select pin supplied for SPI interface");
        err = STATUS_ERR_INVALID_ARG;
    }

    /** create handle **/
    if(!err) {
#ifdef CONFIG_DRIVERS_USE_HEAP
        MFRC_DEV handle = heap_caps_calloc(1, sizeof(MFRC522_Driver_t), MALLOC_CAP_DEFAULT);
        if(handle == NULL) {
            err = STATUS_ERR_NO_MEM;
            log_error(MFRC_TAG, "Error - unable to assign heap memory for handle [%u]", err);
        }
#else
        memset(handle, 0, sizeof(MFRC522_Driver_t));
#endif
    }


    if(!err) {
        handle->comms_bus = init->comms_bus;
        handle->comms_mode = init->comms_mode;
        handle->irq_pin = init->irq_pin;
        handle->rst_pin = init->rst_pin;
    }


    if(!err && handle->comms_mode == MFRC_SPI_COMMS_MODE) {

        spi_device_handle_t dev_handle;
        spi_device_interface_config_t dev = {0};
        dev.clock_speed_hz = 100000;
        dev.mode = 0;
        dev.spics_io_num = init->ss_pin;
        dev.flags = 0; /** msb first & full duplex are default **/
        dev.duty_cycle_pos = 128;
        dev.queue_size = 1;

        err = spi_bus_add_device(handle->comms_bus, &dev, &dev_handle);
    
        if(err) {
            log_error(MFRC_TAG, "Error adding device to the spi bus! [%u]", err);
        }
        else {
            handle->comms_handle = (void *)dev_handle;
        }
    }

    /** initialise the pins **/
    if(!err) {
    
        pin_config.intr_type = GPIO_INTR_DISABLE;
        pin_config.mode = GPIO_MODE_OUTPUT;
        pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        pin_config.pull_up_en = GPIO_PULLUP_ENABLE;

        /** reset pin optional, init **/
        if(init->rst_pin > 0) {
            pin_config.pin_bit_mask = (1 << handle->rst_pin);
            err = gpio_config(&pin_config);
            if(err) {
                log_error(MFRC_TAG, "Error configuring reset pin {%u}", err);
            }
            else {
                handle->rst_en = true;
                log_info(MFRC_TAG, "Reset enabled on pin %u", handle->rst_pin);
            }
        }

        /** interrupt pin, optional init **/
        if(!err && init->irq_pin) {
            pin_config.pin_bit_mask = (1 << handle->irq_pin);
            pin_config.intr_type = GPIO_INTR_NEGEDGE;
            pin_config.mode = GPIO_MODE_INPUT;

            err = gpio_config(&pin_config);
            if(err) {
                log_error(MFRC_TAG, "Error configuring irq pin {%u}", err);
            }
            else {
                err = gpio_isr_handler_add(init->irq_pin, mfrc_irq_function, handle);
                if(err) {
                    log_error(MFRC_TAG, "Error adding isr handler {%u}", err);
                }
                else {
                    handle->irq_en = true;
                    log_info(MFRC_TAG, "Interrupt enabled on pin %u", handle->rst_pin);
                }
            }
        }
    }
    

    /** create task **/
    if(!err) {
        if(xTaskCreate(mrfc_driver_task, "mrfc_driver_task", 5012, handle, 3, &handle->task) != pdTRUE) {
            log_error(MFRC_TAG, "Error creating driver task!");
            err = STATUS_ERR_NO_MEM;
        }
    }

    if(!err && handle->irq_en) {
        if(xTaskCreate(mfrc_irq_task, "mrc_irq_task", 5012, handle, 2, &handle->isr_task) != pdTRUE) {
            log_error(MFRC_TAG, "Error creating irq task!");
            err = STATUS_ERR_NO_MEM;            
        }
    }

    if(err) {
        log_error(MFRC_TAG, "Error initialising MFRC522 driver [%u]", err);
#ifdef CONFIG_DRIVERS_USE_HEAP
        if(handle) {
            heap_caps_free(handle);
        }
#endif
    } 
    else {
        log_info(MFRC_TAG, "Succesfully started MFRC522 Driver");

    if(!err && handle->rst_en) {
        reset_device(handle);
    }

#ifdef DEBUG_MODE
        log_info(MFRC_TAG, "Attempting to read from a register...");
        uint8_t byteval = 0;
        err = mfrc_read_byte_from_address(handle, MRFC_REGADDR_STATUS_1, &byteval);
        if(err) {
            log_error(MFRC_TAG, "Error reading from register [%u]", err);
        }
        else {
            log_info(MFRC_TAG, "Read %02x from status 1 register, should be 0x21", byteval);
        }

        if(!err) {
            log_info(MFRC_TAG, "Attempting a write/confirm read of a register");
            err = mfrc_read_byte_from_address(handle, MRFC_REGADDR_COMM_INTR_EN, &byteval);
            log_info(MFRC_TAG, "Pre-read : %02x (expected 0x80)", byteval);
            byteval = 0b11;
            err = mfrc_read_mod_write_register(handle, MRFC_REGADDR_COMM_INTR_EN, 0b11, byteval, NULL);
            err = mfrc_read_byte_from_address(handle, MRFC_REGADDR_COMM_INTR_EN, &byteval);
            log_info(MFRC_TAG, "Post-read : %02x (expected 0x83)", byteval);
        }
#endif /** DEBUG_MODE **/
    }

    return handle;
}




void mfrc_deinit(MFRC_DEV dev) {

    if(dev != NULL) {
#if defined(INCLUDE_vTaskDelete)
        vTaskDelete(dev->task);
#elif defined(INCLUDE_vTaskSuspend)
        vTaskSuspend(dev->task);
#endif /** INCLUDE_vTaskDelete **/
        heap_caps_free(dev);
    }

    return;
}


status_t mfrc_set_command(MFRC_DEV dev, mfrc_cmd_t *cmd) {
    uint8_t c = *cmd;
    status_t err = STATUS_OK;

    if(c != MFRC_COMMAND_IDLE           &&
       c != MFRC_COMMAND_MEM            &&
       c != MFRC_COMMAND_RAND_ID        &&
       c != MFRC_COMMAND_CALC_CRC       &&
       c != MFRC_COMMAND_TRANSMIT       &&
       c != MFRC_COMMAND_NO_CHANGE      &&
       c != MFRC_COMMAND_RECEIVE        &&
       c != MFRC_COMMAND_TRANSCEIVE     &&
       c != MFRC_COMMAND_MF_AUTHENT     &&
       c != MFRC_COMMAND_SOFT_RESET
    ) {
        err = STATUS_ERR_INVALID_ARG;
    }

    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMMAND, 0x0f, c, &dev->map.command_reg.regval);

    if(!err) {
        dev->current_cmd = c;
    }

    return err;
}



status_t mfrc_set_comm_interrupt_mask(MFRC_DEV dev, uint8_t *intr) {
    status_t err = STATUS_OK;
    uint8_t isr = *intr;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMM_INTR_EN, 0x7F, isr, &dev->map.com_irq_reg.regval);

    if(!err) {
        dev->comms_intr_mask = isr;
    }

    return err;
}

status_t mfrc_get_comm_interrupt_mask(MFRC_DEV dev, uint8_t *intr) {
    status_t err = STATUS_OK;

    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_COMM_INTR_EN, intr);

    return err;
}

status_t mfrc_set_powerdown(MFRC_DEV dev) {
    status_t err = STATUS_OK;
    uint8_t val = (1 << 4);
    val |= MFRC_COMMAND_NO_CHANGE;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMMAND, 0x0f, val, &dev->map.command_reg.regval);
    if(!err) {
        dev->power_en = false;
    }
    return err;
}

status_t mfrc_unset_powerdown(MFRC_DEV dev) {
    status_t err = STATUS_OK;
    uint8_t val = 0;
    val |= MFRC_COMMAND_NO_CHANGE;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMMAND, 0x0f, val, &dev->map.command_reg.regval);
    if(!err) {
        dev->power_en = true;
    }
    return err;
}

status_t mfrc_enable_receiver(MFRC_DEV dev) {
    uint8_t value = 0; 
    status_t err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMMAND, (1 << 5), value, &dev->map.command_reg.regval);
    return err;
}

status_t mfrc_disable_receiver(MFRC_DEV dev) {
    status_t err = STATUS_OK;
    uint8_t value = (1 << 5);
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COMMAND, (1 << 5), value, &dev->map.command_reg.regval);
    return err;
}


status_t mfrc_set_ask(MFRC_DEV dev, bool *val) {
    status_t err = STATUS_OK;
    uint8_t v = *val ? (1 << 6) : 0;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_TX_ASK, (1 << 6), v, &dev->map.tx_ask_reg.regByte);
    
    return err;
}

status_t mfrc_get_ask(MFRC_DEV dev, bool *val) {
    status_t err = STATUS_OK;

    *val = dev->map.tx_ask_reg.regBits.Force100ASK;
    return err;
}

status_t mfrc_set_crc_base(MFRC_DEV dev, mfrc_crc_preset_t *set) {
    status_t err = STATUS_OK;
    uint8_t s = *set;
    if(s >= MFRC_CRC_PRESET_INVALID) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_MODE, 0b11, s, &dev->map.mode_register.regByte);
    }

    return err;
}

status_t mfrc_get_crc_base(MFRC_DEV dev, mfrc_crc_preset_t *set) {
    status_t err = STATUS_OK;
    *set = dev->map.mode_register.regBits.crc_preset;
    return err;
}

status_t mfrc_set_antenna_en(MFRC_DEV dev) {
    status_t err = STATUS_OK;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_TX_CONTROL, 0b11, 0x03, &dev->map.tx_control_reg.regByte);
    return err;
}

status_t mfrc_clear_collisions(MFRC_DEV dev) {
    status_t err = STATUS_OK;
    uint8_t val = 0;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_COLLISION_DETECT, (1 << 7), val, &dev->map.collision_reg.regByte);
    return err;
}


status_t mfrc_get_opendrain_en(MFRC_DEV dev, bool * en) {
    status_t err = STATUS_OK;


    return err;
} 

status_t mfrc_set_opendrain_en(MFRC_DEV dev, bool * en) {
    status_t err = STATUS_OK;


    return err;
}



status_t mfrc_get_rx_uart_sel(MFRC_DEV dev, mfrc_uart_sel_t *sel) {
    status_t err = STATUS_OK;
    *sel = dev->map.rx_sel_reg.regBits.UartSel;

    return err;
}

status_t mfrc_set_rx_uart_sel(MFRC_DEV dev, mfrc_uart_sel_t *sel) {
    status_t err = STATUS_OK;
    uint8_t val = *sel;
    if(val >= MFRC_UART_SEL_INVALID) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        val = (val << 6);
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_RX_SELECT, (0x03 << 6), val, &dev->map.rx_sel_reg.regByte);
    }

    return err;
}

status_t mfrc_get_rx_wait(MFRC_DEV dev, uint8_t *wait) {
    status_t err = STATUS_OK;
    *wait = dev->map.rx_sel_reg.regBits.RxWait;
    return err;
}

status_t mfrc_set_rx_wait(MFRC_DEV dev, uint8_t *wait) {
    status_t err = STATUS_OK;
    uint8_t val = *wait;
    if(val > 0x3F) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_RX_SELECT, 0x3F, val, &dev->map.rx_sel_reg.regByte);
    }
    return err;
}

status_t mfrc_get_rx_min_level(MFRC_DEV dev, uint8_t *min) {
    status_t err = STATUS_OK;
    *min = dev->map.rx_thresh_reg.regBits.MinLevel;

    return err;
}

status_t mfrc_set_rx_min_level(MFRC_DEV dev, uint8_t *min) {
    status_t err = STATUS_OK;
    uint8_t val = *min;

    if(val > 0x0F) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        val = (val << 4);
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_RX_SELECT, 0xF0, val, &dev->map.rx_thresh_reg.regByte);
    }
    return err;
}

status_t mfrc_get_coll_level(MFRC_DEV dev, uint8_t *level) {
    status_t err = STATUS_OK;
    *level = dev->map.rx_thresh_reg.regBits.CollLevel;

    return err;
}

status_t mfrc_set_coll_level(MFRC_DEV dev, uint8_t *level) {
    status_t err = STATUS_OK;
    uint8_t val = *level;
    if(val > 0b111) {
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_RX_THRESH, 0b111, val, &dev->map.rx_thresh_reg.regByte);
    }

    return err;
}

status_t mfrc_get_add_iq_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    *en = dev->map.demod_reg.regBits.AddIQ;
    return err;
}

status_t mfrc_set_add_iq_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    uint8_t val = *en;
    
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_DEMODULATOR, (0x03 << 6), (val << 6), &dev->map.demod_reg.regByte);

    return err;
}

status_t mfrc_get_fix_iq_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    *en = dev->map.demod_reg.regBits.AddIQ;
    return err;
}

status_t mfrc_set_fix_iq_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    uint8_t val = *en;
    
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_DEMODULATOR, (0x01 << 5), (val << 5), &dev->map.demod_reg.regByte);

    return err;
}


status_t mfrc_get_parity_disable(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    *en = dev->map.mf_rx_reg.regBits.ParityDisable;
    return err;
}

status_t mfrc_set_parity_disable(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    uint8_t val = *en;
    
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_DEMODULATOR, (0x01 << 4), (val << 4), &dev->map.mf_rx_reg.regByte);

    return err;
}


status_t mfrc_set_rx_align(MFRC_DEV dev, uint8_t *align) {
    status_t err = STATUS_OK;
    uint8_t val = *align;
    if(val != 0 && val != 1 && val != 7) {
        err = STATUS_ERR_INVALID_ARG;
    }

    else {
        dev->map.bitframe_reg.regBits.rx_align = val;

        err = mfrc_read_mod_write_register(dev, 
                                            MRFC_REGADDR_BIT_FRAMING, 
                                            0b1110000, 
                                            dev->map.bitframe_reg.regByte, 
                                            &dev->map.bitframe_reg.regByte
                                            );
    }

    return err;
}

status_t mfrc_get_rx_align(MFRC_DEV dev, uint8_t *align) {
    status_t err = STATUS_OK;
    *align = dev->map.bitframe_reg.regBits.rx_align;

    return err;
}


status_t mfrc_get_tx_lastbits(MFRC_DEV dev, uint8_t *bits) {
    status_t err = STATUS_OK;
    *bits = dev->map.bitframe_reg.regBits.tx_last_bits;

    return err;
}

status_t mfrc_set_tx_lastbits(MFRC_DEV dev, uint8_t *bits) {
    status_t err = STATUS_OK;
    uint8_t val = *bits;
    if(val > 0b111) {
        err = STATUS_ERR_INVALID_ARG;
    }

    else {
        dev->map.bitframe_reg.regBits.tx_last_bits = val;

        err = mfrc_read_mod_write_register(dev, 
                                            MRFC_REGADDR_BIT_FRAMING, 
                                            0b111, 
                                            dev->map.bitframe_reg.regByte, 
                                            &dev->map.bitframe_reg.regByte);
    }

    return err;
}


status_t mfrc_get_tmr_prescale_even(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;
    *en = dev->map.demod_reg.regBits.TPrescalEven;

    return err;
}

status_t mfrc_set_tmr_prescale_even(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;

    uint8_t val = *en;

    err = mfrc_read_mod_write_register(dev,
                                       MRFC_REGADDR_DEMODULATOR,
                                       (1 << 4), 
                                       val, 
                                       &dev->map.bitframe_reg.regByte
                                       );

    return err;
}


status_t mfrc_set_tmr_auto_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;
    uint8_t val = *en;
    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_DEMODULATOR, (1 << 7), (val << 7), &dev->map.tmode_reg.value.regbyte);    

    return err;
}

status_t mfrc_get_tmr_auto_en(MFRC_DEV dev, bool *en) {
    status_t err = STATUS_OK;
    *en = dev->map.tmode_reg.value.regbits.tmr_auto;

    return err;
}

status_t mfrc_set_tmr_prescaler(MFRC_DEV dev, uint16_t *ps) {
    status_t err = STATUS_OK;
    uint16_t pre = *ps;

    /** check we're using 12 bits max **/
    if(pre > 0xfff) {
        log_info(MFRC_TAG, "Limiting prescaler to maximum (%u)", 0xfff);
        pre = 0xfff;
    }

    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_TMR_MODE, 0x0f, (uint8_t )(pre >> 8), NULL);

    if(!err) {
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_TMR_PRESCALE, 0xff, (uint8_t )pre, NULL);
    }

    return err;
}

status_t mfrc_get_tmr_prescaler(MFRC_DEV dev, uint16_t *ps) {
    status_t err = STATUS_OK;

    *ps = (((uint16_t )dev->map.tmode_reg.value.regbits.tprescaler_hi << 8) |
            (uint16_t )dev->map.tprescaler_low.value.regbyte);
    return err;
}


status_t mfrc_get_fifo_waterlevel(MFRC_DEV dev, uint8_t *wl) {
    status_t err = STATUS_OK;

    *wl = dev->map.fifo_watermark_reg.regBits.fifo_watermark;

    return err;
}

status_t mfrc_set_fifo_waterlevel(MFRC_DEV dev, uint8_t *wl) {
    status_t err = STATUS_OK;
    uint8_t mark = *wl;
    if(mark > MFRC_FIFO_MAX_WL) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_FIFO_WATERLEVEL, MFRC_FIFO_MAX_WL, mark, &dev->map.fifo_watermark_reg.regByte);
    }

    return err;
}



status_t mfrc_get_modwidth(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.modwidth_reg.value.regbyte;
    return err;
}


status_t mfrc_set_modwidth(MFRC_DEV dev, uint8_t *var) {

    status_t err = STATUS_OK;
    uint8_t v = *var;

    err = mfrc_read_mod_write_register(dev, MRFC_REGADDR_MOD_WIDTH, 0xFF, v, &dev->map.modwidth_reg.value.regbyte);
    return err;
}


status_t mfrc_set_rx_gain(MFRC_DEV dev, mfrc_rx_gain_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 8) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 4);
        err = mfrc_read_mod_write_register(
                dev,
                 MRFC_REGADDR_RFC_GAIN,
                 (0b111 << 4), 
                 v, 
                 &dev->map.rfccfg_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_rx_gain(MFRC_DEV dev, mfrc_rx_gain_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.rfccfg_reg.value.regbits.rx_gain;
    return err;
}


status_t mfrc_set_mod_gsn(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 15) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        err = mfrc_read_mod_write_register(
                dev,
                 MRFC_REGADDR_GS_N,
                 0x0f, 
                 v, 
                 &dev->map.GsN_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_mod_gsn(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.GsN_reg.value.regbits.mod_gsn;
    return err;
}


status_t mfrc_set_cw_gsn(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 16) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 4);
        err = mfrc_read_mod_write_register(
                dev,
                MRFC_REGADDR_GS_N,
                0xf0, 
                v, 
                &dev->map.GsN_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_cw_gsn(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.GsN_reg.value.regbits.cw_gsn;
    return err;
}


status_t mfrc_set_cwg_sp(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 63) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        err = mfrc_read_mod_write_register(
                dev,
                MRFC_REGADDR_CWG_SP,
                0x3F, 
                v, 
                &dev->map.GsN_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_cwg_sp(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.cwg_sp_reg.value.regbits.cwg_sp;
    return err;
}

status_t mfrc_set_mod_gsp(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 64) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        err = mfrc_read_mod_write_register(
                dev,
                MRFC_REGADDR_MODG_SP,
                0x3F, 
                v, 
                &dev->map.GsN_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_mod_gsp(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.mod_gsp_reg.value.regbyte;
    return err;
}


status_t mfrc_set_tmr_gated(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 4) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 5);
        err = mfrc_read_mod_write_register(
                dev,
                MRFC_REGADDR_TMR_MODE,
                (0b11 << 5), 
                v, 
                &dev->map.GsN_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_tmr_gated(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.tmode_reg.value.regbits.tmr_gated;
    return err;
}



status_t mfrc_set_tmr_autorestart(MFRC_DEV dev, bool *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 2) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 4);
        err = mfrc_read_mod_write_register(
                dev,
                MRFC_REGADDR_TMR_MODE,
                (1 << 4), 
                v, 
                &dev->map.tmode_reg.value.regbyte
            );
    }
    return err;
}



status_t mfrc_set_tmr_reload(MFRC_DEV dev, uint16_t *var) {
    status_t err = STATUS_OK;
    uint16_t v = *var;
    
    err = mfrc_read_mod_write_register(
            dev,
            MFRC_REGADDR_TCNTR_RELOAD_MSB,
            0, 
            ((uint8_t )(v & 0xff00) >> 8), 
            &dev->map.tmr_ctr_reg.value.regbyte
        );

    if (!err) {
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TCNTR_RELOAD_LSB,
                0, 
                ((uint8_t )(v & 0xff)), 
                &dev->map.tmr_ctr_lsb_reg.value.regbyte
            );
    }

    return err;
}


status_t mfrc_get_tmr_reload(MFRC_DEV dev, uint16_t *var) {
    status_t err = STATUS_OK;
    *var = (((uint16_t )dev->map.tmr_reload_reg.value.regbyte << 8) 
            |((uint16_t )dev->map.tmr_reload_lsb_reg.value.regbyte));
    return err;
}



status_t mfrc_get_tmr_ctr(MFRC_DEV dev, uint16_t *var) {
    status_t err = STATUS_OK;
    uint8_t vals[2] = {0};
    err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_TCNTR_MSB, &vals[1]);
    if(!err) {
        err = mfrc_read_byte_from_address(dev, MRFC_REGADDR_TCNTR_LSB, vals);        
    }
    *var = (((uint16_t )vals[0] << 8) | (uint16_t )vals[1]);
    return err;
}



status_t mfrc_set_tstbus_sel(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 32) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_SEL_1,
                0, 
                v, 
                &dev->map.test_sel1_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_tstbus_sel(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.test_sel1_reg.value.regbits.tstbus_sel;
    return err;
}


status_t mfrc_set_prbs15(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 1) {
        v = 1;
    }
    else {
        /** user code here **/
        v = (v << 5);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_SEL_2,
                (1 << 5), 
                v, 
                &dev->map.tst_sel2_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_prbs15(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.tst_sel2_reg.value.regbits.prbs15;
    return err;
}


status_t mfrc_set_prbs9(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 1) {
        v = 1;
    }
    else {
        /** user code here **/
        v = (v << 6);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_SEL_2,
                (1 << 6), 
                v, 
                &dev->map.tst_sel2_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_prbs9(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.tst_sel2_reg.value.regbits.prbs9;
    return err;
}


status_t mfrc_set_tstbus_flip(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 1) {
        v = 1;
    }
    else {
        /** user code here **/
        v = (v << 7);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_SEL_2,
                (1 << 7), 
                v, 
                &dev->map.tst_sel2_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_tstbus_flip(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.tst_sel2_reg.value.regbits.tstbus_flip;
    return err;
}


status_t mfrc_set_testpin_en(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 64) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 1);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_PIN_EN,
                (0x3f << 1), 
                v, 
                &dev->map.testpin_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_testpin_en(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.testpin_reg.value.regbits.testpin_en;
    return err;
}


status_t mfrc_set_rs232line_en(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 1) {
        v = 1;
    }
    else {
        /** user code here **/
        v = (v << 7);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_PIN_EN,
                (1 << 7), 
                v, 
                &dev->map.testpin_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_rs232line_en(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.testpin_reg.value.regbits.rs232line_en;
    return err;
}


status_t mfrc_set_testpin_value(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 63) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 1);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_PIN_VAL,
                (0x3f << 1), 
                v, 
                &dev->map.testpin_value_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_testpin_value(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.testpin_value_reg.value.regbits.testpin_value;
    return err;
}


status_t mfrc_set_use_io(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = *var;

    if(v > 2) {
        err = STATUS_ERR_INVALID_ARG;
    }
    else {
        /** user code here **/
        v = (v << 7);
        err = mfrc_read_mod_write_register(
                dev,
                MFRC_REGADDR_TEST_PIN_VAL,
                (1 << 7), 
                v, 
                &dev->map.testpin_value_reg.value.regbyte
            );
    }
    return err;
}


status_t mfrc_get_use_io(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    *var = dev->map.testpin_value_reg.value.regbits.use_io;
    return err;
}


status_t mfrc_get_testbus(MFRC_DEV dev, uint8_t *var) {
    status_t err = STATUS_OK;
    uint8_t v = 0;

    err = mfrc_read_byte_from_address(dev, MFRC_REGADDR_TEST_PIN_VAL, &v);
    if(!err) {
        *var = v;
    }

    return err;
}




#ifdef DEBUG_MODE

status_t print_register(MFRC_DEV dev, uint8_t address) {
    uint8_t reg = 0;
    status_t err = mfrc_read_byte_from_address(dev, address, &reg);

    if(!err) {
        log_info(MFRC_TAG, "Read %02x from address %02x", reg, address);
        printBytesOrderExplicit(reg);
    }

    return err;
}

#endif 