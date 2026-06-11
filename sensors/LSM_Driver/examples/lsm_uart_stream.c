/******************************************************/ /**
                                                          *  @file main.c
                                                          *
                                                          *  @brief main application file for
                                                          *ESPHome
                                                          *
                                                          *********************************************************/

#include "LSM_Driver.h"
#include "PeripheralManager.h"
#include "Utilities.h"
#include "WifiDriver.h"
#include "port/interfaces/uart.h"
#include "port/log.h"
#include "port/interfaces/spi.h"
#include "port/system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "GenericCommsDriver.h"
#include "main.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/************************************************
 * Private Functions
 ************************************************/

status_t print_message(CBuff buff)
{
    struct data_pkt {
        float accel[3];
        float gyro[3];
        uint8_t sep_bytes[2];
    } dp;

    uint32_t avail = 0;
    if (cbuffer_get_available_packets(buff, &avail) != STATUS_OK) {
        return 0;
    }

    log_info("=====================\n\n");
    cbuffer_read_packet(buff, &dp);

    uint16_t length = 0;
    cbuffer_get_full_packet_length(buff, &length);
    log_info("Got a sample packet - %u bytes long", length);

    log_info(
        "%.2f %.2f %.2f \t%.2f %.2f %.2f \t%02x %02x\n",
        dp.accel[0],
        dp.accel[1],
        dp.accel[2],
        dp.gyro[0],
        dp.gyro[1],
        dp.gyro[2],
        dp.sep_bytes[0],
        dp.sep_bytes[1]);

    log_info("\n");

    return STATUS_OK;
}

void app_main(void)
{
    // Initialize NVS
    status_t ret = nvs_flash_init();
    if (ret == STATUS_ERR_NVS_NO_FREE_PAGES || ret == STATUS_ERR_NVS_NEW_VERSION_FOUND) {
        STATUS_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    STATUS_ERROR_CHECK(ret);

#ifdef CONFIG_USE_PERIPH_MANAGER
    log_info("MAIN", "Config use periph manager is enabled");

    pm_init_t pm_init = {0};
    pm_init.spi_channel = VSPI_HOST;
    pm_init.init_gpio_isr = true;
    pm_init.init_i2c = true;
    pm_init.init_spi = false;
    pm_init.mosi = 12;
    pm_init.miso = -1;
    pm_init.sck = 13;
    pm_init.i2c_speed = 200000;
    pm_init.i2c_channel = 1;
    pm_init.scl = 5;
    pm_init.sda = 18;
    pm_init.init_uart = true;
    pm_init.uart_channel = UART_NUM_1;
    pm_init.uart_rx = 25;
    pm_init.uart_tx = 26;
    pm_init.uart_cts = -1;
    pm_init.uart_rts = -1;
    pm_init.uart_baud = 115200;

    ret = peripheral_manager_init(&pm_init);

    if (ret != STATUS_OK) {
        log_error("MAIN", "Error starting PM");
    }

    event_loop_handle_t loop = pm_get_event_loop();

#endif

    CBuffer_init_t cbuff_init = {
        .allow_ovr = true,
        .malloc_caps = 0,
        .size = 4096,
    };

    CBuff data_buffer = cbuffer_create(&cbuff_init);
    if (data_buffer == NULL) {
        while (1) {
            log_error("MAIN", "Done fucked up init...");
            vTaskDelay(10000);
        }
    } else {
        uint8_t sep[2] = {0x41, 0x41};
        char *names[6] = {"gyrox", "gyroy", "gyroz", "accelx", "accely", "accelz"};
        uint8_t ids[6] = {1, 2, 3, 4, 5, 6};

        status_t s = cbuffer_load_packet(data_buffer, 6, "hhhhhh", names, ids);

        if (s) {
            log_info("error loading pkt %u\n", s);
        }

        s = cbuffer_config_packet(data_buffer, true, sep, 2, false, CBUFF_TS_PRECISION_SECOND);

        if (s) {
            log_info("error config pkt %u\n", s);
        }

        s = cbuffer_start_task(
            data_buffer,
            CBUFF_IO_TYPE_UART,
            1,
            0,
            CBUFF_TASK_PACKET_DISPATCH_CONTINUOUS,
            0);
        if (s) {
            log_info("Error starting task %u\n", s);
        }
    }

    event_map_init_t evt = {
        .act = print_message,
        .args = NULL,
        .event_id = CBUFF_EVENTCODE_NEARFULL,
        .handle = data_buffer,
        .get = NULL,
        .set = NULL,
    };

    add_event_map(&evt);

    LSM_initData_t init = {
        .accelRate = LSM_ACCODR_104_HZ,
        .addrPinState = 0,
        .assignFifoBuffer = false,
        .cbuff = data_buffer,
        .cbuff_store_packets = true,
        .cbuff_store_raw = true,
        .commMode = LSM_DEVICE_COMM_MODE_I2C,
        .commsChannel = 1,
        .gyroRate = LSM_ACCODR_104_HZ,
        .opMode = LSM_OPMODE_GYRO_ACCEL,
        .int1Pin = 17,
        .int2Pin = 0,
        .use_cbuffer = true,
    };

    LSM_DriverHandle_t lsm_device;

    LSMDEV lsm = &lsm_device;

    if (LSM_init(lsm, &init) != STATUS_OK) {
        while (1) {
            log_error("MAIN", "Error starting LSM");
            vTaskDelay(100);
        }
    }

    // vTaskDelay(pdMS_TO_TICKS(5));

    /** set the interrupt settings **/
    uint8_t val = (LSM_INT1_TYPE_FIFO_THR);
    bool en = 1;

    // // LSM_reboot_memory(lsm);
    // vTaskDelay(10);

    status_t lsm_err = LSM_set_interrupt_1(lsm, &val);
    if (lsm_err) {
        log_error("MAIN", "Error 1 {%u}", lsm_err);
    }

    /** try setting the interrupt pin to open drain **/
    lsm_err = LSM_set_interrupt_pp_od(lsm, &en);

    uint16_t val2 = 1000; /** 100 * 2-byte measurements **/
    lsm_err = LSM_set_fifo_watermark(lsm, &val2);
    if (lsm_err) {
        log_error("MAIN", "Error 3 {%u}", lsm_err);
    }

    uint16_t pkts = 0;
    uint32_t buffpkts = 0;

    val = 0;
    LSM_set_fifo_mode(lsm, &val);

    LSM_get_fifo_pkt_count(lsm, &pkts);

    /** set the packet types **/
    val = (LSM_PKT1_GYRO | LSM_PKT2_ACCL);
    lsm_err = LSM_setFIFOpackets(lsm, &val);
    if (lsm_err) {
        log_error("MAIN", "Error 4 {%u}", lsm_err);
    }

    LSM_set_stop_on_fifo_thresh(lsm, &en);
    /** set fifo odr to LSM_FIFO_ODR_833_HZ **/
    val = LSM_FIFO_ODR_833_HZ;
    lsm_err = LSM_set_fifo_odr(lsm, &val);
    if (lsm_err) {
        log_error("MAIN", "Error 5 {%u}", lsm_err);
    }

    // /** set fifo mode **/
    val = LSM_FIFO_MODE_FIFO;
    lsm_err = LSM_set_fifo_mode(lsm, &val);
    if (lsm_err) {
        log_error("MAIN", "Error 2 {%u}", lsm_err);
    }

    // dump_info(lsm);

    pm_add_new_peripheral(&lsm_periph_template, 0x58, lsm);

    // float gx, gy, gz, ax, ay, az;
    uint16_t bytes_avail = 0;

    while (1) {
        // LSM_sample_latest(lsm);

        // LSM_getGyroX(lsm, &gx);
        // LSM_getGyroY(lsm, &gy);
        // LSM_getGyroZ(lsm, &gz);

        // LSM_getAccelX(lsm, &ax);
        // LSM_getAccelY(lsm, &ay);
        // LSM_getAccelZ(lsm, &az);

        // log_info("GX: %.3f GY: %.3f GZ: %.3f \t AX: %.3f AY: %.3f AZ: %.3f\n", gx, gy, gz, ax, ay,
        // az);

        vTaskDelay(pdMS_TO_TICKS(100));

        LSM_get_fifo_pkt_count(lsm, &pkts);
        cbuffer_get_available_packets(data_buffer, &buffpkts);
        log_info("Fifo pkt count: %u Cbuffer Packets: %u\n", pkts, buffpkts);

        if (pkts > 990) {
            LSM_read_fifo_to_cbuffer(lsm);
        }
    }
    /* here be dragons */
}
