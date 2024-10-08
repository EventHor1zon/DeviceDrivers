/***************************************
* \file     BME280_Driver.c
* \brief    A FreeRTOS/ESP-IDF driver for the BME280 bosch humidity/temp/pressure 
*           sensor. Can be called in periodic or on-demand sensing, and in 
*           a number of different sampling modes
*
* \date     July 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "port/types.h"
#include "port/error_type.h"
#include "port/log.h"
#include "Utilities.h"
#include "GenericCommsDriver.h"
#include "BME280_Driver.h"


/****** Function Prototypes ***********/
static int32_t bm280_compensate_T_int32(BM_DEV dev);
static uint32_t bm280_compensate_P_int64(BM_DEV dev);
#ifdef BME_280
static uint32_t bm280_compensate_H_int32(BM_DEV dev);
#endif
static status_t bm280_getDeviceStatus(BM_DEV dev);
/************ ISR *********************/

/****** Global Data *******************/

#ifdef CONFIG_USE_PERIPH_MANAGER
const parameter_t bm_param_map[bm_param_len] = {
    {"Sample Interval", 1, &bm280_getSampleInterval, &bm280_setSampleInterval, NULL, DATATYPE_UINT8, 7, (GET_FLAG | SET_FLAG)},
    {"Filter Setting", 2, &bm280_getFilterSetting, &bm280_setFilterSetting, NULL, DATATYPE_UINT8, 4, (GET_FLAG | SET_FLAG)},
    {"Device ID", 3, &bm280_getDeviceID, NULL, NULL, DATATYPE_UINT8, 0, (GET_FLAG)},
    {"Temperature", 4, &bm280_getTemperature, NULL, NULL, DATATYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG)},
    {"Pressure", 5, &bm280_getPressure, NULL, NULL, DATATYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG)},
    {"Humidity", 6, &bm280_getHumidity, NULL, NULL, DATATYPE_FLOAT, 0, (GET_FLAG | STREAM_FLAG)},
    {"Temperature OSample", 7, &bm280_getTemperatureOS, &bm280_setTemperatureOS, NULL, DATATYPE_UINT8, 5, (GET_FLAG | SET_FLAG)},
    {"Pressure OSample", 8, &bm280_getPressureOS, &bm280_setPressureOS, NULL, DATATYPE_UINT8, 5, (GET_FLAG | SET_FLAG)},
    {"Humidity OSample", 9, &bm280_getHumidityOS, &bm280_setHumidityOS, NULL, DATATYPE_UINT8, 5, (GET_FLAG | SET_FLAG)},
    {"Sample Mode", 10, &bm280_getSampleMode, &bm280_setSampleMode, NULL, DATATYPE_UINT8, 3, (GET_FLAG | SET_FLAG)},
    {"Sample Type", 11, &bm280_getSampleType, NULL, NULL, DATATYPE_UINT8, 7, (GET_FLAG)},
    {"Update Measurements", 12, NULL, NULL, &bm280_updateMeasurements, DATATYPE_NONE, 0, (ACT_FLAG)},
};


const peripheral_t bme_peripheral_template = {
    .handle = NULL,
    .param_len = bm_param_len,
    .params = bm_param_map,
    .peripheral_name = "BME280",
    .peripheral_id = 0
};

#endif

const int wait_sample_fin = 10;
const int wait_new_sample = 50;
const int wait_idle = 1000;
const char *BM_DRIVER_TAG = "[BM280 DRIVER]";



/****** Private Functions *************/

/** Calibration Functions :  The following calibration fucntions are adapted from the Bosch BME280 Data sheet **/

/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. */
/* dev->calibrationData.t_fine carries fine temperature as global value int32_t dev->calibrationData.t_fine; */
static int32_t bm280_compensate_T_int32(BM_DEV dev)
{
    if (dev->calibrationAquired)
    {
        int32_t adc_T = dev->sensorData.rawTemperature;
        int32_t var1, var2, T;

        var1 = ((((adc_T >> 3) - ((int32_t)dev->calibrationData.dig_T1 << 1))) * ((int32_t)dev->calibrationData.dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)dev->calibrationData.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calibrationData.dig_T1))) >> 12) * ((int32_t)dev->calibrationData.dig_T3)) >> 14;
        dev->calibrationData.t_fine = var1 + var2;
        T = (dev->calibrationData.t_fine * 5 + 128) >> 8;
        return T;
    }
    else
    {
        log_error(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}

/* Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).*/
/* Output value of “24674867” represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa */
static uint32_t bm280_compensate_P_int64(BM_DEV dev)
{
    if (dev->calibrationAquired)
    {
        int32_t adc_P = dev->sensorData.rawPressure;
        int64_t var1, var2, p;
        var1 = ((int64_t)dev->calibrationData.t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)dev->calibrationData.dig_P6;
        var2 = var2 + ((var1 * (int64_t)dev->calibrationData.dig_P5) << 17);
        var2 = var2 + (((int64_t)dev->calibrationData.dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)dev->calibrationData.dig_P3) >> 8) + ((var1 * (int64_t)dev->calibrationData.dig_P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calibrationData.dig_P1) >> 33;
        if (var1 == 0)
        {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)dev->calibrationData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)dev->calibrationData.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calibrationData.dig_P7) << 4);
        return (uint32_t)p;
    }
    else
    {
        log_error(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}


#ifdef BME_280

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t bm280_compensate_H_int32(BM_DEV dev)
{
    if (dev->calibrationAquired)
    {
        int32_t adc_H = dev->sensorData.rawHumidity;
        int32_t v_x1_u32r;
        v_x1_u32r = (dev->calibrationData.t_fine - ((int32_t)76800));
        v_x1_u32r = (((((adc_H << 14) - (((int32_t)dev->calibrationData.dig_H4) << 20) - (((int32_t)dev->calibrationData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dev->calibrationData.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dev->calibrationData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dev->calibrationData.dig_H2) + 8192) >> 14));
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->calibrationData.dig_H1)) >> 4));
        v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
        v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
        return (uint32_t)(v_x1_u32r >> 12);
    }
    else
    {
        log_error(BM_DRIVER_TAG, "Error Calibration data not aquired yet!");
        return 0;
    }
}
#endif

static uint16_t bm280_sleepTime(BM_DEV dev)
{
    uint8_t timeSetting = dev->devSettings.sampleInterval;
    uint16_t time;
    switch (timeSetting)
    {
    case BM_T_STDBY_0_5MS:
        time = 1; /** this one is trick? divide by 2, I guess **/
        break;
    case BM_T_STDBY_62_5MS:
        time = 63;
        break;
    case BM_T_STDBY_125MS:
        time = 125;
        break;
    case BM_T_STDBY_250MS:
        time = 250;
        break;
    case BM_T_STDBY_500MS:
        time = 500;
        break;
    case BM_T_STDBY_1000MS:
        time = 1000;
        break;
#ifdef BME_280
    case BM_T_STDBY_10MS:
        time = 10;
        break;
    case BM_T_STDBY_20MS:
        time = 20;
        break;
#else
    case BM_T_STDBY_2000MS:
        time = 2000;
        break;
    case BM_T_STDBY_4000MS:
        time = 4000;
        break;
#endif
    default:
        time = 1000;
        break;
    }

    return time;
}


static status_t bm280_getDeviceStatus(BM_DEV dev)
{
    status_t trxStatus = STATUS_OK;

    uint8_t statusBuffer = 0;
    trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &statusBuffer);
    if (trxStatus == STATUS_OK)
    {
        dev->sensorData.statusMeasure = (statusBuffer & BM_STATUS_MEASURE_MASK) ? 1 : 0;
        dev->sensorData.statusUpdate = (statusBuffer & BM_STATUS_UPDATE_MASK) ? 1 : 0;
    }

    return trxStatus;
}


static status_t bm280_getCalibrationData(BM_DEV dev)
{
    status_t trxStatus = STATUS_OK;

#ifdef BME_280
    uint8_t buffer[BM_CALIBR_DATA_BANK1_LEN] = {0};
#else
    uint8_t buffer[BM_CALIBR_DATA_LEN] = {0};
#endif

    /* check the status register for calibration load complete */
    uint8_t statusReg = 0;
    gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DEV_STATUS, 1, &statusReg);

    while (statusReg & BM_STATUS_UPDATE_MASK)
    {
        vTaskDelay(10);
        gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DEV_STATUS, 1, &statusReg);
    }

#ifdef BME_280
    /** retrieve the calibration data - add +1 to length of bank1 as there's an unused byte (A0, index[24]) in there **/
    uint8_t bufferB[BM_CALIBR_DATA_BANK2_LEN] = {0};
    trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_BANK1_LEN, buffer);
    trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DIGH2_LSB, BM_CALIBR_DATA_BANK2_LEN, bufferB);
#else
    trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DIGT1_LSB, BM_CALIBR_DATA_LEN, buffer);
#endif

    if (trxStatus == STATUS_OK)
    {
        dev->calibrationData.dig_T1 = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];
        dev->calibrationData.dig_T2 = (int16_t)buffer[3] << 8 | (int16_t)buffer[2];
        dev->calibrationData.dig_T3 = (int16_t)buffer[5] << 8 | (int16_t)buffer[4];
        dev->calibrationData.dig_P1 = (uint16_t)buffer[7] << 8 | (uint16_t)buffer[6];
        dev->calibrationData.dig_P2 = (int16_t)buffer[9] << 8 | (int16_t)buffer[8];
        dev->calibrationData.dig_P3 = (int16_t)buffer[11] << 8 | (int16_t)buffer[10];
        dev->calibrationData.dig_P4 = (int16_t)buffer[13] << 8 | (int16_t)buffer[12];
        dev->calibrationData.dig_P5 = (int16_t)buffer[15] << 8 | (int16_t)buffer[14];
        dev->calibrationData.dig_P6 = (int16_t)buffer[17] << 8 | (int16_t)buffer[16];
        dev->calibrationData.dig_P7 = (int16_t)buffer[19] << 8 | (int16_t)buffer[18];
        dev->calibrationData.dig_P8 = (int16_t)buffer[21] << 8 | (int16_t)buffer[20];
        dev->calibrationData.dig_P9 = (int16_t)buffer[23] << 8 | (int16_t)buffer[22];

#ifdef BME_280
        int16_t dig_H4_lsb = 0, dig_H4_msb = 0, dig_H5_lsb = 0, dig_H5_msb = 0;
        dev->calibrationData.dig_H1 = buffer[25];
        dev->calibrationData.dig_H2 = (int16_t)bufferB[1] << 8 | (int16_t)bufferB[0];
        dev->calibrationData.dig_H3 = bufferB[2];
        dig_H4_msb = (int16_t)(int8_t)bufferB[3] * 16;
        dig_H4_lsb = (int16_t)(bufferB[4] & 0x0F);
        dev->calibrationData.dig_H4 = dig_H4_msb | dig_H4_lsb;
        dig_H5_msb = (int16_t)(int8_t)bufferB[5] * 16;
        dig_H5_lsb = (int16_t)(bufferB[4] >> 4);
        dev->calibrationData.dig_H5 = dig_H5_msb | dig_H5_lsb;
        dev->calibrationData.dig_H6 = (int8_t)bufferB[6];
#endif
        dev->calibrationAquired = true;
    }
    else
    {
        log_info(BM_DRIVER_TAG, "Error reading calibration data");
    }

    return trxStatus;
}


static status_t bm280_InitDeviceSettings(BM_DEV dev)
{

    status_t trxStatus = STATUS_OK;

    uint8_t commands[BM_CONFIG_WRITE_LEN] = {0};

    /* put the device in sleep mode */
    gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, commands);

    switch (dev->devSettings.sampleMode)
    {
    case BM_SAMPLE_OFF:
        break;
    case BM_FORCE_MODE:
        commands[0] = (uint8_t)BM_FORCE_MODE;
        break;
    case BM_NORMAL_MODE:
        commands[0] = (uint8_t)BM_NORMAL_MODE;
        commands[1] |= BM_DEFAULT_T_STDBY; /** t_standby = 0.5ms **/
        break;
    default:
        log_error(BM_DRIVER_TAG, "Error - incorrect sample mode type selected");
        break;
    }

    if(dev->use_cbuffer) {
        char pattern[3] = "fff";
        char *names[3];
        char *n[3] = {"temperature", "pressure", "humidity"};
        uint8_t ids[3] = {0};
        uint8_t sz = 0;

        switch(dev->devSettings.sampleType) {
            case BM_MODE_TEMP:
                sz = 1;
                names[0] = n[0];
                ids[0] = BM_TEMP_MEASURE_ID;
                break;
            case BM_MODE_TEMP_PRESSURE:
                sz = 2;
                ids[0] = BM_TEMP_MEASURE_ID;
                ids[1] = BM_PRESSURE_MEASURE_ID;
                names[0] = n[0];
                names[1] = n[1];
                break;
#ifdef BME_280
            case BM_MODE_TEMP_HUMIDITY:
                sz = 2;
                names[0] = n[0];
                names[1] = n[2];
                ids[0] = BM_TEMP_MEASURE_ID;
                ids[1] = BM_HUMIDITY_MEASURE_ID;
                break;
            case BM_MODE_TEMP_PRESSURE_HUMIDITY:
                sz = 3;
                names[0] = n[0];
                names[1] = n[1];
                names[2] = n[2];
                ids[0] = BM_TEMP_MEASURE_ID;
                ids[1] = BM_PRESSURE_MEASURE_ID;
                ids[2] = BM_HUMIDITY_MEASURE_ID;
                break;
#endif
            default:
                break;
        }

        cbuffer_clear_packet(dev->cbuff);
        trxStatus = cbuffer_load_packet(dev->cbuff, sz, pattern, names, ids);
        if(trxStatus) {
            log_error(BM_DRIVER_TAG, "Error in pattern loading: %u", trxStatus);
            trxStatus = 0;
        }
        else {
            log_info(BM_DRIVER_TAG, "Loaded pattern succesfully");
        }
    }

    switch (dev->devSettings.sampleType)
    {
    case BM_MODE_TEMP:
        commands[0] |= BM_CTRL_TEMP_BIT;
        break;
    case BM_MODE_TEMP_PRESSURE:
        commands[0] |= (BM_CTRL_TEMP_BIT | BM_CTRL_PRESSURE_BIT);
        break;
#ifdef BME_280
    case BM_MODE_TEMP_HUMIDITY:
        commands[2] = 1;
        commands[0] |= BM_CTRL_TEMP_BIT;
        break;
    case BM_MODE_TEMP_PRESSURE_HUMIDITY:
        commands[2] = 1;
        commands[0] |= (BM_CTRL_TEMP_BIT | BM_CTRL_PRESSURE_BIT);
        break;
#endif
    default:
        break;
    }

#ifdef BME_280
    /** have to do 2 writes to 0xF2 and 0xF4-5, because 0xF3 is read only :/ **/
    trxStatus = gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CTRL_HUMID, 1, &commands[2]);
#endif
    trxStatus = gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CONFIG, 1, &commands[1]);
    trxStatus = gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, commands);
    if (trxStatus != STATUS_OK)
    {
        log_error(BM_DRIVER_TAG, "Error in setting control registers");
    }
    else
    {
        dev->sampleMask = commands[0];
        dev->configMask = commands[1];
    }

    return trxStatus;
}

/****** Global Functions *************/

status_t bm280_getDeviceID(BM_DEV dev, uint8_t *deviceID)
{

    status_t trxStatus = STATUS_OK;

    uint8_t devID = 0;
    trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, (uint8_t)BM_REG_ADDR_DEVICEID, 1, &devID);

    if (trxStatus == STATUS_OK)
    {
        *deviceID = devID;
    }

    return trxStatus;
}


status_t bm280_updateMeasurements(BM_DEV dev)
{

    status_t trxStatus = STATUS_OK;
    uint8_t forcedMeasure = dev->sampleMask | BM_CTRL_MODE_FORCED;
    uint8_t rxBuffer[BM_MEASURE_READ_LEN] = {0};
    uint8_t reg = 0;
    uint8_t retries = 10;

    float temp[4] = {0};

    if (dev->devSettings.sampleMode == BM_FORCE_MODE)
    {
#ifdef DEBUG
        log_info(BM_DRIVER_TAG, "Telling device to sample...");
#endif
        trxStatus = gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CTRL_MEASURE, 1, &forcedMeasure);
        if (trxStatus != STATUS_OK)
        {
            log_error(BM_DRIVER_TAG, "Error in writing to mode");
        }
        vTaskDelay(pdMS_TO_TICKS(wait_new_sample));

        /** check sample finished **/
        trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg);

        while (reg & BM_STATUS_MEASURE_MASK && trxStatus == STATUS_OK && retries > 0) {
            trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg);
            vTaskDelay(pdMS_TO_TICKS(wait_sample_fin));
            retries--;
        }
    }

    if (trxStatus == STATUS_OK)
    {   
        /** Read the fifo **/
        trxStatus = gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_PRESSURE_MSB, (uint16_t)BM_MEASURE_READ_LEN, rxBuffer);
    }

    if (trxStatus == STATUS_OK)
    {
        int32_t regP = (uint32_t)rxBuffer[0] << 16 | (uint32_t)rxBuffer[1] << 8 | (uint32_t)rxBuffer[2];
        int32_t regT = (uint32_t)rxBuffer[3] << 16 | (uint32_t)rxBuffer[4] << 8 | (uint32_t)rxBuffer[5];
        dev->sensorData.rawPressure = regP >> 4;
        dev->sensorData.rawTemperature = regT >> 4;
#ifdef BME_280
        dev->sensorData.rawHumidity = (uint32_t)rxBuffer[6] << 8 | (uint32_t)rxBuffer[7];
#endif

#ifdef DEBUG
        if (regT == 0x80000)
        {
            log_info(BM_DRIVER_TAG, "Info: Temp is disabled");
        }
        if (regP == 0x80000)
        {
            log_info(BM_DRIVER_TAG, "Info: Pressure is disabled");
        }
        if (dev->sensorData.rawHumidity == 0x8000)
        {
            log_info(BM_DRIVER_TAG, "Info: Humid is disabled");
        }
#endif
        switch (dev->devSettings.sampleType)
        {
        case BM_MODE_TEMP:
            dev->sensorData.calibratedTemperature = bm280_compensate_T_int32(dev);
            dev->sensorData.realTemperature = (float)dev->sensorData.calibratedTemperature / 100.0;
            if (dev->use_cbuffer) {
                temp[0] = dev->sensorData.realTemperature;
                cbuffer_write_packet(dev->cbuff, temp, sizeof(float));
            }
            break;
        case BM_MODE_TEMP_PRESSURE:
            dev->sensorData.calibratedTemperature = bm280_compensate_T_int32(dev);
            dev->sensorData.calibratedPressure = bm280_compensate_P_int64(dev);
            dev->sensorData.realTemperature = (float)dev->sensorData.calibratedTemperature / 100.0;
            dev->sensorData.realPressure = (float)dev->sensorData.calibratedPressure / 256;
            if (dev->use_cbuffer) {
                temp[0] = dev->sensorData.realTemperature;
                temp[1] = dev->sensorData.realPressure;
                cbuffer_write_packet(dev->cbuff, temp, sizeof(float) * 2);
            }
            break;
#ifdef BME_280
        case BM_MODE_TEMP_HUMIDITY:
            dev->sensorData.calibratedTemperature = bm280_compensate_T_int32(dev);
            dev->sensorData.realTemperature = (float)dev->sensorData.calibratedTemperature / 100.0;
            dev->sensorData.calibratedHumidity = bm280_compensate_H_int32(dev);
            dev->sensorData.realHumidity = (float)dev->sensorData.calibratedHumidity / 1024;
            if (dev->use_cbuffer) {
                temp[0] = dev->sensorData.realTemperature;
                temp[1] = dev->sensorData.realHumidity;
                cbuffer_write_packet(dev->cbuff, temp, sizeof(float) * 2);
            }
            break;
        case BM_MODE_TEMP_PRESSURE_HUMIDITY:
            dev->sensorData.calibratedTemperature = bm280_compensate_T_int32(dev);
            dev->sensorData.realTemperature = (float)dev->sensorData.calibratedTemperature / 100.0;
            dev->sensorData.calibratedPressure = bm280_compensate_P_int64(dev);
            dev->sensorData.realPressure = (float)dev->sensorData.calibratedPressure / 256;
            dev->sensorData.calibratedHumidity = bm280_compensate_H_int32(dev);
            dev->sensorData.realHumidity = (float)dev->sensorData.calibratedHumidity / 1024;
            if (dev->use_cbuffer) {
                temp[0] = dev->sensorData.realTemperature;
                temp[1] = dev->sensorData.realPressure;
                temp[2] = dev->sensorData.realHumidity;
                cbuffer_write_packet(dev->cbuff, temp, sizeof(float) * 3);
            }
            break;
#endif
        default:
            break;
        }
    }
    else
    {
        log_error(BM_DRIVER_TAG, "Error in reading data [%u]", trxStatus);
    }

    return trxStatus;
}


status_t bm280_getTemperature(BM_DEV dev, float *realTemp)
{
    status_t status = STATUS_OK;

    *realTemp = dev->sensorData.realTemperature;

    return status;
}


status_t bm280_getPressure(BM_DEV dev, float *realPressure)
{
    status_t status = STATUS_OK;

    *realPressure = dev->sensorData.realPressure;

    return status;
}


#ifdef BME_280
status_t bm280_getHumidity(BM_DEV dev, float *realHumidity)
{
    status_t status = STATUS_OK;

    *realHumidity = dev->sensorData.realHumidity;

    return status;
}


status_t bm280_getHumidityOS(BM_DEV dev, uint8_t *humidOS)
{
    status_t status = STATUS_OK;
    *humidOS = dev->devSettings.humidOS;
    return status;
}


status_t bm280_setHumidityOS(BM_DEV dev, BM_overSample_t *os)
{
    status_t status = STATUS_OK;
    uint8_t _os = *os;

    if(_os > BM_OS_16) {
        status = STATUS_ERR_INVALID_ARG;
    }

    if (status == STATUS_OK && 
        gcd_i2c_read_mod_write(dev->i2cChannel, 
                               dev->deviceAddress, 
                               BM_REG_ADDR_CTRL_HUMID, 
                               _os, 
                               0b111) == STATUS_OK)
    {
        dev->devSettings.humidOS = *os;
        if (_os > 0) {
            BYTE_SET_BITS(dev->devSettings.sampleType, BM_MODE_HUMIDITY);
        }
        else {
            BYTE_UNSET_BITS(dev->devSettings.sampleType, BM_MODE_HUMIDITY);
        }
    }
    
    return status;
}

#endif /** BME_280 **/

status_t bm280_getTemperatureOS(BM_DEV dev, uint8_t *tempOS)
{
    status_t status = STATUS_OK;
    *tempOS = dev->devSettings.humidOS;
    return status;
}


status_t bm280_setTemperatureOS(BM_DEV dev, BM_overSample_t *os)
{
    status_t status = STATUS_OK;
    uint8_t _os = *os;

    if(_os > BM_OS_16) {
        status = STATUS_ERR_INVALID_ARG;
    }

    if (status == STATUS_OK &&
        gcd_i2c_read_mod_write(dev->i2cChannel, 
                               dev->deviceAddress, 
                               BM_REG_ADDR_CTRL_MEASURE, 
                               (_os << 5), 
                               0b11100000) == STATUS_OK)
    {
        dev->devSettings.tempOS = _os;

        if (_os > 0) {
            BYTE_SET_BITS(dev->devSettings.sampleType, BM_MODE_TEMP);
        }
        else {
            BYTE_UNSET_BITS(dev->devSettings.sampleType, BM_MODE_TEMP);
        }
    }

    return status;
}


status_t bm280_getPressureOS(BM_DEV dev, uint8_t *presOS)
{
    status_t status = STATUS_OK;
    uint8_t p = *presOS;
    p = dev->devSettings.humidOS;
    return status;
}



status_t bm280_setPressureOS(BM_DEV dev, BM_overSample_t *os)
{
    status_t status = STATUS_OK;
    uint8_t _os = *os;

    if(_os > BM_OS_16) {
        status = STATUS_ERR_INVALID_ARG;
    }

    if(status == STATUS_OK &&
       gcd_i2c_read_mod_write(dev->i2cChannel, 
                              dev->deviceAddress, 
                              BM_REG_ADDR_CTRL_MEASURE, 
                              (_os << 2), 
                              0b11100) == STATUS_OK)
    {
        dev->devSettings.pressOS = *os;
        if (_os > 0) {
            BYTE_SET_BITS(dev->devSettings.sampleType, BM_MODE_PRESSURE);
        }
        else {
            BYTE_UNSET_BITS(dev->devSettings.sampleType, BM_MODE_PRESSURE);
        }

    }

    return status;
}


status_t bm280_getSampleMode(BM_DEV dev, uint8_t *sampleMode)
{
    status_t status = STATUS_OK;

    *sampleMode = dev->devSettings.sampleMode;

    return status;
}


status_t bm280_setSampleMode(BM_DEV dev, BM_sampleMode_t *sampleMode)
{
    status_t status = STATUS_OK;
    uint8_t sm = *sampleMode;
    if(sm > 3 || sm == 2) {
        status = STATUS_ERR_INVALID_ARG;
    }
    if (status == STATUS_OK && 
        gcd_i2c_read_mod_write(dev->i2cChannel, 
                               dev->deviceAddress, 
                               BM_REG_ADDR_CTRL_MEASURE, 
                               sm, 
                               0b11) == STATUS_OK)
    {
        dev->devSettings.sampleMode = sm;
    }

    return status;
}


status_t bm280_getSampleType(BM_DEV dev, uint8_t *sampleType)
{
    status_t status = STATUS_OK;

    *sampleType = dev->devSettings.sampleType;

    return status;
}


status_t bm280_getFilterSetting(BM_DEV dev, uint8_t *filter)
{
    status_t status = STATUS_OK;
    *filter = dev->devSettings.filterCoefficient;
    return status;
}


status_t bm280_setFilterSetting(BM_DEV dev, bm_filter_t *filter)
{
    status_t status = STATUS_OK;
    uint8_t reg = 0;
    uint8_t f = *filter;

    if(f > BM_FILTER_16) {
        status = STATUS_ERR_INVALID_ARG;
    }

    if(status == STATUS_OK && 
        gcd_i2c_read_mod_write(dev->i2cChannel,
                               dev->deviceAddress, 
                               BM_REG_ADDR_CONFIG, 
                               (f << 2), 
                               0b111000) == STATUS_OK)
    {
        dev->devSettings.filterCoefficient = f;
    }
    return status;
}


status_t bm280_getSampleInterval(BM_DEV dev, uint8_t *dT)
{
    status_t status = STATUS_OK;
    *dT = dev->devSettings.sampleInterval;
    return status;
}


status_t bm280_setSampleInterval(BM_DEV dev, BM_standbyT_t *dT)
{
    status_t status = STATUS_OK;
    uint8_t reg = 0;
    uint8_t _dt = *dT;

    if(_dt >= BM_T_STDBY_END) {
        status = STATUS_ERR_INVALID_ARG;
    } 
    else if(gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg) == STATUS_OK)
    {
        reg &= 0b00011111;
        reg |= (_dt << 5);
        status = gcd_i2c_write_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_CONFIG, 1, &reg);
        dev->devSettings.sampleInterval = _dt;
    }
    else{
        status = STATUS_ERR_TIMEOUT;
    }
    return status;
}


void bmCtrlTask(void *args)
{
    /**  DONE: Message pump - dont really need - only actuve command is sample?
     *        task delay based on timing? 
     *        auto sample mode for latest?
     * **/
    BM_DEV dev = (BM_DEV )args;

    uint16_t sleep_time = 0;

    while (1)
    {
#ifdef DEBUG
        log_info(BM_DRIVER_TAG, "Mode: %u", dev->devSettings.sampleMode);
#endif
        if (dev->devSettings.sampleMode == BM_FORCE_MODE)
        {
            /** wait for task notify  to sample**/
            uint32_t trigd = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (trigd > 0) {
                bm280_updateMeasurements(dev);
            }
#ifdef DEBUG
            else {
                log_info(BM_DRIVER_TAG, "Not sampling");                
            }
#endif        
        }
        else if (dev->devSettings.sampleMode == BM_NORMAL_MODE)
        {
            uint8_t reg = 0;

            /** check the device isn't copying sample data... **/
            if (gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
            {
                while (gcd_i2c_read_address(dev->i2cChannel, dev->deviceAddress, BM_REG_ADDR_DEV_STATUS, 1, &reg) & BM_STATUS_MEASURE_MASK)
                {
                    vTaskDelay(pdMS_TO_TICKS(wait_sample_fin));
                }
            }
            log_info(BM_DRIVER_TAG, "updating");
            bm280_updateMeasurements(dev);
            sleep_time = bm280_sleepTime(dev);

            /** sleep until next measurement - this can get busy at high frequencies **/
            /** TODO: Test at high frequency sampling **/
            vTaskDelay(pdMS_TO_TICKS(sleep_time));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(wait_idle));
        }
    }
}

#ifdef CONFIG_DRIVERS_USE_HEAP
BM_DEV bm280_init(bm_initData_t *initData)
#else
BM_DEV bm280_init(BM_DEV dev, bm_initData_t *initData)
#endif
{
    status_t initStatus = STATUS_OK;

#ifdef CONFIG_DRIVERS_USE_HEAP
    /** assign some heap memory for the control structure **/
    BM_DEV dev = (BM_DEV )calloc(1, sizeof(bm_controlData_t));
    if (dev == NULL)
    {
        log_error(BM_DRIVER_TAG, "Error in assigning control structure memory!");
        initStatus = STATUS_ERR_NO_MEM;
    }
#else
    memset(dev, 0, sizeof(bm_controlData_t));
#endif

    if (initStatus == STATUS_OK)
    {
        dev->devSettings.sampleType = initData->sampleType;
        dev->devSettings.sampleMode = initData->sampleMode;
        dev->devSettings.humidOS = 0;
        dev->devSettings.tempOS = 0;
        dev->devSettings.pressOS = 0;
        dev = initData;
        dev->i2cChannel = initData->i2cChannel;

        if(initData->use_cbuffer && initData->cbuff != NULL) {
            dev->cbuff = initData->cbuff;
            dev->use_cbuffer = true;
        }

        if (initData->addressPinState)
        {
            dev->deviceAddress = (uint8_t)BM_I2C_ADDRESS_SDHIGH;
        }
        else
        {
            dev->deviceAddress = (uint8_t)BM_I2C_ADDRESS_SDLOW;
        }

        log_info(BM_DRIVER_TAG, "Device address - %u", dev->deviceAddress);
    }

    if (initStatus == STATUS_OK)
    {
        initStatus = bm280_getCalibrationData(dev);
        if (initStatus == STATUS_OK)
        {
            log_info(BM_DRIVER_TAG, "Succesfully got the calibration data!");
        }
        else
        {
            log_info(BM_DRIVER_TAG, "Error getting Calibration data!");
        }
    }

    if (initStatus == STATUS_OK)
    {
        initStatus = bm280_InitDeviceSettings(dev);
        if (initStatus == STATUS_OK)
        {
            log_info(BM_DRIVER_TAG, "Succesfully wrote device settings data!");
        }
        else
        {
            log_info(BM_DRIVER_TAG, "Error writing device settings!");
        }
    }

    bm280_getDeviceStatus(dev);
 
    xTaskCreate(bmCtrlTask, "bmCtrlTask", 5012, (void *)dev, 3, NULL);

    return dev;
}
