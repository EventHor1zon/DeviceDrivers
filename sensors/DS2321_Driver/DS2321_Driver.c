/***************************************
* \file     DS2321_Driver.c
* \brief    A driver for the RTC device
*
* \date     March 2021
* \author   RJAM
****************************************/

/********* Includes *******************/
#include "port/error_type.h"
#include "port/log.h"
#include "port/types.h"
#include "port/malloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GenericCommsDriver.h"
#include "DS2321_Driver.h"


const char *DS_TAG = "DS2321";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

/****** Private Functions *************/

static uint8_t seconds_from_byte(uint8_t byte) {
    uint8_t secs = 0;
    secs += (byte >> 4) * 10;
    secs += (byte & 0x0f);
    return secs;
}


static uint8_t mins_from_byte(uint8_t byte) {
    uint8_t mins = 0;
    mins += (byte >> 4) * 10;
    mins += (byte & 0x0f);
    return mins;
}


static uint8_t hours_from_byte(uint8_t byte, bool twentyfour) {
    uint8_t hours = 0;
    hours = (byte & 0x0f);
    hours += ((byte & 0b00010000) >> 4) * 10;
    if(twentyfour) {
        hours += ((byte & 0b0010000) >> 5) * 20;
    }
    return hours;
}


static bool pm_from_byte(uint8_t byte) {
    return (byte & 0b00100000);
}


static uint8_t day_from_byte(uint8_t byte) {
    uint8_t day = (byte & 0b00000111);
    return day;
}


static uint8_t date_from_byte(uint8_t byte) {
    uint8_t date = 0;
    date += ((byte & 0b00110000) >> 4) * 10;
    date += (byte & 0x0f);
    return date;
}


static uint8_t month_from_byte(uint8_t byte) {
    uint8_t month = 0;
    month += ((byte & 0b00010000) >> 4) * 10;
    month += (byte & 0x0f);
    return month;
}


static uint8_t year_from_byte(uint8_t byte) {
    uint8_t year = 0;
    year += ((byte & 0xf0) >> 4) * 10;
    year += (byte & 0x0f);
    return year;
}


static void ds2321_driver_task(void *args) {
 
    DS2321_DEV dev = (ds2321_handle_t *)args;

   while(1) {

        if(dev->opmode == DS2321_OPMODE_ONDEMAND) {
            /** we don't really need the task - sleep until mode change **/
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else if(dev->opmode == DS2321_OPMODE_SAMPLE_1S) {
            STATUS_ERROR_CHECK(ds2321_update_time(dev));
            ds2321_dump_time(dev);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if(dev->opmode == DS2321_OPMODE_SAMPLE_5S) {
            ds2321_update_time(dev);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
   }
   /** here be dragons **/
}

/****** Global Data *******************/

/****** Global Functions *************/
#ifdef CONFIG_DRIVERS_USE_HEAP
DS2321_DEV ds2321_init(ds2321_init_t *ini) {
#else 
DS2321_DEV ds2321_init(DS2321_DEV dev, ds2321_init_t *ini) {
#endif
    status_t err = STATUS_OK;
    
    if(!gcd_i2c_check_bus(ini->i2c_bus)) {
        log_error(DS_TAG, "Invalid I2C bus");
        err = STATUS_ERR_INVALID_ARG;
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    DS2321_DEV dev = NULL;
    if(!err) {
        dev = (ds2321_handle_t *)heap_caps_calloc(1, sizeof(ds2321_handle_t), MALLOC_CAP_DEFAULT);
        if(dev == NULL) {
            err = STATUS_ERR_NO_MEM;
            log_error(DS_TAG, "Error assigning driver handle memory");
        }
    }
#else 
    memset(dev, 0, sizeof(ds2321_driver_task));
#endif

    if(!err) {
        /** set some defaults **/
        dev->i2c_bus = ini->i2c_bus;
        dev->opmode = DS2321_OPMODE_SAMPLE_1S;
        dev->settings.twentyfour_hour = true;
    }

    if(!err && xTaskCreate(ds2321_driver_task, "ds2321_driver_task", 5012, dev, 3, &(dev->t_handle)) != pdTRUE) {
        err = STATUS_ERR_NO_MEM;
        log_error(DS_TAG, "Error creating driver task");       
    }

#ifdef CONFIG_DRIVERS_USE_HEAP
    if(err && dev != NULL) {
        /** free mem if init fails **/
        heap_caps_free(dev);
    }
#endif

    if(err) {
        log_error(DS_TAG, "Failed to intialise DS2321 Driver :(");
    }
    else {
        log_info(DS_TAG, "Succesfully started DS2321 Driver");
    }

    return dev;
}




status_t ds2321_dump_time(DS2321_DEV dev) {


    log_info(DS_TAG, "Time: %02u:%02u:%02u - %02u / %02u / %02u", 
            dev->time.hours.hours_24, dev->time.minutes, dev->time.seconds, 
            dev->time.date, dev->time.month, dev->time.year);

    return STATUS_OK;
}





status_t ds2321_set_opmode(DS2321_DEV dev, uint8_t *opmode) {
    status_t err = STATUS_OK;
    uint8_t m = *opmode;
    if(m > DS2321_OPMODE_SAMPLE_5S) {
        err = STATUS_ERR_INVALID_ARG;
    } 
    else {
        dev->opmode = m;
        xTaskNotifyGive(dev->t_handle); /** if in on-demand, unblock **/
    }

    return err;
}


status_t ds2321_get_twentyfour(DS2321_DEV Dev, bool *state) {
    status_t status = STATUS_OK;
    *state = Dev->settings.twentyfour_hour;
    return status;
}


status_t ds2321_set_twentyfour(DS2321_DEV Dev, bool *state) {
    uint8_t i = (uint8_t )*state;
    uint8_t regval = 0;
    status_t err = gcd_i2c_read_address(Dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_HOURS, 1, &regval);
    if(!err) {   
        if(i && !(regval & (i << 5))) {
            regval |= (i << 5);
            err = gcd_i2c_write_address(Dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_HOURS, 1, &regval);
            if(!err) {
                Dev->settings.twentyfour_hour = true;
            }
        }
        else if(!(i) && regval & (1 << 5)){
            regval &= ~(1 << 5); 
            err = gcd_i2c_write_address(Dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_HOURS, 1, &regval);
            if(!err) {
                Dev->settings.twentyfour_hour = false;
            }
        }
    }
    return err;
}


status_t ds2321_update_time(DS2321_DEV dev) {
    status_t err = STATUS_OK;

    uint8_t buffer[7] = {0};

    err = gcd_i2c_read_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_SECONDS, 7, buffer);

    if(!err) {
        dev->time.seconds = seconds_from_byte(buffer[0]);
        dev->time.minutes = mins_from_byte(buffer[1]);
        if(dev->settings.twentyfour_hour) {
            dev->time.hours.hours_24 = hours_from_byte(buffer[2], 1);
        }
        else {
            dev->time.hours.hours_12 = hours_from_byte(buffer[3], 0);
            dev->time.am = !(pm_from_byte(buffer[2]));
        }
        dev->time.day = day_from_byte(buffer[4]);
        dev->time.date = date_from_byte(buffer[5]);
        dev->time.month = month_from_byte(buffer[6]);
        dev->time.year = year_from_byte(buffer[7]);
    }

    return err;
}


status_t ds2321_set_seconds(DS2321_DEV dev, uint8_t *secs) {
    status_t status = STATUS_OK;
    uint8_t t = *secs;
    if(t > 59) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = t % 10;
        uint8_t _t = t / 10;
        uint8_t r = ((_t << 4) | _s); 

        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_SECONDS, 1, &r);
    }
    return status;
}


status_t ds2321_set_minutes(DS2321_DEV dev, uint8_t *mins) {
    status_t status = STATUS_OK;
    uint8_t t = *mins;
    if(t > 59) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = t % 10;
        uint8_t _t = t / 10;
        uint8_t r = ((_t << 4) | _s); 

        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_MINUTES, 1, &r);
    }
    return status;
}


status_t ds2321_set_hours(DS2321_DEV dev, uint8_t *hours) {
    status_t status = STATUS_OK;
    uint8_t t = *hours;
    if((dev->settings.twentyfour_hour && t > 23) || ((!(dev->settings.twentyfour_hour)) && (t > 12 || t < 1))) {
        status = STATUS_ERR_INVALID_ARG;
    } 
    else {
        uint8_t _s = t % 10;
        uint8_t _t = t / 10;    /** this works, if _t == 2, the 20hr set instead of 10 **/
        uint8_t r = ((_t << 4) | _s); 
        /** make sure we dont overwrite the 24hr bit **/
        if(dev->settings.twentyfour_hour) {
            r |= 0b01000000;
        }
        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_HOURS, 1, &r);
    }
    return status;
}


status_t ds2321_set_day(DS2321_DEV dev, uint8_t *day) {
    status_t status = STATUS_OK;
    uint8_t t = *day;
    if(t > 7 || t < 1) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = t % 10;
        uint8_t _t = t / 10;
        uint8_t r = ((_t << 4) | _s); 

        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_DAY, 1, &r);
    }
    return status;
}


status_t ds2321_set_date(DS2321_DEV dev, uint8_t *date) {
    status_t status = STATUS_OK;
    uint8_t d = *date;
    if(d > 31 || d < 1) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = d % 10;
        uint8_t _t = d / 10;
        uint8_t r = ((_t << 4) | _s); 
        log_info(DS_TAG, "Setting data: %02x", r);
        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_DATE, 1, &r);
    }
    return status;
}


status_t ds2321_set_month(DS2321_DEV dev, uint8_t *m) {
    status_t status = STATUS_OK;
    uint8_t _m = *m;
    if(_m > 31  || _m < 1) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = _m % 10;
        uint8_t _t = _m / 10;
        uint8_t r = ((_t << 4) | _s); 

        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_MONTH, 1, &r);
    }
    return status;
}


status_t ds2321_set_year(DS2321_DEV dev, uint8_t *y) {
    status_t status = STATUS_OK;
    uint8_t _y = *y;
    if(_y > 99 ) {
        status = STATUS_ERR_INVALID_ARG;
    } else {
        uint8_t _s = _y % 10;
        uint8_t _t = _y / 10;
        uint8_t r = ((_t << 4) | _s); 
        log_info(DS_TAG, "Year: %02x %02x %02x", _s, _t, r);

        status = gcd_i2c_write_address(dev->i2c_bus, DS2321_I2C_DEVICE_ADDR, DS2321_REGADDR_YEAR, 1, &r);
    }
    return status;
}