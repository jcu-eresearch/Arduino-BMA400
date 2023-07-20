/*
 * Arduino-BM400 is an Arduino Library for the Bosch BMA400 chip.
 * Copyright (C) 2021  eResearch, James Cook University
 * Author: NigelB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Repository: https://github.com/jcu-eresearch/Arduino-BMA400
 *
 */

#include "Arduino-BMA400.h"

ArduinoBMA400::ArduinoBMA400(SPIClass *spi, PinName CS, uint8_t new_step_counter_config[BMA_STEP_COUNTER_CONFIG_SIZE])
{
    memset(&bma400, 0, sizeof(bma400));
    bma400.intf = BMA400_SPI_INTF;
    this->address = CS;
    interface_ptr = spi;
    copyStepCounterConfig(new_step_counter_config);
};

ArduinoBMA400::ArduinoBMA400(TwoWire *i2c, uint8_t address, uint8_t new_step_counter_config[BMA_STEP_COUNTER_CONFIG_SIZE]){
    memset(&bma400, 0, sizeof(bma400));
    bma400.intf = BMA400_I2C_INTF;
    this->address = address;
    interface_ptr = i2c;
    copyStepCounterConfig(new_step_counter_config);
};

void ArduinoBMA400::copyStepCounterConfig(uint8_t new_step_counter_config[BMA_STEP_COUNTER_CONFIG_SIZE]){
    for(size_t i = 0; i < BMA_STEP_COUNTER_CONFIG_SIZE; i++)
    {
        step_counter_config[i] = new_step_counter_config[i];
    }
};


ArduinoBMA400_Status ArduinoBMA400::begin()
{
    int8_t rslt;
    bma400.intf_ptr = this;
    rslt = bma400_init(&bma400);
    printf("init result: %i\r\n", rslt);
    if(!isError(getStatus(rslt)))
    {
        rslt = bma400_set_step_counter_param(step_counter_config, &bma400);
        if(!isError(getStatus(rslt)))
        {
            uint8_t retrieved_step_counter_config[BMA_STEP_COUNTER_CONFIG_SIZE];
            rslt = bma400_get_regs(0x59, retrieved_step_counter_config, BMA_STEP_COUNTER_CONFIG_SIZE, &bma400);
            if(!isError(getStatus(rslt)))
            {
                if(memcmp(retrieved_step_counter_config, step_counter_config, BMA_STEP_COUNTER_CONFIG_SIZE) != 0)
                {
                    return ArduinoBMA400_Status_Failed_To_Set_Config_ERROR;
                }
            }
        };

    }
    return getStatus(rslt);
};

uint8_t ArduinoBMA400::getChipID()
{
    return bma400.chip_id;
};

bool ArduinoBMA400::validChipID()
{
    return getChipID() == BMA400_CHIP_ID;
};

ArduinoBMA400_Status ArduinoBMA400::softReset()
{
    return getStatus(bma400_soft_reset(&bma400));
};

ArduinoBMA400_Status ArduinoBMA400::startStepCounting()
{
    int rslt = 0;
    bma400_sensor_conf conf[2];
    struct bma400_int_enable int_en[2];
    memset(conf, 0, sizeof(conf));
    conf[0].type = BMA400_STEP_COUNTER_INT;
    conf[1].type = BMA400_ACCEL;
    rslt = bma400_get_sensor_conf(conf, 2, &this->bma400);
    // bma400_check_rslt("bma400_set_sensor_conf", rslt);

    conf[0].param.step_cnt.int_chan = BMA400_INT_CHANNEL_1;

    conf[1].param.accel.odr = BMA400_ODR_100HZ;
    conf[1].param.accel.range = BMA400_RANGE_2G;
    conf[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(conf, 2, &this->bma400);
    // bma400_check_rslt("bma400_set_sensor_conf", rslt);  


    int_en[0].type = BMA400_STEP_COUNTER_INT_EN;
    int_en[0].conf = BMA400_ENABLE;

    int_en[1].type = BMA400_LATCH_INT_EN;
    int_en[1].conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(int_en, 2, &this->bma400);      

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &this->bma400);
    return false;
};

ArduinoBMA400_Status ArduinoBMA400::startAccelerometer()
{
    int8_t rslt;
    bma400_sensor_conf conf;
    bma400_int_enable int_en;
    conf.type = BMA400_ACCEL;
    rslt = bma400_get_sensor_conf(&conf, 1, &this->bma400);
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_RANGE_2G;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;  
    rslt = bma400_set_sensor_conf(&conf, 1, &this->bma400);
    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &this->bma400); 

    int_en.type = BMA400_DRDY_INT_EN;
    int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&int_en, 1, &this->bma400);
    return rslt;
}

/**
 * @brief Convert the internal accelerometer representation to meters per second squared.
 * 
 * This function is from:
 * https://github.com/BoschSensortec/BMA400-API/blob/13086eb4702d743cac930dbba8a1f3096b0371bc/examples/accelerometer/accelerometer.c
 */
#define GRAVITY_EARTH     (9.80665f)
static float lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
    float accel_ms2;
    int16_t half_scale;

    half_scale = 1 << (bit_width - 1);
    accel_ms2 = (GRAVITY_EARTH * accel_data * g_range) / half_scale;

    return accel_ms2;

}

ArduinoBMA400_Status ArduinoBMA400::readAccelerometer(double &x, double &y, double &z)
{
    int8_t rslt;
    bma400_sensor_data data;
    while(!hasIntStatus(BMA400IntStatus_DRDY)){}
    rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, &this->bma400);

    if(rslt == BMA400_OK)
    {
            x = lsb_to_ms2(data.x, 2, 12);
            y = lsb_to_ms2(data.y, 2, 12);
            z = lsb_to_ms2(data.z, 2, 12);
    }
    return 0;
}

bool ArduinoBMA400::hasIntStatus(ArduinoBMA400_IntStatus_e status)
{
    int8_t rslt;
    uint16_t int_status;
    rslt = bma400_get_interrupt_status(&int_status, &this->bma400);
    if(rslt != BMA400_OK)
    {
        return false;
    }

    return status && int_status;

}

ArduinoBMA400_Status ArduinoBMA400::countSteps(uint32_t &step_count, uint8_t &activity)
{
    int8_t rslt = bma400_get_steps_counted(&step_count, &activity, &bma400);
    return getStatus(rslt);
}

bool ArduinoBMA400::isError(u_int32_t status)
{
    return (status & ArduinoBMA400_Status_ERROR) > 0;
}


ArduinoBMA400_Status ArduinoBMA400::getStatus(int8_t result)
{
    ArduinoBMA400_Status ret = ArduinoBMA400_Status_UNKNOWN;
    switch(result)
    {
        case BMA400_OK:
        {
            ret = ArduinoBMA400_Status_OK;
        }break;
        case BMA400_E_NULL_PTR:
        {
            ret = ArduinoBMA400_Status_Null_PTR_ERROR;
        }break;        
        case BMA400_E_COM_FAIL:
        {
            ret = ArduinoBMA400_Status_Com_Fail_ERROR;
        }break;        
        case BMA400_E_DEV_NOT_FOUND:
        {
            ret = ArduinoBMA400_Status_Dev_Not_Found_ERROR;
        }break;        
        case BMA400_E_INVALID_CONFIG:
        {
            ret = ArduinoBMA400_Status_Invalid_Config_ERROR;
        }break; 
        default:
        {

        }      
    }
    return ret;
}

