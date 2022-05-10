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


ArduinoBMA400::ArduinoBMA400(SPIClass *spi, PinName CS)
{
    memset(&bma400, 0, sizeof(bma400));
    bma400.intf = BMA400_SPI_INTF;
    this->address = CS;
    interface_ptr = spi;
}

ArduinoBMA400::ArduinoBMA400(TwoWire *i2c, uint8_t address){
    memset(&bma400, 0, sizeof(bma400));
    bma400.intf = BMA400_I2C_INTF;
    this->address = address;
    interface_ptr = i2c;
}


void ArduinoBMA400::begin()
{
    int8_t rslt;
    bma400.intf_ptr = this;
    rslt = bma400_init(&bma400);
    printf("init result: %i\r\n", rslt);
    softReset();
    bma400_int_enable step_int{};
    step_int.type = BMA400_STEP_COUNTER_INT_EN;
    step_int.conf = BMA400_ENABLE;
    bma400_enable_interrupt(&step_int, 1, &bma400);
    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma400);


//    bma400.dev_id = ;
//int8_t rslt;
//memset(&bma400, 0, sizeof(bma400));
//bma400.intf_ptr = NULL; /* To attach your interface device reference */
////bma400.delay_ms = delay_ms;
//bma400.dev_id = ARGOSTAG_BMA400_ADDR_SEL;
////bma400.read = i2c_reg_read;
////bma400.write = i2c_reg_write;
//bma400.intf = BMA400_I2C_INTF;
//
//rslt = bma400_init(&bma);
//print_rslt(rslt);
//
//rslt = bma400_soft_reset(&bma);
//print_rslt(rslt);
//
//step_int.type = BMA400_STEP_COUNTER_INT_EN;
//step_int.conf = BMA400_ENABLE;
//
//rslt = bma400_enable_interrupt(&step_int, 1, &bma);
//print_rslt(rslt);
//
//rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma);
//print_rslt(rslt);
}

void ArduinoBMA400::softReset()
{
    bma400_soft_reset(&bma400);
}

void ArduinoBMA400::countSteps(uint32_t &step_count, uint8_t &activity)
{
    bma400_get_steps_counted(&step_count, &activity, &bma400);
}