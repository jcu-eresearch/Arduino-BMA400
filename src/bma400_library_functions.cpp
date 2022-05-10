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

#include "bma400.h"
#include "Arduino-BMA400.h"

BMA400_INTF_RET_TYPE bma400_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if(intf_ptr != nullptr)
    {
        auto bma400 = (ArduinoBMA400*)intf_ptr;
        if(bma400->bma400.intf == BMA400_I2C_INTF)
        {
            TwoWire *i2c = (TwoWire*)bma400->interface_ptr;
            i2c->beginTransmission(bma400->address);
            i2c->write(reg_addr);
            i2c->endTransmission();

            i2c->requestFrom(bma400->address, length);
            size_t i = 0;;
            for(; i < length; i++)
            {
                reg_data[i] = i2c->read();
            }
            if(i == length)
            {
                return BMA400_INTF_RET_SUCCESS;
            }
            return BMA400_E_COM_FAIL;

        }else if(bma400->bma400.intf == BMA400_SPI_INTF)
        {
            //ToDo: Implement SPI bma400_read
        }
    }
    return BMA400_E_NULL_PTR;
}

BMA400_INTF_RET_TYPE bma400_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if(intf_ptr != nullptr)
    {
        auto bma400 = (ArduinoBMA400 *) intf_ptr;
        if (bma400->bma400.intf == BMA400_I2C_INTF) {

            TwoWire *i2c = (TwoWire*)bma400->interface_ptr;
            i2c->beginTransmission(bma400->address);
            i2c->write(reg_addr);
            size_t i = 0;
            for(; i < length; i++) {
                i2c->write(reg_data[i]);
            }
            i2c->endTransmission();
            if(i == length)
            {
                return BMA400_INTF_RET_SUCCESS;
            }
            return BMA400_E_COM_FAIL;
        } else if (bma400->bma400.intf == BMA400_SPI_INTF) {
            //ToDo: Implement SPI bma400_read
        }
    }
    return BMA400_E_NULL_PTR;
}

void bma400_delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

