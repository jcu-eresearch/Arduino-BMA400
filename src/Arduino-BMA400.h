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

#ifndef ARDUINO_BMA400_H
#define ARDUINO_BMA400_H

#include "SPI.h"
#include "Wire.h"

#include "bma400.h"

enum ArduinoBMA400_Status_e
{

};

class ArduinoBMA400 {
    friend BMA400_INTF_RET_TYPE bma400_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
    friend BMA400_INTF_RET_TYPE bma400_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
private:
    bma400_dev bma400;
    uint8_t address;
    void* interface_ptr;
public:

    explicit ArduinoBMA400(SPIClass *spi, PinName CS);
    explicit ArduinoBMA400(TwoWire *i2c, uint8_t address);

    void begin();
    void softReset();
    void countSteps(uint32_t &step_count, uint8_t &activity);
};

#endif //ARGOSTAG_ARDUINO_BMA400_H
