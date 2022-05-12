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
    ArduinoBMA400_Status_UNKNOWN = 0,
    ArduinoBMA400_Status_OK = 1,
    ArduinoBMA400_Status_ERROR = 0b10,
    ArduinoBMA400_Status_Null_PTR_ERROR = 0b110,
    ArduinoBMA400_Status_Com_Fail_ERROR = 0b1010,
    ArduinoBMA400_Status_Dev_Not_Found_ERROR = 0b10010,
    ArduinoBMA400_Status_Invalid_Config_ERROR = 0b100010,
};

typedef uint32_t ArduinoBMA400_Status;

enum ArduinoBMA400_IntStatus_e
{
    BMA400IntStatus_WAKEUP = BMA400_ASSERTED_WAKEUP_INT,
    BMA400IntStatus_ORIENT = BMA400_ASSERTED_ORIENT_CH,
    BMA400IntStatus_GEN1 = BMA400_ASSERTED_GEN1_INT,
    BMA400IntStatus_GEN2 = BMA400_ASSERTED_GEN2_INT,
    BMA400IntStatus_OVERRUN = BMA400_ASSERTED_INT_OVERRUN,
    BMA400IntStatus_FIFO_FULL = BMA400_ASSERTED_FIFO_FULL_INT,
    BMA400IntStatus_FIFO_WM = BMA400_ASSERTED_FIFO_WM_INT,
    BMA400IntStatus_DRDY = BMA400_ASSERTED_DRDY_INT,
    BMA400IntStatus_STEP = BMA400_ASSERTED_STEP_INT,
    BMA400IntStatus_S_TAP = BMA400_ASSERTED_S_TAP_INT,
    BMA400IntStatus_D_TAP = BMA400_ASSERTED_D_TAP_INT,
    BMA400IntStatus_ACT_CH_X = BMA400_ASSERTED_ACT_CH_X,
    BMA400IntStatus_ACT_CH_Y = BMA400_ASSERTED_ACT_CH_Y,
    BMA400IntStatus_ACT_CH_Z = BMA400_ASSERTED_ACT_CH_Z 
};

class ArduinoBMA400 {
    friend BMA400_INTF_RET_TYPE bma400_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
    friend BMA400_INTF_RET_TYPE bma400_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
private:
    bma400_dev bma400;
    uint8_t address;
    void* interface_ptr;
    ArduinoBMA400_Status getStatus(int8_t result);
public:

    explicit ArduinoBMA400(SPIClass *spi, PinName CS);
    explicit ArduinoBMA400(TwoWire *i2c, uint8_t address);

    void begin();
    ArduinoBMA400_Status softReset();
    ArduinoBMA400_Status countSteps(uint32_t &step_count, uint8_t &activity);
    ArduinoBMA400_Status startStepCounting();
    ArduinoBMA400_Status startAccelerometer();
    ArduinoBMA400_Status readAccelerometer(double &x, double &y, double &z);
    bool hasIntStatus(ArduinoBMA400_IntStatus_e status);

    bool isError(u_int32_t);

};

#endif //ARGOSTAG_ARDUINO_BMA400_H
