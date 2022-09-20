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
#include "Arduino.h"

const char* ArduinoBMA400_::getActivityString(uint8_t activity)
{
    switch (activity)
    {
        case BMA400_STILL_ACT:
            return (const char*)F("Still");
            break;
        case BMA400_WALK_ACT:
            return (const char*)F("Walking");
            break;
        case BMA400_RUN_ACT:
            return (const char*)F("Running");
            break;
        default:
            break;
    }
    return (const char*)F("Undefined");
};