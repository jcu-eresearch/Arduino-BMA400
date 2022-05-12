#include "Arduino.h"
#include "Wire.h"
#include "Arduino-BMA400.h"

#ifndef BMA_ADDRESS
#define BMA_ADDRESS BMA400_I2C_ADDRESS_SDO_HIGH
#endif


ArduinoBMA400 bma(&Wire, BMA_ADDRESS);
UART *stream = &Serial;

void i2c_scan(HardwareI2C *I2C)
{
    Serial.println();
    Serial.print("     0 1 2 3 4 5 6 7 8 9 A B C D E F");
    uint8_t status;
    for(size_t i = 0; i <= 127; i++)
    {

        if((i % 16) == 0)
        {
            if(i == 0){
                Serial.printf("\r\n0x00 ");
            }else {
                Serial.printf("\r\n0x%02x ", i);
            }
        }
        
        I2C->beginTransmission(i);
        status = 0xFF;
        status = I2C->endTransmission();
        if(status == 0)
        {
            Serial.print("* ");
        }else
        {
            Serial.print("- ");
        }

    }
    Serial.println();
};


void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(50000);
    i2c_scan(&Wire);
    bma.begin();
    bma.softReset();
    bma.startAccelerometer();

        
}

void loop()
{
    double x=0, y=0, z=0;
    bma.readAccelerometer(x, y, z);

    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.println(z);
}