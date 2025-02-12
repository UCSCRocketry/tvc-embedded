#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMUSensor {
public:
    IMUSensor(uint8_t address = 55);

    bool begin();
    void enableExternalCrystal();
    bool getOrientation(float &x, float &y, float &z);

private:
    Adafruit_BNO055 bno;
};

#endif // IMUSENSOR_H
