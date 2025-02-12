#include "imu.h"

IMUSensor::IMUSensor(uint8_t address) : bno(address) {}

bool IMUSensor::begin() {
    if (!bno.begin()) {
        return false;
    }
    delay(1000);
    return true;
}

void IMUSensor::enableExternalCrystal() {
    bno.setExtCrystalUse(true);
}

bool IMUSensor::getOrientation(float &x, float &y, float &z) {
    sensors_event_t event;
    bno.getEvent(&event);

    if (event.orientation.x != NAN && event.orientation.y != NAN && event.orientation.z != NAN) {
        x = event.orientation.x;
        y = event.orientation.y;
        z = event.orientation.z;
        return true;
    }
    return false;
}
