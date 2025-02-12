#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <RH_RF95.h>

class LoRa {
public:
    // Constructor
    LoRa(int csPin, int intPin, float frequency, uint8_t txPower);

    // Initializes the LoRa module
    bool init();

    // Sends a message
    bool sendMessage(const char* message);

    // rx function
    bool is_available();

private:
    int _csPin;
    int _intPin;
    float _frequency;
    uint8_t _txPower;
    RH_RF95 _rf95; // RH_RF95 object
};

#endif // LORA_H