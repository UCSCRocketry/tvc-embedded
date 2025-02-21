#include "lora.h"
#include <Arduino.h>

LoRa::LoRa(int csPin, int intPin, float frequency, uint8_t txPower)
    : _csPin(csPin), _intPin(intPin), _frequency(frequency), _txPower(txPower), _rf95(csPin, intPin) {}

bool LoRa::init() {
    pinMode(_csPin, OUTPUT);
    pinMode(_intPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // Manual reset (if needed)
    digitalWrite(_csPin, LOW);
    delay(10);
    digitalWrite(_csPin, HIGH);
    delay(10);

    if (!_rf95.init()) {
        Serial.println("LoRa radio init failed");
        return false;
    }

    Serial.println("LoRa radio init OK!");

    if (!_rf95.setFrequency(_frequency)) {
        Serial.println("SetFrequency failed");
        return false;
    }

    Serial.print("Set Freq to: ");
    Serial.println(_frequency);

    _rf95.setTxPower(_txPower, false);
    Serial.print("TX Power set to: ");
    Serial.println(_txPower);

    return true;
}

// Sends a message
bool LoRa::sendMessage(const char* packet) {
    Serial.print("Sending: ");
    Serial.println(packet);

    // Transmit the packet
    _rf95.send((uint8_t *)packet, strlen(packet) + 1); // Include null terminator
    _rf95.waitPacketSent();
    Serial.println("Packet sent!");

    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    Serial.println("Waiting for reply...");
    if (_rf95.waitAvailableTimeout(1000)) {
        if (_rf95.recv(buf, &len)) {
            Serial.print("Got reply: ");
            Serial.println((char *)buf);
            Serial.print("RSSI: ");
            Serial.println(_rf95.lastRssi(), DEC);
        } else {
            Serial.println("Receive failed");
        }
    } else {
        Serial.println("No reply, is there a listener around?");
    }
}

bool LoRa::is_available() {
    return _rf95.available();
}
