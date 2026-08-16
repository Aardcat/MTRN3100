#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {

class LidarArray {
public:
    // Constructor accepting pin assignments and custom I2C addresses
    LidarArray(uint8_t pinFront, uint8_t pinLeft, uint8_t pinRight,
               uint8_t addrFront = 0x52, uint8_t addrLeft = 0x54, uint8_t addrRight = 0x56)
        : pinF(pinFront), pinL(pinLeft), pinR(pinRight),
          addrF(addrFront), addrL(addrLeft), addrR(addrRight) {}

    void begin() {
        Wire.begin();

        turnOff(pinF);
        turnOff(pinL);
        turnOff(pinR);
        delay(10);

        // 2. Wake up and configure FRONT sensor ONLY
        turnOn(pinF);
        sensorF.init(); // FIX: Call as void, do not put inside if()
        sensorF.configureDefault();
        sensorF.setAddress(addrF);
        sensorF.setTimeout(250);

        // 3. Wake up and configure LEFT sensor ONLY
        turnOn(pinL);
        sensorL.init(); // FIX: Call as void, do not put inside if()
        sensorL.configureDefault();
        sensorL.setAddress(addrL);
        sensorL.setTimeout(250);

        // 4. Wake up and configure RIGHT sensor ONLY
        turnOn(pinR);
        sensorR.init(); // FIX: Call as void, do not put inside if()
        sensorR.configureDefault();
        sensorR.setAddress(addrR);
        sensorR.setTimeout(250);

    }

    uint16_t readFrontMM() { return sensorF.readRangeSingleMillimeters(); }
    uint16_t readLeftMM()  { return sensorL.readRangeSingleMillimeters(); }
    uint16_t readRightMM() { return sensorR.readRangeSingleMillimeters(); }
    uint8_t readFrontAddress() { return sensorF.readReg(0x212); }
    uint8_t readLeftAddress()  { return sensorL.readReg(0x212); }
    uint8_t readRightAddress() { return sensorR.readReg(0x212); }

    bool timedOutFront() { return sensorF.timeoutOccurred(); }
    bool timedOutLeft()  { return sensorL.timeoutOccurred(); }
    bool timedOutRight() { return sensorR.timeoutOccurred(); }

private:
    void turnOff(uint8_t pin) {
        if (pin != 255) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }

    void turnOn(uint8_t pin) {
        if (pin != 255) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH); // Pulling XSHUT high wakes up sensor
            delay(10);               // Small delay for boot sequence stability
        }
    }

    uint8_t pinF, pinL, pinR;
    uint8_t addrF, addrL, addrR;
    VL6180X sensorF, sensorL, sensorR;
};

}  // namespace mtrn3100
