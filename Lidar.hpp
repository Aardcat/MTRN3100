// =============================================================================
//  Lidar.hpp  -  Front VL6180X time-of-flight distance sensor
//  MTRN3100 Micromouse   (same sensor as Lab02)
//
//  AI DISCLOSURE (assignment 5.1): written with assistance of a generative AI
//  (Claude); logic reviewed by the team.
// =============================================================================
//  WHAT THIS IS FOR
//  A VL6180X measures the distance to whatever is in front of it, in mm
//  (reliable up to ~200 mm - perfect for the 200->100 mm task).
//
//  THE ADDRESS PROBLEM
//  The kit has up to 3 TOF sensors on the SAME I2C bus, and every VL6180X
//  powers up at the SAME default address (0x29). If two are on at once they
//  clash. Each sensor has a shutdown pin (its "GP0" line on the PCB). So we:
//     1. hold ALL sensors off,
//     2. switch ON only the FRONT sensor,
//     3. initialise it.
//  For task 3.2 we only need the front one, so that is all we turn on.
//
//  WIRING (from the kit schematic): the three TOF GP0/enable lines are on
//  A0 (TOF1), A1 (TOF2), A2 (TOF3). Set which one is your FRONT sensor below.
//
//  Requires the Pololu "VL6180X" Arduino library (the one used in Lab02).
// =============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    // frontEnable = GP0 pin of the FRONT sensor.
    // other1/other2 = GP0 pins of the other two sensors (held OFF so they don't
    // clash on the bus). Use 255 if a sensor is not fitted.
    Lidar(uint8_t frontEnable, uint8_t other1 = 255, uint8_t other2 = 255)
        : frontPin(frontEnable), otherA(other1), otherB(other2) {}

    bool begin() {
        Wire.begin();
        turnOff(frontPin);           // start with everything off
        turnOff(otherA);
        turnOff(otherB);
        delay(10);

        pinMode(frontPin, OUTPUT);   // power up ONLY the front sensor
        digitalWrite(frontPin, HIGH);
        delay(50);                   // let it boot

        sensor.init();
        sensor.configureDefault();
        sensor.setTimeout(250);
        return true;
    }

    // Distance in mm from the SENSOR face to the wall (0..~255).
    uint8_t readMM() { return sensor.readRangeSingleMillimeters(); }

    bool timedOut() { return sensor.timeoutOccurred(); }

private:
    void turnOff(uint8_t pin) {
        if (pin != 255) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    }
    uint8_t frontPin, otherA, otherB;
    VL6180X sensor;
};

}  // namespace mtrn3100
