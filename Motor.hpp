// =============================================================================
//  Motor.hpp  -  DRV8835 (PH/EN mode) single-motor driver
//  MTRN3100 Micromouse - Week 4 Barebones Movement
//
//  AI DISCLOSURE (per assignment 5.1): This file was written with the
//  assistance of a generative AI (Claude). Logic was reviewed by the team.
// =============================================================================
//
//  The kit uses a DRV8835 dual H-bridge wired in PHASE/ENABLE mode:
//      ENABLE (xEN)  <- a PWM pin   -> sets motor SPEED   (0-255)
//      PHASE  (xPH)  <- a DIR pin   -> sets motor DIRECTION (HIGH/LOW)
//
//  setPWM() takes a signed value in [-255, 255]:
//      positive -> one direction, negative -> the other, 0 -> stop (coast).
// =============================================================================
#pragma once

#include <Arduino.h>

        #define MAX_PWM 150 // this change makes its smoother

namespace mtrn3100 {

class Motor {
public:
    // invert = true flips the meaning of positive/negative so that a positive
    // setPWM() always drives the WHEEL forwards regardless of how it is mounted.
    Motor(uint8_t pwm_pin, uint8_t dir_pin, bool invert = false)
        : pwm_pin(pwm_pin), dir_pin(dir_pin), invert(invert) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        digitalWrite(dir_pin, LOW);
        analogWrite(pwm_pin, 0);
    }

    // Drive the motor. Sign controls direction, magnitude controls speed.
    void setPWM(int16_t pwm) {
        if (invert) pwm = -pwm;

        // Clamp to the valid analogWrite range.
        if (pwm >  MAX_PWM) pwm =  MAX_PWM;
        if (pwm < -MAX_PWM) pwm = -MAX_PWM;

        // Phase pin selects direction; enable pin gets the |speed|.
        digitalWrite(dir_pin, (pwm >= 0) ? HIGH : LOW);
        analogWrite(pwm_pin, abs(pwm));
    }

    void stop() {
        analogWrite(pwm_pin, 0);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    const bool    invert;
};

}  // namespace mtrn3100
