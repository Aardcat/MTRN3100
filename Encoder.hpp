// =============================================================================
//  Encoder.hpp  -  Quadrature encoder reader (interrupt driven)
//  MTRN3100 Micromouse - Week 4 Barebones Movement
//
//  AI DISCLOSURE (per assignment 5.1): This file was written with the
//  assistance of a generative AI (Claude). Logic was reviewed by the team.
// =============================================================================
//
//  Channel A of each motor encoder is wired to an Arduino Nano interrupt pin
//  (D2 = INT0, D3 = INT1). On every RISING edge of channel A we read channel B
//  to work out which way the wheel turned, then add/subtract one count.
//
//  This class is written to support MORE THAN ONE encoder, unlike the Lab03
//  starter. Each object stores its own count. Because attachInterrupt() needs
//  a plain function (not a member), the .ino defines tiny wrapper ISRs that
//  call readEncoder() on the matching object.
// =============================================================================
#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    // encA must be an interrupt-capable pin (D2 or D3 on the Nano).
    Encoder(uint8_t encA, uint8_t encB, bool invert = false)
        : enc_a_pin(encA), enc_b_pin(encB), invert(invert) {
        pinMode(enc_a_pin, INPUT_PULLUP);
        pinMode(enc_b_pin, INPUT_PULLUP);
    }

    // Called from the interrupt service routine on a RISING edge of channel A.
    // Channel B's level tells us the direction of rotation.
    void readEncoder() {
        // If A leads B we count one way; if B leads A we count the other.
        bool b = digitalRead(enc_b_pin);
        int8_t step = b ? 1 : -1;
        if (invert) step = -step;
        count += step;
    }

    // Raw signed pulse count since the last reset().
    long getCount() const {
        // count is volatile (updated in ISR); read it atomically.
        noInterrupts();
        long c = count;
        interrupts();
        return c;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }

private:
    const uint8_t enc_a_pin;
    const uint8_t enc_b_pin;
    const bool    invert;
    volatile long count = 0;
};

}  // namespace mtrn3100
