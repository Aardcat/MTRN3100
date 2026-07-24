#pragma once

#include <Arduino.h>
#include <Math.h>

#define COUNTS_PER_REV 700
#define FORWARD 0
#define LEFT 90
#define BACKWARD 180
#define RIGHT -90

namespace mtrn3100 {
class MovementController {
public:
    MovementController(float radius, float wheelBase) : x(0), y(0), localH(0), globalX(0), globalY(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    // Global reset - resets all parameters
    void globalZero() {
        x = 0;
        y = 0;
        localH = 0;
        globalX = 0;
        globalY = 0;
        h = 0;
        currFacing = FORWARD;
    }
    
    // Local reset - resets only local parameters
    void zero() {
		x = 0;
		y = 0;
		localH = 0;
        localHZeroRef = h;
        lastLPos = 0;
        lastRPos = 0;
    }

    void update(float leftValue, float rightValue, float angleValue) {
        float delta_left_radians = ((leftValue - lastLPos) / COUNTS_PER_REV) * 2 * PI; 
        float delta_right_radians = ((rightValue - lastRPos) / COUNTS_PER_REV) * 2 * PI; 

        float delta_s = (R * delta_left_radians) / 2 + (R * delta_right_radians) / 2;

        // Calculating forward kinematics (x and y pos)
        x += delta_s * cos(localH);
        globalX += delta_s * cos(h);
        y += delta_s * sin(localH);
        globalY += delta_s * sin(h);

        // Calculating forward kinematics (heading)
        float localHTrue = (angleValue * PI / 180) - localHZeroRef;
        localH = atan2(sin(localHTrue), cos(localHTrue)); // wraps around to be within -pi to pi
        h = (angleValue * PI / 180);

        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    void setCurrFacing(int direction) { currFacing = direction; }
    void setCellX(int value) { cellX = value; }
    void setCellY(int value) { cellY = value; }

    void setGlobalX(int value) { globalX = value; }
    void setGlobalY(int value) { globalY = value; }

    float getX() const { return x; }
    float getY() const { return y; }
    float getLocalH() const { return localH; }
    float getLocalHDeg() const { return localH * (180 / PI); }
    
    float getGlobalX() const { return globalX; }
    float getGlobalY() const { return globalY; }
    float getH() const { return h; }
    float getHDeg() const { return h * (180 / PI); }

    int getCurrFacing() const { return currFacing; }
    int getCellX() const { return cellX; }
    int getCellY() const { return cellY; }

    float signedDistanceToValue(float input, float target) const {
        float remainder = fmod(input, target);

        if (remainder > 90.0) {
            remainder -= 180.0;
        } else if (remainder < -90.0) {
            remainder += 180.0;
        }

        return remainder;
    }

private:
    float x, y, localH;
    float globalX, globalY, h;
    float localHZeroRef;
    const float R, B;
    float lastLPos, lastRPos;
    int currFacing;
    int cellX, cellY;
};

}
