#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDControl.hpp"
#include "MovementController.hpp"
#include "Lidar.hpp"
#include "Wire.h"
#include "Filter.hpp"

#include <Math.h>
#include <Wire.h>
#include "VL6180X.h"
#include <MPU6050_light.h>


// arduino pins
#define M1_PWM 11
#define M1_DIR 12
#define M1_ENC_A 2    
#define M1_ENC_B 7

#define M2_PWM 9
#define M2_DIR 10
#define M2_ENC_A 3     
#define M2_ENC_B 8


// Wheel constants
#define COUNTS_PER_REV 700.0f      
#define WHEEL_DIAMETER_MM 32.0f    
#define WHEEL_RADIUS_MM 16.0f
#define WHEEL_TRACK_MM 84.0f  

////////////////////////////////////////////////////////////////////////////////////////////////////
// PLEASE TUNE THESE PID VALUES!!!!!
////////////////////////////////////////////////////////////////////////////////////////////////////

// PID values for small heading adjustment
#define KP_H            20
#define KI_H            0  
#define KD_H            0
// PID values for forward speed control
#define KP_V            84
#define KI_V            2  
#define KD_V            8
// PID values for turning speed control
#define KP_W            25
#define KI_W            0  
#define KD_W            1

// Tolerances, maximums and minimums
#define TOLERANCE_FORWARD    10     // tolerance (mm)
#define TOLERANCE_SIDEWAYS   5      // tolerance (mm)
#define TOLERANCE_TURNING    2.5f      // tolerance (degrees)
#define TOLERANCE_FRONT_LIDAR 25
#define SETTLE_MS     250    // must stay within tolerance this long to finish
#define MOVE_TIMEOUT  8000   // give up on a move after this many ms (safety)
#define START_DELAY   2000   // pause after power-on so you can place + step back
#define MIN_PWM 20
#define MAX_PWM 255
#define MAX_ADJUSTMENT_PWM 70

// Constants for maze
#define FORWARD 0
#define LEFT 90
#define BACKWARD 180
#define RIGHT -90
#define CELL_LENGTH 180

// Motor and encoder inverting
#define M1_MOTOR_INVERT true
#define M2_MOTOR_INVERT false       
#define M1_ENC_INVERT  false
#define M2_ENC_INVERT  true 

// Lidar enable pins A0 A3 A1
#define TOF_FRONT_PIN  A0    
#define TOF_LEFT_PIN   A3
#define TOF_RIGHT_PIN  A1

// 3.2 drive and stop (DO NOT NEED FOR REAL MICROMOUSE MAZE) 
#define TARGET_MM          100.0f  // desired gap between robot FRONT and wall
#define SENSOR_TO_FRONT_MM 0.0f    // MEASURE: mm the sensor face sits behind the front bumper
#define STOP_TOL_MM        6.0f    // deadband so it rests within +/-5 mm
#define KP_LIDAR           1.5f    // PWM per mm of distance error (tune)
#define MIN_MOVE_PWM       40      // min PWM 
#define BASE_PWM     55       

// Maze start and end points (given)
#define START_ROW 0
#define START_COL 0
#define START_DIR BACKWARD
#define GOAL_ROW 9
#define GOAL_COL 9


// constructors
MPU6050 mpu(Wire);
mtrn3100::Motor motorL(M1_PWM, M1_DIR, M1_MOTOR_INVERT);
mtrn3100::Motor motorR(M2_PWM, M2_DIR, M2_MOTOR_INVERT);
mtrn3100::PIDController ControllerV(KP_V, KI_V, KD_V, TOLERANCE_FORWARD, false); // PID Controller for forward movement
mtrn3100::PIDController ControllerH(KP_H, KI_H, KD_H, TOLERANCE_TURNING, false); // PID Controller for small heading adjustment
mtrn3100::PIDController ControllerW(KP_W, KI_W, KD_W, TOLERANCE_TURNING, true); // PID Controller for stationary turns
mtrn3100::Encoder encL(M1_ENC_A, M1_ENC_B, M1_ENC_INVERT);
mtrn3100::Encoder encR(M2_ENC_A, M2_ENC_B, M2_ENC_INVERT);
mtrn3100::MovementController MovementControl(WHEEL_RADIUS_MM, WHEEL_TRACK_MM);

mtrn3100::LidarArray lidars(TOF_FRONT_PIN, TOF_LEFT_PIN, TOF_RIGHT_PIN);

// filters
Filter filterL(3);  // faster side response
Filter filterF(3);  // fastest front stop response
Filter filterR(3);  // faster side response
Filter filterIMU(10);


// struct for defining cell based movement
struct cellMovement {
    int row;
    int col;
};


// Tiny wrappers so attachInterrupt() can reach the encoder objects.
void isrL() { encL.readEncoder(); }
void isrR() { encR.readEncoder(); }

//  Unit conversions
long countsForDistance(float mm) {
    float wheel_circ = PI * WHEEL_DIAMETER_MM;          // mm per wheel revolution
    return static_cast<long>(mm / wheel_circ * COUNTS_PER_REV + 0.5f);
}

long countsForTurn(float degrees) {
    float arc_mm = (WHEEL_TRACK_MM / 2.0f) * (degrees * PI / 180.0f);
    return countsForDistance(arc_mm);
}

// Speed clamping helper function
float clamp(float value, float min, float max) {
    if (value > 0) {
        return (value < min) ? min : ((value > max) ? max : value);
    } else if (value < 0) {
        return -((-value < min) ? min : ((-value > max) ? max : -value));
    } else {
        return 0;
    }
}

// given a target distance in mm, drive there in a straight line while not deviating from 
// straight line track
void driveStraight(float distance) {
    encL.reset();
    encR.reset();
    MovementControl.zero(); // Sets current position and heading as the origin and constructs a set of
                            // local coordinates based on this position
    bool at_destination = false;
    uint32_t settle_start = 0;

    ControllerH.zeroAndSetTarget(0, 0); // Target set as zero so that heading adjustment controller always tries to adjust back to correct heading
    ControllerV.zeroAndSetTarget(0, distance);

    delay(150);
    
    while (!at_destination) {
        mpu.update();
        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());

        // Updates local coordinates
        float curr_x = MovementControl.getX();
        float curr_y = MovementControl.getY();
        float curr_h = MovementControl.getLocalHDeg();
        
        /*
        Serial.print("curr_x is: ");
        Serial.println(curr_x);
        Serial.print("curr_y is: ");
        Serial.println(curr_y);
        Serial.print("curr_h is: ");
        Serial.println(curr_h);
        */

        float adjustment_speed = 0; // default (L/R) adjustment speed should be zero
        float base_speed = clamp(ControllerV.compute(curr_x), MIN_PWM, MAX_PWM);
        float leftPWM;
        float rightPWM;

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // please tune the PID better than I did so that you don't need this 30% reduction!!!
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        if (curr_y > TOLERANCE_SIDEWAYS) {
            // drifted too far left
            adjustment_speed = fabs(clamp(ControllerH.compute(curr_y), MIN_PWM, MAX_ADJUSTMENT_PWM));
            base_speed = base_speed * 0.7;
            leftPWM = base_speed + adjustment_speed;
            rightPWM = base_speed - (adjustment_speed * 1.5);

        } else if (curr_y < -TOLERANCE_SIDEWAYS) {
            // drifted too far right
            adjustment_speed = fabs(clamp(ControllerH.compute(curr_y), MIN_PWM, MAX_ADJUSTMENT_PWM));
            base_speed = base_speed * 0.7;
            leftPWM = base_speed - (adjustment_speed * 1.5);
            rightPWM = base_speed + adjustment_speed;

        } else if (curr_h > TOLERANCE_TURNING) {
            // pointing too much left (only if not drifting too much)
            adjustment_speed = fabs(clamp(ControllerH.compute(curr_h), MIN_PWM, MAX_ADJUSTMENT_PWM));
            base_speed = base_speed * 0.7;
            leftPWM = base_speed + adjustment_speed;
            rightPWM = base_speed - (adjustment_speed * 1.5);

        } else if (curr_h < -TOLERANCE_TURNING) {
            // pointing too much right (only if not drifting too much)
            adjustment_speed = fabs(clamp(ControllerH.compute(curr_h), MIN_PWM, MAX_ADJUSTMENT_PWM));
            base_speed = base_speed * 0.7;
            leftPWM = base_speed - (adjustment_speed * 1.5);
            rightPWM = base_speed + adjustment_speed;   

        } else {
            // within tolerances -> normal operation
            leftPWM = base_speed;
            rightPWM = base_speed;
        }

        // Check front lidar distance, stop if too close to wall in front
        /*
        float raw_distance_f = lidars.readFrontMM();
        float front_distance = filterF.update(raw_distance_f);
        Serial.print("Front lidar distance is: ");
        Serial.println(front_distance);
        if (front_distance <= TOLERANCE_FRONT_LIDAR) {
            Serial.println("stop!");
            base_speed == 0;
            leftPWM = base_speed;
            rightPWM = base_speed;
        }
        */
        
        if (abs(ControllerV.getError()) <= TOLERANCE_FORWARD) {
            motorL.stop();
            motorR.stop();
            if (settle_start == 0) {
                settle_start = millis();
            } 
            if (millis() - settle_start >= SETTLE_MS) {
                at_destination = true;
            }
        } else {
            settle_start = 0;
            motorL.setPWM(clamp(leftPWM, MIN_PWM, MAX_PWM));
            motorR.setPWM(clamp(rightPWM, MIN_PWM, MAX_PWM));
        }
    }
    
    motorL.stop();
    motorR.stop();    
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
}

// turn to a target (local or global) heading (in degrees)
void turn(float heading, bool global) {
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
    float current_global_H = MovementControl.getHDeg();
    encL.reset();
    encR.reset();

    float target_turns;
    
    if (global) {
        target_turns = heading - current_global_H;
    } else {
        target_turns = heading;
    }
    //wrapping heading
    while (target_turns > 180) { 
        target_turns -= 360;
    }
    while (target_turns < -180) {
        target_turns += 360;
    };

    MovementControl.zero(); // Same as the function for moving forward, sets local origin at it's current pos
   
    bool at_destination = false;

    ControllerW.zeroAndSetTarget(0, target_turns);

    // Serial.print("turn start x is ");
    // Serial.println(MovementControl.getGlobalX());
    // Serial.print("turn start y is ");
    // Serial.println(MovementControl.getGlobalY());
    uint32_t settle_start = 0;

    while (!at_destination) {
        mpu.update();
        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
        float local_heading = MovementControl.getLocalHDeg();
        float speed = ControllerW.compute(local_heading);

        if (abs(ControllerW.getError()) <= TOLERANCE_TURNING) {
            motorL.stop();
            motorR.stop();
            if (settle_start == 0) {
                settle_start = millis();
            } 
            if (millis() - settle_start >= SETTLE_MS) {
                at_destination = true;
            }
        } else {
            settle_start = 0;
            speed = clamp(speed, MIN_PWM, MAX_PWM);
            motorL.setPWM(-speed);
            motorR.setPWM(speed);
        }
    }
    motorL.stop();
    motorR.stop();
}

// Calculates straight line path given an x and y global coordinate, turns towards the destination, 
// then drives there using the driveStraight function.
void driveToGlobalCoordinates(float x, float y) {
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
    float curr_x = MovementControl.getGlobalX();
    float curr_y = MovementControl.getGlobalY();

    // Serial.print("x is ");
    // Serial.println(x);
    // Serial.print("y is ");
    // Serial.println(y);

    // Serial.print("curr_x is ");
    // Serial.println(curr_x);
    // Serial.print("curr_y is ");
    // Serial.println(curr_y);

    float delta_x = x - curr_x;
    float delta_y = y - curr_y;

    float distance = hypot(delta_x, delta_y);
    
    float heading = atan2(delta_y, delta_x) * 180 / PI;

    turn(heading, true);

    // Serial.print("distance is ");
    // Serial.println(distance);
    // Serial.print("desired heading is ");
    // Serial.println(heading);
    // Serial.print("current heading is ");
    // Serial.println(curr_heading);

    delay(100);

    driveStraight(distance);

    // Serial.print("new x is ");
    // Serial.println(MovementControl.getGlobalX());
    // Serial.print("new y is ");
    // Serial.println(MovementControl.getGlobalY());
}

// Executes a given string of commands where 'l' = left turn (90 degree turn), 'r' = right turn 
// (-90 degree turn), and 'f' = forward (drive straight 180mm i.e. one cell length)
// Uses CellX and CellY in the movementcontroller class -> these tell the robot it's x and y 
// coordinate within the current maze cell that it's in. Used for updating the direction it's 
// currently facing a bit easier. (only used within this function)


//pure speed optimisaion function 

void movementChain(const String &commands) {
    MovementControl.setCurrFacing(FORWARD);
    MovementControl.setCellX(0);
    MovementControl.setCellY(0);
    int i = 0;

    while (i < commands.length()) {
        Serial.println("Executing command...");

        if (commands[i] == 'f') {
            // Driving forward
            Serial.println("Driving forward!");
            int straight_count = 0;
            while (i + straight_count < commands.length() && commands[i + straight_count] == 'f') {
                straight_count++;
            }
            int distance = straight_count * CELL_LENGTH;
            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCellX(MovementControl.getCellX() + distance);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCellX(MovementControl.getCellX() - distance);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCellY(MovementControl.getCellY() + distance);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCellY(MovementControl.getCellY() - distance);
            }
            i+= straight_count - 1;
            driveToGlobalCoordinates(MovementControl.getCellX(), MovementControl.getCellY());
        } else if (commands[i] == 'l') {
            // Turning left
            Serial.println("Turning left!");
            
            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCurrFacing(LEFT);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCurrFacing(BACKWARD);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCurrFacing(RIGHT);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCurrFacing(FORWARD);
            }
            
            turn(90, false);
        } else if (commands[i] == 'r') {
            // Turning right
            Serial.println("Turning right!");

            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCurrFacing(RIGHT);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCurrFacing(BACKWARD);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCurrFacing(LEFT);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCurrFacing(FORWARD);
            }

            turn(-90, false);
        }

        Serial.println("Action executed.");
        delay(150);
        i++;
    }
}

// Helper function for driving to global coordinates, given a target maze cell
// In maze -> start and end points given as (row, column, direction)
void driveToMazeCell(int row, int col) {
    int dist_row = row - START_ROW;
    int dist_col = col - START_COL;
    int target_x;
    int target_y;

    if (START_DIR == FORWARD) {
        // north -> x and y inverted
        target_x = -dist_row * 180;
        target_y = -dist_col * 180;
    } else if (START_DIR == RIGHT) {
        // east -> x is y, y is -x
        target_x = dist_col * 180;
        target_y = -dist_row * 180;
    } else if (START_DIR == BACKWARD) {
        // south -> x and y both correct
        target_x = dist_row * 180;
        target_y = dist_col * 180;
    } else if (START_DIR == LEFT) {
        // west -> x is -y, y is x
        target_x = -dist_col * 180;
        target_y = dist_row * 180;
    }

    driveToGlobalCoordinates(target_x, target_y);
}

// ----------------------------------------------------------------------------
//  SETUP 
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isrR, RISING);

    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    //while(status!=0){ } 
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");

    // Init lidars
    lidars.begin();

    motorL.stop();
    motorR.stop();

    delay(START_DELAY);

    //String commands = "flflflflflflflflflflflflrrr";
    //movementChain(commands);
    int move_length = 12;
    struct cellMovement moves[move_length] = {
        {1, 0},
        {1, 1},
        {0, 1},
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
        {0, 0},
        {1, 0},
        {1, 1},
        {0, 1},
        {0, 0},
    };

    for (int i = 0; i < move_length; i++) {
        driveToMazeCell(moves[i].row, moves[i].col);
    }

    
    
}

void loop() {
    
    int lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000) {
        lastPrintTime = millis();

        uint8_t distF = lidars.readFrontMM();
        uint8_t distL = lidars.readLeftMM();
        uint8_t distR = lidars.readRightMM();

        Serial.print("Front: ");
        if (lidars.timedOutFront()) Serial.print("TIMEOUT"); else Serial.print(distF);
        Serial.print(" mm | ");
        Serial.print(lidars.readFrontAddress());

        Serial.print("Left: ");
        if (lidars.timedOutLeft()) Serial.print("TIMEOUT"); else Serial.print(distL);
        Serial.print(" mm | ");
        Serial.print(lidars.readLeftAddress());

        Serial.print("Right: ");
        if (lidars.timedOutRight()) Serial.print("TIMEOUT"); else Serial.print(distR);
        Serial.println(" mm");
        Serial.print(lidars.readRightAddress());
    }
    
}
