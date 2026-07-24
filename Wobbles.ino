#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDControl.hpp"
#include "BangBangControl.hpp"
#include "MovementController.hpp"
#include "Lidar.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

#include <Math.h>

// arduino pins
#define M1_PWM 11
#define M1_DIR 12
#define M1_ENC_A 2    
#define M1_ENC_B 7

#define M2_PWM 9
#define M2_DIR 10
#define M2_ENC_A 3     
#define M2_ENC_B 8

// Front lidar enable pins
#define TOF_FRONT_PIN  A3     //a0 is left lidar
#define TOF_LEFT_PIN A0
#define TOF_RIGHT_PIN A1

// constants
#define COUNTS_PER_REV 700.0f      
#define WHEEL_DIAMETER_MM 32.0f    
#define WHEEL_RADIUS_MM 16.0f
#define WHEEL_TRACK_MM 84.0f  

// PID values for small heading adjustment
#define KP_H            60
#define KI_H            0  
#define KD_H            8
// PID values for forward speed control
#define KP_V            80
#define KI_V            0  
#define KD_V            10
// PID values for turning speed control
#define KP_W            180
#define KI_W            5  
#define KD_W            10

// Tolerances, maximums and minimums
#define TOLERANCE_FORWARD    10     // tolerance (mm)
#define TOLERANCE_SIDEWAYS   5      // tolerance (mm)
#define TOLERANCE_TURNING    2.5f      // tolerance (degrees)
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

// motor and encoder inverting
#define M1_MOTOR_INVERT true
#define M2_MOTOR_INVERT false       
#define M1_ENC_INVERT  false
#define M2_ENC_INVERT  true 

// 3.2 drive and stop 
#define TARGET_MM          100.0f  // desired gap between robot FRONT and wall
#define SENSOR_TO_FRONT_MM 0.0f    // MEASURE: mm the sensor face sits behind the front bumper
#define STOP_TOL_MM        6.0f    // deadband so it rests within +/-5 mm
#define KP_LIDAR           1.5f    // PWM per mm of distance error (tune)
#define MIN_MOVE_PWM       40      // min PWM 
#define BASE_PWM     55       


// constructors
MPU6050 mpu(Wire);
mtrn3100::Motor motorL(M1_PWM, M1_DIR, M1_MOTOR_INVERT);
mtrn3100::Motor motorR(M2_PWM, M2_DIR, M2_MOTOR_INVERT);
mtrn3100::PIDController ControllerV(KP_V, KI_V, KD_V, TOLERANCE_FORWARD);
mtrn3100::PIDController ControllerH(KP_H, KI_H, KD_H, TOLERANCE_TURNING);
mtrn3100::PIDController ControllerW(KP_H, KI_H, KD_H, TOLERANCE_TURNING);
mtrn3100::Encoder encL(M1_ENC_A, M1_ENC_B, M1_ENC_INVERT);
mtrn3100::Encoder encR(M2_ENC_A, M2_ENC_B, M2_ENC_INVERT);
mtrn3100::MovementController MovementControl(WHEEL_RADIUS_MM, WHEEL_TRACK_MM);
mtrn3100::Lidar lidar(TOF_FRONT_PIN, TOF_LEFT_PIN, TOF_RIGHT_PIN);


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

// given a target distance in mm, drive there while not deviating from straight line track
void driveStraight(float distance) {
    encL.reset();
    encR.reset();
    MovementControl.zero();
    bool at_destination = false;

    ControllerH.zeroAndSetTarget(0, 0);
    ControllerV.zeroAndSetTarget(0, distance);

    delay(150);

    while (!at_destination) {
        mpu.update();
        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());

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

        float adjustment_speed = 0;
        float base_speed = clamp(ControllerV.compute(curr_x), MIN_PWM, MAX_PWM);
        float leftPWM;
        float rightPWM;

        // The following block of if statements will tell the robot to adjust the speed of its wheels by a PWM adjustment factor,
        // based on if it is drifting left or right of its designated track (will be corrected first) or if it is pointing off the centerline.
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
        
        if (base_speed == 0) {
            motorL.stop();
            motorR.stop();
        } else {
            motorL.setPWM(clamp(leftPWM, MIN_PWM, MAX_PWM));
            motorR.setPWM(clamp(rightPWM, MIN_PWM, MAX_PWM));
            /*
            Serial.print("left speed is: ");
            Serial.println(leftPWM);
            Serial.print("right speed is: ");
            Serial.println(rightPWM);
            */
        }
        
        at_destination = base_speed == 0;
    }

    motorL.stop();
    motorR.stop();
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
}

// turn to a target (local or global) heading (in degrees)
void turn(float heading, bool global) {
    encL.reset();
    encR.reset();
    MovementControl.zero();
    bool at_destination = false;

    ControllerW.zeroAndSetTarget(0, heading);
    float start_x = MovementControl.getGlobalX();
    float start_y = MovementControl.getGlobalY();

    Serial.print("turn start x is ");
    Serial.println(MovementControl.getGlobalX());
    Serial.print("turn start y is ");
    Serial.println(MovementControl.getGlobalY());

    while (!at_destination) {
        mpu.update();
        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
        float curr_h;
        if (global) {
            curr_h = MovementControl.getHDeg();
        } else {
            curr_h = MovementControl.getLocalHDeg();
        }
        float speed = clamp(ControllerW.compute(curr_h), MIN_PWM, MAX_PWM);

        //Serial.println(curr_h);
     
        motorL.setPWM(-speed);
        motorR.setPWM(speed);

        at_destination = (speed == 0);
    }

    motorL.stop();
    motorR.stop();
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());

    float end_x = MovementControl.getGlobalX();
    float end_y = MovementControl.getGlobalY();

    //MovementControl.setGlobalX(start_x);
    //MovementControl.setGlobalY(start_y);
    Serial.print("turn end x is ");
    Serial.println(MovementControl.getGlobalX());
    Serial.print("turn end y is ");
    Serial.println(MovementControl.getGlobalY());
}

void driveToGlobalCoordinates(float x, float y) {
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
    float curr_x = MovementControl.getGlobalX();
    float curr_y = MovementControl.getGlobalY();

    Serial.print("x is ");
    Serial.println(x);
    Serial.print("y is ");
    Serial.println(y);

    Serial.print("curr_x is ");
    Serial.println(curr_x);
    Serial.print("curr_y is ");
    Serial.println(curr_y);

    float delta_x = x - curr_x;
    float delta_y = y - curr_y;

    float distance = hypot(delta_x, delta_y);
    

    float heading = atan2(delta_y, delta_x) * 180 / PI;

    Serial.print("distance is ");
    Serial.println(distance);
    Serial.print("heading is ");
    Serial.println(heading);

    turn(heading, true);

    delay(100);

    driveStraight(distance);

    Serial.print("new x is ");
    Serial.println(MovementControl.getGlobalX());
    Serial.print("new y is ");
    Serial.println(MovementControl.getGlobalY());
}

void movementChain(const String &commands) {
    MovementControl.setCurrFacing(FORWARD);
    MovementControl.setCellX(0);
    MovementControl.setCellY(0);
    int i = 0;

    while (i < commands.length()) {
        Serial.println("Executing command...");

        if (commands[i] == 'f') {
            Serial.println("Driving forward!");

            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCellX(MovementControl.getCellX() + CELL_LENGTH);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCellX(MovementControl.getCellX() - CELL_LENGTH);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCellY(MovementControl.getCellY() + CELL_LENGTH);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCellY(MovementControl.getCellY() - CELL_LENGTH);
            }

            driveToGlobalCoordinates(MovementControl.getCellX(), MovementControl.getCellY());

        } else if (commands[i] == 'l') {
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
            
            //Serial.println(MovementControl.getCurrFacing());
            turn(90, false);


            //turn(MovementControl.getCurrFacing(), true);

        } else if (commands[i] == 'r') {
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

            //Serial.println(MovementControl.getCurrFacing());
            turn(-90, false);

            
            //turn(MovementControl.getCurrFacing(), false);

        }

        Serial.println("Action executed.");
        delay(150);
        i++;
    }
}

void driveAndStop() {
    Serial.println("Drive & stop: holding 100 mm from wall (runs continuously).");
    unsigned long t = 0;

    while (true) {
        int reading  = lidar.readMM();              
        float gap    = reading - SENSOR_TO_FRONT_MM;  
        float error  = gap - TARGET_MM;              
        float mag    = (error < 0) ? -error : error;

        int16_t out;
        if (mag <= STOP_TOL_MM) {
            out = 0;                                   
        } else {
            out = (int16_t)(KP_LIDAR * error);         
            int16_t om = (out < 0) ? -out : out;       
            if (om < MIN_MOVE_PWM) out = (error > 0) ? MIN_MOVE_PWM : -MIN_MOVE_PWM;
            if (out >  BASE_PWM) out =  BASE_PWM;
            if (out < -BASE_PWM) out = -BASE_PWM;
        }
        motorL.setPWM(out);                            
        motorR.setPWM(out);

        if (millis() - t >= 200) {
            t = millis();
            Serial.print("dist="); Serial.print(reading);
            Serial.print("mm  err="); Serial.print(error, 1);
            Serial.print("  pwm="); Serial.println(out);
        }
    }
}

// ----------------------------------------------------------------------------
//  SETUP 
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isrR, RISING);

    //Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");

    lidar.begin();
    Serial.println("Front lidar started.");

    motorL.stop();
    motorR.stop();


    delay(START_DELAY);             // place robot on the line, then step away

    // Task 1 - straight line
    /*
    driveStraight(1000);
    delay(300);
    */

    // Task 2 - LIDAR
    //driveAndStop();

    // Task 3 - Turning
    
    turn(-90, true);
    unsigned long start_time = millis();
    unsigned long delay_time = 7000;
    while (millis() - start_time < delay_time) {
        mpu.update();
        MovementControl.update(0, 0, mpu.getAngleZ());
        Serial.println(MovementControl.getHDeg());
    }
    turn(-88, true);
    

    // Task 4 - Chaining Movements
    /*
    String commands = "lfrfffrf";
    movementChain(commands);
    */
}

void loop() {
    // Sequence runs once in setup(); nothing to repeat here.
}
