//version5.8

#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDControl.hpp"
#include "MovementController.hpp"
#include "Lidar.hpp"
#include "Wire.h"
#include "Filter.hpp"
#include "MazeMap.hpp"
#include "Display.hpp"

#include <Math.h>
#include "VL6180X.h"
#include <MPU6050_light.h>

// ---------------------------------------------------------------------------
//  DEBUG SWITCH  (flash saver)
// ---------------------------------------------------------------------------
//  Set DEBUG to 1 while developing to get the Serial messages back.
//  Set it to 0 for the competition run: every print disappears at compile
//  time, which removes the text AND the number->string formatting code from
//  flash (worth roughly 2 kB).
#define DEBUG 0

#if DEBUG
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif



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
#define KP_H            3.0
#define KI_H            0  
#define KD_H            1.0
// PID values for forward speed control
#define KP_V            84
#define KI_V            2  
#define KD_V            8
// PID values for turning speed control
#define KP_W            30
#define KI_W            0  
#define KD_W            1

// Tolerances, maximums and minimums
#define TOLERANCE_FORWARD    10     // tolerance (mm)
#define TOLERANCE_SIDEWAYS   5      // tolerance (mm)
#define TOLERANCE_TURNING    1.0f      // tolerance (degrees)
#define TOLERANCE_HEADING 0.8f
#define TOLERANCE_FRONT_LIDAR 30
#define STRAIGHT_SETTLE_MS 175
#define TURN_SETTLE_MS     200    // must stay within tolerance this long to finish
#define MOVE_TIMEOUT  6000   // give up on a move after this many ms (safety)
#define TURN_TIMEOUT  4000   // give up on a move after this many ms (safety)
#define START_DELAY   2000   // pause after power-on so you can place + step back
#define MIN_PWM 20
#define MAX_PWM 255
#define MIN_TURN_PWM 10
#define MAX_TURN_PWM 75

#define MAX_BASE_PWM        90
#define MAX_MOTOR_PWM       120
#define MAX_ADJUSTMENT_PWM  4

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
#define TOF_FRONT_PIN  A2    
#define TOF_LEFT_PIN   A0
#define TOF_RIGHT_PIN  A1

// 3.2 drive and stop (DO NOT NEED FOR REAL MICROMOUSE MAZE) 
#define TARGET_MM          100.0f  // desired gap between robot FRONT and wall
#define SENSOR_TO_FRONT_MM 20.0f    // MEASURE: mm the sensor face sits behind the front bumper
#define KP_LIDAR           0.8f    // PWM per mm of distance error (tune)
#define MIN_MOVE_PWM       40      // min PWM 
#define BASE_PWM     55       
#define MAX_LIDAR_ADJUSTMENT 8
#define LIDAR_MIN_VALID_MM     20.0f
#define LIDAR_MAX_VALID_MM     200.0f
#define LIDAR_DEADBAND_MM      3.0f

// Maze mapping / OLED display (assignment 4.3)
// A wall counts as "there" if the lidar reads closer than this (mm).
// Cell is 180 mm wide, so ~100 is a safe midpoint - TUNE on the real maze.
#define WALL_THRESHOLD_MM 120
#define FRONT_WALL_THRESHOLD_MM 80


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
mtrn3100::PIDController ControllerH(KP_H, KI_H, KD_H, TOLERANCE_HEADING, false); // PID Controller for small heading adjustment
mtrn3100::PIDController ControllerW(KP_W, KI_W, KD_W, TOLERANCE_TURNING, true); // PID Controller for stationary turns
mtrn3100::Encoder encL(M1_ENC_A, M1_ENC_B, M1_ENC_INVERT);
mtrn3100::Encoder encR(M2_ENC_A, M2_ENC_B, M2_ENC_INVERT);
mtrn3100::MovementController MovementControl(WHEEL_RADIUS_MM, WHEEL_TRACK_MM);

mtrn3100::LidarArray lidars(TOF_FRONT_PIN, TOF_LEFT_PIN, TOF_RIGHT_PIN);

// Maze map + OLED display (assignment 4.3 "visualisation ... with a % completion")
mtrn3100::MazeMap maze;
mtrn3100::Display display;

// filters
Filter filterL(3);  // faster side response
Filter filterF(3);  // fastest front stop response
Filter filterR(3);  // faster side response


// struct for defining cell based movement
struct cellMovement {
    int row;
    int col;
};

struct Waypoint {
    float x;
    float y;
};

struct ContinuousSection {
    const Waypoint *points;
    int count;
};


//change this struct with coordinates on teh day
const Waypoint continuousPath67[] = {
    {375,300},
    {292,127},
    {214,112},
    {150,75},
    {100,50}
};
const ContinuousSection continuousSections[] = {
    { continuousPath67, sizeof(continuousPath67) / sizeof(continuousPath67[0]) }
};

const int continuousSectionCount = sizeof(continuousSections) / sizeof(continuousSections[0]);


// Tiny wrappers so attachInterrupt() can reach the encoder objects.
void isrL() { encL.readEncoder(); }
void isrR() { encR.readEncoder(); }

//  Unit conversions
// long countsForDistance(float mm) {
//     float wheel_circ = PI * WHEEL_DIAMETER_MM;          // mm per wheel revolution
//     return static_cast<long>(mm / wheel_circ * COUNTS_PER_REV + 0.5f);
// }

// long countsForTurn(float degrees) {
//     float arc_mm = (WHEEL_TRACK_MM / 2.0f) * (degrees * PI / 180.0f);
//     return countsForDistance(arc_mm);
// }

uint8_t compassFacing() {
    uint8_t startIdx;
    if      (START_DIR == FORWARD)  startIdx = 0;   // north
    else if (START_DIR == RIGHT)    startIdx = 1;   // east
    else if (START_DIR == BACKWARD) startIdx = 2;   // south
    else                            startIdx = 3;   // west

    // how many 90-degree ANTICLOCKWISE turns since the start
    uint8_t ccw;
    int f = MovementControl.getCurrFacing();
    if      (f == FORWARD)  ccw = 0;
    else if (f == LEFT)     ccw = 1;
    else if (f == BACKWARD) ccw = 2;
    else                    ccw = 3;                // RIGHT

    return (uint8_t)((startIdx + 4 - ccw) & 3);
}

// Convert CellX/CellY (mm travelled from the start cell) back into maze row/col.
// This is the inverse of the transform used in driveToMazeCell().
int8_t currentRow() {
    int dx = MovementControl.getCellX() / CELL_LENGTH;
    int dy = MovementControl.getCellY() / CELL_LENGTH;
    if      (START_DIR == FORWARD)  return START_ROW - dx;
    else if (START_DIR == RIGHT)    return START_ROW - dy;
    else if (START_DIR == BACKWARD) return START_ROW + dx;
    else                            return START_ROW + dy;
}

int8_t currentCol() {
    int dx = MovementControl.getCellX() / CELL_LENGTH;
    int dy = MovementControl.getCellY() / CELL_LENGTH;
    if      (START_DIR == FORWARD)  return START_COL - dy;
    else if (START_DIR == RIGHT)    return START_COL + dx;
    else if (START_DIR == BACKWARD) return START_COL + dy;
    else                            return START_COL - dx;
}

// Record the current cell + the walls around it, then redraw the OLED.
// Call this whenever the robot is STOPPED in a cell (lidar reads are slow, and
// they are only meaningful when stationary).
void updateMapAndDisplay() {
    int8_t  row    = currentRow();
    int8_t  col    = currentCol();
    uint8_t facing = compassFacing();

    maze.visit(row, col);

    // one reading per sensor, filtered, converted to "is there a wall?"
    bool wallF = filterF.update(lidars.readFrontMM()) < WALL_THRESHOLD_MM;
    bool wallL = filterL.update(lidars.readLeftMM())  < WALL_THRESHOLD_MM;
    bool wallR = filterR.update(lidars.readRightMM()) < WALL_THRESHOLD_MM;
    maze.setWallsFromSensors(row, col, facing, wallF, wallL, wallR);

    display.showMaze(maze, row, col);

    DBG(F("cell(")); DBG(row); DBG(F(","));
    DBG(col); DBG(F(") "));
    DBG(maze.percentComplete()); DBGLN(F("% mapped"));
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

float lidarCorrections() {

    float left_distance  = filterL.update(lidars.readLeftMM());
    float right_distance = filterR.update(lidars.readRightMM());

    bool isLeftWall  = left_distance < WALL_THRESHOLD_MM;
    bool isRightWall = right_distance < WALL_THRESHOLD_MM;

    float error = 0.0f;

    if (isLeftWall && isRightWall) {
        error = left_distance - right_distance;
    }
    else if (isLeftWall) {
        float desired_side_distance = 70.0f;
        error = left_distance - desired_side_distance;
    }
    else if (isRightWall) {
        float desired_side_distance = 70.0f;
        error = desired_side_distance - right_distance;
    }

    if (fabs(error) < 5.0f) {
        return 0.0f;
    }

    float corr = error * KP_LIDAR;

    return constrain(
        corr,
        -MAX_LIDAR_ADJUSTMENT,
        MAX_LIDAR_ADJUSTMENT
    );
}

// given a target distance in mm, drive there in a straight line while not deviating from 
// straight line track
void driveStraight(float distance) {
    encL.reset();
    encR.reset();
    MovementControl.zero();
    bool at_destination = false;
    uint32_t move_start = millis();
    uint32_t settle_start = 0;
    uint32_t last_lidar_update = 0;
    float lidar_adj = 0.0f;
    uint8_t front_distance = 255;

    ControllerH.zeroAndSetTarget(0, 0); // Target set as zero so that heading adjustment controller always tries to adjust back to correct heading
    ControllerV.zeroAndSetTarget(0, distance);

    while (!at_destination) {
        mpu.update();
        long left_count = encL.getCount();
        long right_count = encR.getCount();

        MovementControl.update(left_count, right_count, mpu.getAngleZ());

        // Updates local coordinates
        float curr_x = MovementControl.getX();
        float curr_y = MovementControl.getY();
        float curr_h = MovementControl.getLocalHDeg();
        
        float distance_error = distance - curr_x;
        //timeout for moves
        if (millis() - move_start > MOVE_TIMEOUT) {
            motorL.stop();
            motorR.stop();
            break;
        }


        float adjustment_speed = 0.0f; // default (L/R) adjustment speed should be zero
        float base_speed = clamp(ControllerV.compute(curr_x), MIN_PWM, MAX_BASE_PWM);

        float leftPWM = base_speed;
        float rightPWM = base_speed;

        if (fabs(curr_h) > TOLERANCE_HEADING) {
            adjustment_speed = fabs(ControllerH.compute(curr_h));
            // if (fabs(base_speed) < 40.0f) {
            //     max_heading_adjustment = 2.0f;
            // }
            adjustment_speed = constrain(adjustment_speed, 0, MAX_ADJUSTMENT_PWM);
        } 
        if (curr_h > TOLERANCE_HEADING) {
            leftPWM  += adjustment_speed;
            rightPWM -= adjustment_speed;
        }
        else if (curr_h < -TOLERANCE_HEADING) {
            leftPWM  -= adjustment_speed;
            rightPWM += adjustment_speed;
        }
   
    if (fabs(curr_h) < 3.0f) {
        if (millis() - last_lidar_update >= 30) {
            last_lidar_update = millis();
            lidar_adj = lidarCorrections();
            lidar_adj = constrain( lidar_adj, -MAX_LIDAR_ADJUSTMENT, MAX_LIDAR_ADJUSTMENT);
        }
    } else {
        lidar_adj = 0.0f;
    }
        // if (front_distance < FRONT_WALL_THRESHOLD_MM) {
        //     motorL.stop();
        //     motorR.stop();
        //     at_destination = true;
        //     break;
        // }
        leftPWM  -= lidar_adj;
        rightPWM += lidar_adj;
        leftPWM = constrain(leftPWM, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
        rightPWM = constrain(rightPWM, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);


        if (distance_error <= TOLERANCE_FORWARD) {
            motorL.stop();
            motorR.stop();
            if (settle_start == 0) {
                settle_start = millis();
            } 
            if (millis() - settle_start >= STRAIGHT_SETTLE_MS) {
                at_destination = true;
            }
        } else {
            settle_start = 0;
            motorL.setPWM(leftPWM);
            motorR.setPWM(rightPWM);
        }
    }
    
    motorL.stop();
    motorR.stop();    
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
}

float wrapAngle(float angle) {
    while (angle > 180.0f) { 
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

// turn to a target (local or global) heading (in degrees)
void turn(float heading, bool global) {
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
    float current_global_H = MovementControl.getHDeg();
    float target_turns;    
    if (global) {
        target_turns = wrapAngle(heading - current_global_H);
    } else {
        target_turns = wrapAngle(heading);
    }

    uint32_t move_start = millis();
    encL.reset();
    encR.reset();

    MovementControl.zero(); // Same as the function for moving forward, sets local origin at it's current pos
    ControllerW.zeroAndSetTarget(0, target_turns);
    bool at_destination = false;

    uint32_t settle_start = 0;
    bool settling = false;
    float previous_error = target_turns;

    while (!at_destination) {
        mpu.update();
        if (millis() - move_start > TURN_TIMEOUT) {
            motorL.stop();
            motorR.stop();
            break;
        }

        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
        float local_heading = MovementControl.getLocalHDeg();
        float speed = ControllerW.compute(local_heading);
        float error = fabs(ControllerW.getError());

        float max_expected_turn = fabs(target_turns) + 30.0f;

        if (fabs(local_heading) > max_expected_turn) {
            motorL.stop();
            motorR.stop();
            break;
        }
    float current_error =
            wrapAngle(target_turns - local_heading);

        if (fabs(current_error - previous_error) > 180.0f) {
            motorL.stop();
            motorR.stop();
            break;
        }

    previous_error = current_error;

        if (error <= TOLERANCE_TURNING) {
            motorL.stop();
            motorR.stop(); 
            if (settle_start == 0) {
                settle_start = millis();
            }
            if ( millis() - settle_start >= TURN_SETTLE_MS) {
                at_destination = true;
            }             
        }
        else {
            settle_start = 0;
            speed = clamp(speed,MIN_TURN_PWM, MAX_TURN_PWM);
            if (error < 15.0f) {
                speed = constrain(speed, -35, 35);
            }

            if (error < 5.0f) {
                speed = constrain(speed, -15, 15);
            }
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

    float delta_x = x - curr_x;
    float delta_y = y - curr_y;

    float distance = hypot(delta_x, delta_y);
    float heading =  atan2(delta_y, delta_x) * 180 / PI;

    // float heading;
    // if (fabs(delta_y) < 30.0f) {
    //     heading = (delta_x >=0) ? 0.0f : 180.0f;
    // } else if (fabs(delta_x) < 30.0f) {
    //     heading = (delta_y >=0) ? 0.0f : 180.0f;
    // } else {
    //     heading = atan2(delta_y, delta_x) * 180 / PI;
    // }

    turn(heading, true);
    driveStraight(distance);
    turn(heading, true);

}

// Executes a given string of commands where 'l' = left turn (90 degree turn), 'r' = right turn 
// (-90 degree turn), and 'f' = forward (drive straight 180mm i.e. one cell length)
// Uses CellX and CellY in the movementcontroller class -> these tell the robot it's x and y 
// coordinate within the current maze cell that it's in. Used for updating the direction it's 
// currently facing a bit easier. (only used within this function)


//continuos part for 4.2


void driveContinuousSection(const ContinuousSection &section) {
    for (int i = 0; i < section.count; i++) {
        mpu.update();
        MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());
        float curr_x = MovementControl.getGlobalX();
        float curr_y = MovementControl.getGlobalY();

        float dx = section.points[i].x - curr_x;
        float dy = section.points[i].y - curr_y;
        float distance = hypot(dx,dy);
        if (distance <10.0f) {
            continue;
        }
        driveToGlobalCoordinates(section.points[i].x, section.points[i].y);
    }
}

void pcc(float startHeading, float &desiredHeading) {
    mpu.update();
    MovementControl.update(encL.getCount(),encR.getCount(),mpu.getAngleZ());
    float currentHeading = MovementControl.getHDeg();
    float relativeHeading = wrapAngle(currentHeading - startHeading);    
    int quarterTurns = round(relativeHeading / 90.0f);
    desiredHeading = wrapAngle(startHeading + quarterTurns * 90.0f);
    turn(desiredHeading, true);
    int facingIndex = ((quarterTurns % 4) + 4) % 4;

    if (facingIndex == 0) {
        MovementControl.setCurrFacing(FORWARD);
    } else if (facingIndex == 1) {
        MovementControl.setCurrFacing(LEFT);
    } else if (facingIndex == 2) {
        MovementControl.setCurrFacing(BACKWARD);
    } else {
        MovementControl.setCurrFacing(RIGHT);
    }
}

void movementChain(const char *commands, const ContinuousSection *sections, int sectionCount) {
    MovementControl.setCurrFacing(FORWARD);
    MovementControl.setCellX(0);
    MovementControl.setCellY(0);
    mpu.update();
    MovementControl.update(encL.getCount(), encR.getCount(), mpu.getAngleZ());

    float startHeading = MovementControl.getHDeg();
    float desiredHeading = startHeading;
    int i = 0;
    int continuousIndex = 0;
    updateMapAndDisplay(); 

    while (i < (int)strlen(commands)) {
        // updateMapAndDisplay();
        if (commands[i] == 'f') {
            // Driving forward
            DBGLN(F("Driving forward!"));
            int straight_count = 0;

            int targetCellX = MovementControl.getCellX();
            int targetCellY = MovementControl.getCellY();

            if (MovementControl.getCurrFacing() == FORWARD) {
                targetCellX += CELL_LENGTH;
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                targetCellX -= CELL_LENGTH;
            } else if (MovementControl.getCurrFacing() == LEFT) {
                targetCellY += CELL_LENGTH;
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                targetCellY -= CELL_LENGTH;
            }
            turn(desiredHeading, true);
            driveStraight(CELL_LENGTH);
            mpu.update();
            MovementControl.update( encL.getCount(), encR.getCount(), mpu.getAngleZ());

            float straightHeadingError = MovementControl.getLocalHDeg();

            // Undo the heading error from the straight
            if (fabs(straightHeadingError) > TOLERANCE_HEADING) {
                turn(-straightHeadingError, false);
            }

            MovementControl.setCellX(targetCellX);
            MovementControl.setCellY(targetCellY);
        } else if (commands[i] == 'l') {
            // Turning left
            DBGLN(F("Turning left!"));
            
            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCurrFacing(LEFT);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCurrFacing(BACKWARD);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCurrFacing(RIGHT);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCurrFacing(FORWARD);
            }
            desiredHeading += 90.0f;
            while (desiredHeading > 180.0f)
                desiredHeading -= 360.0f;

            turn(desiredHeading, true);            

        } else if (commands[i] == 'r') {
            // Turning right
            DBGLN(F("Turning right!"));

            if (MovementControl.getCurrFacing() == FORWARD) {
                MovementControl.setCurrFacing(RIGHT);
            } else if (MovementControl.getCurrFacing() == RIGHT) {
                MovementControl.setCurrFacing(BACKWARD);
            } else if (MovementControl.getCurrFacing() == BACKWARD) {
                MovementControl.setCurrFacing(LEFT);
            } else if (MovementControl.getCurrFacing() == LEFT) {
                MovementControl.setCurrFacing(FORWARD);
            }
            desiredHeading -= 90.0f;
            while (desiredHeading < -180.0f)
                desiredHeading += 360.0f;

            turn(desiredHeading, true);   
        } else if (commands[i] == 'c') {
            DBGLN(F("Continuous section!"));
            if (continuousIndex < sectionCount) {
                driveContinuousSection(sections[continuousIndex]);
                continuousIndex++;
                pcc(startHeading, desiredHeading);
            }
        }
        updateMapAndDisplay();   // refresh map + % after every completed action
        i++;
    }
}

// Helper function for driving to global coordinates, given a target maze cell
// In maze -> start and end points given as (row, column, direction)
void driveToMazeCell(int row, int col) {
    //check to see if the maze is the correct size
    if (row < 0 || row >= 9 || col < 0 || col >= 9) {
        Serial.print("INVALID MAZE CELL: (");
        return;
    }
    int dist_row = row - START_ROW;
    int dist_col = col - START_COL;
    int target_x = 0;
    int target_y = 0;

    if (START_DIR == FORWARD) {
        // north -> x and y inverted
        target_x = -dist_row * CELL_LENGTH;
        target_y = -dist_col * CELL_LENGTH;
    } else if (START_DIR == RIGHT) {
        // east -> x is y, y is -x
        target_x = dist_col * CELL_LENGTH;
        target_y = -dist_row * CELL_LENGTH;
    } else if (START_DIR == BACKWARD) {
        // south -> x and y both correct
        target_x = dist_row * CELL_LENGTH;
        target_y = dist_col * CELL_LENGTH;
    } else if (START_DIR == LEFT) {
        // west -> x is -y, y is x
        target_x = -dist_col * CELL_LENGTH;
        target_y = dist_row * CELL_LENGTH;
    }

    driveToGlobalCoordinates(target_x, target_y);
    MovementControl.setCellX(target_x);
    MovementControl.setCellY(target_y);

    updateMapAndDisplay();
}
// void showMoveInfo(float heading, float delta_x, float delta_y) {

//     char line1[20];
//     char line2[20];

//     snprintf(line1, sizeof(line1), "H: %.1f", heading);
//     snprintf(line2, sizeof(line2), "dx%.0f dy%.0f", delta_x, delta_y);

//     display.showMessage(line1, line2);

//     delay(1500);
// }
// void showEncoderCounts() {
//     long leftCount  = encL.getCount();
//     long rightCount = encR.getCount();

//     char line1[20];
//     char line2[20];

//     snprintf(line1, sizeof(line1), "L: %ld", leftCount);
//     snprintf(line2, sizeof(line2), "R: %ld", rightCount);

//     display.showMessage(line1, line2);
//     delay(1500);
// }

// ----------------------------------------------------------------------------
//  SETUP 
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isrR, RISING);

    Wire.begin();

    // Start the OLED first so we can show progress while the IMU calibrates.
    display.begin();
    display.showMessage("WOBBLES", "starting...");

    //Set up the IMU
    byte status = mpu.begin();
    DBG(F("MPU6050 status: "));
    DBGLN(status);
    //while(status!=0){ } 
    
    DBGLN(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    DBGLN(F("Done!\n"));

    // Init lidars
    lidars.begin();

    motorL.stop();
    motorR.stop();

    delay(START_DELAY);

    //const char *commands = "flflflflflflflflflflflflrrr";
    const char *commands = "frffflfflfrfrflflfrflfclffrflf";
    // const char *commands = "fffff";

    movementChain(commands, continuousSections, continuousSectionCount);

    // leave the finished map + % on screen for the demonstrator
    display.showMaze(maze, currentRow(), currentCol());

    
}

void loop() {
    
    // int lastPrintTime = 0;
    // if (millis() - lastPrintTime >= 1000) {
    //     lastPrintTime = millis();

    //     uint8_t distF = lidars.readFrontMM();
    //     uint8_t distL = lidars.readLeftMM();
    //     uint8_t distR = lidars.readRightMM();

    //     DBG(F("Front: "));
    //     if (lidars.timedOutFront()) DBG(F("TIMEOUT")); else DBG(distF);
    //     DBG(F(" mm | "));
    //     DBG(lidars.readFrontAddress());

    //     DBG(F("Left: "));
    //     if (lidars.timedOutLeft()) DBG(F("TIMEOUT")); else DBG(distL);
    //     DBG(F(" mm | "));
    //     DBG(lidars.readLeftAddress());

    //     DBG(F("Right: "));
    //     if (lidars.timedOutRight()) DBG(F("TIMEOUT")); else DBG(distR);
    //     DBGLN(F(" mm"));
    //     DBG(lidars.readRightAddress());
    // }
    
}
