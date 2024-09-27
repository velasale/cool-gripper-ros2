/* Script for controlling the tandem gripper with an Arduino Zero board 
 *  
 * Alejandro Velasquez
 * velasale@oregonstate.edu
 * 09/27/2024
 */

// TODO STEPPER
// TODO VALVE
// TODO SENSOR PRESSURE
// TODO SENSOR DISTANCE


// libraries
#include <Wire.h>
#include <time.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_VL53L0x.h>
#include <Stepper.h>


// pins
const byte ENABLE_PINA = 8;
const byte ENABLE_PINB = 13;



// parameters
const int STEPS_PER_REVOLUTION = 200;   // Adjust according to your motor
const int STEPS = 1500;
const int STEP_SPEED = 15;
/* If L298N driver is used, speed should be within these ranges:
 *   <Accelstepper.h>  450 < x < 1100  units: steps/sec
 *   <Stepper.h>       140 < x < 340   units: rpm
 */
const int CLOSING_SPEED = 240;    
const int CLOSING_SPEED_FAST = 340;
const int OPENING_SPEED = 330;
const int TOTAL_DISTANCE = 58 * (200/8)   // 58mm * (200 steps / 1rev) * (1rev / 8mm)



// 


void setup() {
  // Initialize stepper motor
  




  
}
