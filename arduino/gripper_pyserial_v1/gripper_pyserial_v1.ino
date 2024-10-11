/* Script for controlling the tandem gripper with an Arduino Zero board 
 *  
 * Alejandro Velasquez
 * velasale@oregonstate.edu
 * 09/27/2024
 *
 * Colaborators
 */

// TODO VALVE
// TODO SENSOR PRESSURE
// TODO SENSOR DISTANCE

// TODO STEPPER
// TODO SERVO


// libraries
#include <Wire.h>
#include <time.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_VL53L0X.h>
#include <Stepper.h>


// stepper pins
const byte ENABLE_PINA = 8;     // pin to enable part A of stepper driver
const byte ENABLE_PINB = 13;    // pin to enable part B of stepper driver
// valve pins
#define VALVE 7;                // pin to turn on/off electric valve



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
const int TOTAL_DISTANCE = 58 * (200/8);   // 58mm * (200 steps / 1rev) * (1rev / 8mm)


// initializations
Stepper gripperStepper(STEPS_PER_REVOLUTION, 9, 10, 11, 12);
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


// Serial read stuff
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int dataNumber = 0;


void setup() {
    // Initialize stepper motor

    // Initialize pressure and distance sensors
    mpr.begin();
    lox.begin();
    lox.startRangeContinuous(10);

    // Serial initialization
    Serial.begin(9600);
    while (!Serial);
    clearInputBuffer();

    // Initialize VALVE pin as output
    delay(10);
    pinMode(VALVE, OUTPUT);
    delay(10);
    digitalWrite(VALVE, LOW);
    delay(10);
  
}


void loop(){
    //wait for command
    recWithStartEndMarker();

    //execute command
    parseCommands();
}

void vacuumOn(){
    Serial.println("Arduino: turning vacuum on");
    digitalWrite(VALVE, HIGH);
}


void vacuumOff(){
    Serial.println("Arduino: turning vacumm off");
    digitalWrite(VALVE, LOW);
}



void motorSteps(int stp_speed, int stp_distance){
    digitalWrite(ENABLE_PINA, HIGH);
    digitalWrite(ENABLE_PINB, HIGH);

    gripperStepper.setSpeed(stp_speed);
    gripperStepper.step(stp_distance);

    digitalWrite(ENABLE_PINA, LOW);
    digitalWrite(ENABLE_PINB, LOW);
    delay(100);
}



void clearInputBuffer(){
    while(Serial.available() > 0){
        Serial.read();
    }
}

void recWithStartEndMarker(){
    static boolean recInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while(Serial.available() > 0 && newData == false){
        rc = Serial.read();
        if(recInProgress == true){
            if (rc != endMarker){
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars){
                    ndx = numChars - 1;
                }
            }
            else{
                receivedChars[ndx] = '\0'   //terminate string
                recInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker){
            recInProgress = true;
        }
    }
}





void parseCommands(){
    int c = 0;
    int c_idx = 0;
    int t_idx = 0;
    char temp[32];

    if (newData == true){
        //Convert serial monitor to int and cast as float
        int len = strlen(receivedChars);
        c = float atoi(receivedChars);
        newData = false

        //Manage serial input accordingly
        if (c==vacuum_on){
            vacuum_on();
        }
        else if (c==vacuum_off){
            vacuum_off();
        }

    }
}