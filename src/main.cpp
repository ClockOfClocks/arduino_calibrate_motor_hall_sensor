#include <Arduino.h>

const int hallPinPower = 52;     // the number of the hall effect sensor pin power
const int hallPin = 53;     // the number of the hall effect sensor pin
const int ledPin =  13;     // the number of the LED pin
// variables will change:
int hallState = 0;          // variable for reading the hall sensor status

int state = 0;
int calibrateState = 0;
int calibrateStepIn;
int calibrateStepOut;

#include <AccelStepper.h>
#define HALFSTEP 8

// Motor pin definitions
#define motor1Pin1  30     // IN1 on the ULN2003 driver 1
#define motor1Pin2  32     // IN2 on the ULN2003 driver 1
#define motor1Pin3  34     // IN3 on the ULN2003 driver 1
#define motor1Pin4  36     // IN4 on the ULN2003 driver 1

int fullRotationSteps = 4075;

AccelStepper stepper1(HALFSTEP, motor1Pin1, motor1Pin3, motor1Pin2, motor1Pin4);


void setup() {
  Serial.begin(9600);

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(hallPinPower, OUTPUT);
  // initialize the hall effect sensor pin as an input:
  pinMode(hallPin, INPUT_PULLUP);

  stepper1.setAcceleration(ceil(fullRotationSteps/4));
  stepper1.setMaxSpeed(ceil(fullRotationSteps/4));
  stepper1.setSpeed(ceil(fullRotationSteps/4));
}

void loop(){

  switch (state){
    case 0: // calibration

    // Power on hall sensor
    digitalWrite(hallPinPower, HIGH);

    // read the state of the hall effect sensor:
    hallState = digitalRead(hallPin);

    if (hallState == LOW) {
      // turn LED on:
      digitalWrite(ledPin, HIGH);
      if(calibrateState == 1){
        calibrateStepIn = stepper1.currentPosition();
        calibrateState = 2;
      }
      //Serial.println(stepper1.currentPosition());
    }
    else {
      // turn LED off:
      digitalWrite(ledPin, LOW);
      //Serial.println(0);
      if(calibrateState == 0){
        calibrateState = 1;
      }else if(calibrateState == 2){
        calibrateStepOut = stepper1.currentPosition();

        Serial.println(calibrateStepIn);
        Serial.println(calibrateStepOut);

        stepper1.setCurrentPosition(ceil((calibrateStepOut - calibrateStepIn) / 2));
        calibrateState = 3;
        stepper1.moveTo(0);
        state = 1;
      }
    }

    stepper1.runSpeed();

    break;

    case 1:
      // Power off hall sensor
      digitalWrite(hallPinPower, LOW);

      stepper1.run();
      if (0 == stepper1.distanceToGo()){
        stepper1.moveTo(ceil(fullRotationSteps/2));
      }
    break;
    case 2:
    break;
  }
}
