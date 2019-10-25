#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create motor and motor shield objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(4);

#define leftSensor A1
#define rightSensor A2
char message[20] = "";


int anglePin = A0;
int angle = 0;


//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

int motorSpeed = 0;
int motorDelta = 10;



void setup() {
  Serial.begin(9600);
  AFMS.begin();
  motor1->setSpeed(30);
  motor2->setSpeed(30);
  setPoint = 0;  //set point at zero degrees
  previousTime = millis();
}

void loop() {
  angle = analogRead(anglePin);  // read the input pin
//  Serial.println(angle);
  int l = analogRead(leftSensor);
  int r = analogRead(rightSensor);
  

  output = computePID(angle);
  motorSpeed += motorDelta * output;
  delay(100);

  sprintf(message, "  %d %d ", l, r);
  Serial.println(message);

  motor1->setSpeed(motorSpeed > 0 ? motorSpeed : -motorSpeed);
  motor2->setSpeed(motorSpeed > 0 ? motorSpeed : -motorSpeed);
  motor1->run(motorSpeed > 0 ? BACKWARD : FORWARD);
  motor2->run(motorSpeed > 0 ? BACKWARD : FORWARD);


//PID CONTROL LOOP
//  input = analogRead(A0);                //read from rotary encoder connected to A0
//  output = computePID(input);
//  delay(100);
//  analogWrite(3, output);                //control the motor based on PID value

}



 
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
