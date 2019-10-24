#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create motor and motor shield objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor = AFMS.getMotor(3);
Adafruit_DCMotor *motor1 = AFMS.getMotor(4);
//Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

#define leftSensor A1
#define rightSensor A2
char message[20] = "";


int analogPin = A0;
int val = 0;


//PID constants
double kp = 2
double ki = 5
double kd = 1
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;



void setup() {
  Serial.begin(9600);
  AFMS.begin();
  motor->setSpeed(30);
  motor1->setSpeed(30);
  setPoint = 0;                     //set point at zero degrees

}

void loop() {
  vroom = 0;
  val = analogRead(analogPin);  // read the input pin
//  Serial.println(val);
  int l = analogRead(leftSensor);
  int r = analogRead(rightSensor);
  sprintf(message, "  %d %d ", l, r);
  Serial.println(message);

  motor->run(BACKWARD);
  motor1->run(BACKWARD);
//  motorSpeed = 500 - val
//  if (val > 500) {
//    motor->run(BACKWARD);
//  }
//  else {
//    motor->run(FORWARD);
//  }


//PID CONTROL LOOP
//  input = analogRead(A0);                //read from rotary encoder connected to A0
//  output = computePID(input);
//  delay(100);
//  analogWrite(3, output);                //control the motor based on PID value

}

 
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
