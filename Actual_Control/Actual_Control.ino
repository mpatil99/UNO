//IMU imports
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
/**************************************************************************//*    
Display sensor calibration status*
//**************************************************************************/
void displayCalStatus(void){  
  /* Get the four calibration values (0..3) */  
  /* Any sensor data reporting 0 should be ignored, */  
  /* 3 means 'fully calibrated" */  
  uint8_t system, gyro, accel, mag;  
  system = gyro = accel = mag = 0;  
  bno.getCalibration(&system, &gyro, &accel, &mag);  
  /* The data should be ignored until the system calibration is > 0 */  
  Serial.print("\t");  
  if (!system)  {    
    Serial.print("! ");  }  
    /* Display the individual values */  
    Serial.print("Sys:");  
    Serial.print(system, DEC);  
    Serial.print(" G:");  
    Serial.print(gyro, DEC);  
    Serial.print(" A:");  
    Serial.print(accel, DEC);  
    Serial.print(" M:");  
    Serial.println(mag, DEC);
}
bool isGyroCal(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    return gyro == 3;
}
/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}
int eeAddress = 0;
long bnoID;
bool foundCalib = false;

adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;


//Manu defined pins
#define PWM_PIN 6
#define DIR_PIN 2
unsigned long prev_time;
unsigned long currentTime;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(11, INPUT);
//  prev_time = 0;
//
//  //IMU setup
//  Serial.println("Orientation Sensor Test"); Serial.println("");
//    /* Initialise the sensor */
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//  bno.getSensor(&sensor);
//    if (bnoID != sensor.sensor_id)
//    {
//        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
//        delay(500);
//    }
//    else
//    {
//        Serial.println("\nFound Calibration for this sensor in EEPROM.");
//        eeAddress += sizeof(long);
//        EEPROM.get(eeAddress, calibrationData);
//
//        displaySensorOffsets(calibrationData);
//
//        Serial.println("\n\nRestoring Calibration data to the BNO055...");
//        bno.setSensorOffsets(calibrationData);
//
//        Serial.println("\n\nCalibration data loaded into BNO055");
//        foundCalib = true;
//    }
//
//    delay(1000);
//
//    /* Display some basic information on this sensor */
//    displaySensorDetails();
//
//    /* Optional: Display current status */
//    displaySensorStatus();
//
//   /* Crystal must be configured AFTER loading calibration data into BNO055. */
//    bno.setExtCrystalUse(true);
//
//    sensors_event_t event;
//    bno.getEvent(&event);
//    /* always recal the mag as It goes out of calibration very often */
//    if (foundCalib){
//        Serial.println("Move sensor slightly to calibrate magnetometers");
//        while (!isGyroCal())
//        {
//            bno.getEvent(&event);
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    else
//    {
//        Serial.println("Please Calibrate Sensor: ");
//        while (!isGyroCal())
//        {
//            bno.getEvent(&event);
//
//            Serial.print("X: ");
//            Serial.print(event.orientation.x, 4);
//            Serial.print("\tY: ");
//            Serial.print(event.orientation.y, 4);
//            Serial.print("\tZ: ");
//            Serial.print(event.orientation.z, 4);
//
//            /* Optional: Display calibration status */
//            displayCalStatus();
//
//            /* New line for the next sample */
//            Serial.println("");
//
//            /* Wait the specified delay before requesting new data */
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//
//    Serial.println("\nFully calibrated!");
//    Serial.println("--------------------------------");
//    Serial.println("Calibration Results: ");
//    adafruit_bno055_offsets_t newCalib;
//    bno.getSensorOffsets(newCalib);
//    displaySensorOffsets(newCalib);
//
//    Serial.println("\n\nStoring calibration data to EEPROM...");
//
//    eeAddress = 0;
//    bno.getSensor(&sensor);
//    bnoID = sensor.sensor_id;
//
//    EEPROM.put(eeAddress, bnoID);
//
//    eeAddress += sizeof(long);
//    EEPROM.put(eeAddress, newCalib);
//    Serial.println("Data stored to EEPROM.");
//
//    Serial.println("\n--------------------------------\n");
//    delay(500);


  
}
////
////unsigned long prevTime;
////unsigned long currentTime;
//unsigned long potDelay = 300;
//#define potSense A2
//int potVal = analogRead(potSense);
//int oldPotAbs = analogRead(potSense);
//int newPotAbs = 0;
////Read Pontentiometer
//void readPot(){
//  currentTime = millis();
//  if (currentTime >= prevTime + potDelay){
//    newPotAbs = analogRead(potSense);
////    Serial.print("New: ");
//    Serial.println(newPotAbs);
////    Serial.print("Subtraction: ");
////    Serial.println(oldPotAbs- newPotAbs);
//  }
//}

//Motor Control Methods
void killMotor(){
  digitalWrite(PWM_PIN, LOW);
}

void driveMotor( int dir, int pwm_signal) {
  Serial.println(dir);
  Serial.println(pwm_signal);
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwm_signal);
}

double desired_angle = 91.75;
int safetyFlag =0;
void loop() {
  safetyFlag = digitalRead(11);
  Serial.println(safetyFlag);
  if(safetyFlag == 0){
//    uint32_t cur_time = 0;
//    
//    float delta_t = (cur_time - prev_time) / 1000.0;
//  
//    // handle the case where this is the first time through the loop
//    if (prev_time == 0) {
//      delta_t = 0.01;
//    }
  
    //TODO: Check Values at some point
//    if (angle > 3000 || angle < -3000)     // If angle is not within +- 3 degrees, reset counter that waits for start
//    {
//      start_counter = 0;
//    }
  
    
  //  // put your main code here, to run repeatedly:
//    sensors_event_t event;
//    bno.getEvent(&event);
  //  /* Display the floating point data */
  //  Serial.print("X: ");
  //  Serial.print(event.orientation.x, 4);
  //  Serial.print("\tY: ");
  //  Serial.print(event.orientation.y, 4);
  //  Serial.print("\tZ: ");
  //  Serial.println(event.orientation.z, 4);
//    if (event.orientation.x == 0, event.orientation.y == 0, event.orientation.z == 0) {
//        Serial.print("FAILFAILFAILFAILFAILFAIL");
//        /*
//        *  Look for the sensor's unique ID at the beginning oF EEPROM.
//        *  This isn't foolproof, but it's better than nothing.
//        */
//        bno.getSensor(&sensor);
//        if (bnoID != sensor.sensor_id)
//        {
//            Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
//            delay(500);
//        }
//        else
//        {
//            Serial.println("\nFound Calibration for this sensor in EEPROM.");
//            eeAddress += sizeof(long);
//            EEPROM.get(eeAddress, calibrationData);
//    
//            displaySensorOffsets(calibrationData);
//    
//            Serial.println("\n\nRestoring Calibration data to the BNO055...");
//            bno.setSensorOffsets(calibrationData);
//    
//            Serial.println("\n\nCalibration data loaded into BNO055");
//            foundCalib = true;
//        }
//        delay(1000);
//      }
  
      /* Optional: Display calibration status */
//      displayCalStatus();
  
      /* Optional: Display sensor status (debug only) */
      //displaySensorStatus();
  
      /* New line for the next sample */
//      Serial.println("");
  
      /* Wait the specified delay before requesting new data */
//      delay(BNO055_SAMPLERATE_DELAY_MS);
//  
//      angle = event.orientation.y;
//  
//      double error_angle = angle - desired_angle;
//  
  
  
  //  if(event.orientation.y > 0){
  ////    driveMotor(LOW, 10);
  //    digitalWrite(DIR_PIN, LOW);
  //    analogWrite(PWM_PIN, 30);
  //  } else {
      digitalWrite(DIR_PIN, LOW);
      analogWrite(PWM_PIN, 100);
      Serial.print("run");
  //  }
     
  //  delay(2000);
//    digitalWrite(PWM_PIN, LOW);
//    driveMotor(HIGH, 20);
  //  delay(2000);
  //
  //
  //  analogWrite(PWM_PIN, 0);
  //  delay(1000);
  } else{
//    driveMotor(LOW, 0);
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 0);
    Serial.println("stop");
    killMotor();
  }


}
