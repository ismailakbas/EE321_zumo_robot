// EE321 Project: Object Counter Zumo Robot
// Made by Hasan Alp Doyduk and İsmail Akbaş


// Libraries
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <Wire.h>  // Used for I2C communication
#include <LSM303.h>  // Compass library

// Pinouts
#define BUZZER_PIN 3
#define MZ80_PIN 6
#define LED_PIN 13

// Initial definitions
#define NUM_SENSORS 6
#define SPEED 200

// Initial definitions for calibration of LSM303 sensor
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60  // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ 0x1C  // CRA_REG_M value for magnetometer 220 Hz update rate
// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 5

// Create instances (like objects in classes)
ZumoMotors motors;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);
ZumoReflectanceSensorArray reflectanceSensors;

// Inıtial definitions 
unsigned int sensorValues[NUM_SENSORS];
unsigned int positionVal = 0;
unsigned int counter = 0; // Object counter
unsigned int isObject = 0;
unsigned int turner = 0;
unsigned int isLedOn = 0;
bool myExit = false;
float myheading;
float myrelative_heading;
float mytarget_heading;




void setup() {

  turner = 0;
  Serial.begin(9600);  // Serial communication
  Serial.println("Starting.");
  pinMode(MZ80_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); 
  pinMode(BUZZER_PIN, OUTPUT); 

  digitalWrite(LED_PIN, HIGH);
  isLedOn = 1;


  // --------------------- Start Of The Reflectance Calibration --------------------
  reflectanceSensors.init();
  unsigned long startTime = millis();
  while (millis() - startTime < 5000)
  {
    reflectanceSensors.calibrate();
  }
  // -------------------------- End Of The Calibration -----------------------------

  // At the end of the bottom sensor calibration blink LED 3 times 
  // 3 HIGH + 3 LOW (H -> LHLHL 5 times)
  isLedOn = 1;

  for (int i = 1; i < 6; i++) {
    if (isLedOn) {
      digitalWrite(LED_PIN, LOW);
      isLedOn = 0;
      delay(100);
    } 
    else {
      digitalWrite(LED_PIN, HIGH);
      isLedOn = 1;
      delay(100);
    }
  }


  // ----------------------- Start Of The Compass Calibration ---------------------
  CompassCalibration();
  // ---------------------------- End Of The Calibration --------------------------

  // At the end of the compass calibration blink LED 5 times
  // 5 HIGH + 5 LOW (H -> LHLHLHLHL 9 times)
  digitalWrite(LED_PIN, HIGH);
  isLedOn = 1;

  for (int i = 1; i < 8; i++) {
    if (isLedOn) {
      digitalWrite(LED_PIN, LOW);
      isLedOn = 0;
      delay(100);
    } 
    else {
      digitalWrite(LED_PIN, HIGH);
      isLedOn = 1;
      delay(100);
    }
  }
}




void loop() {

  // Target vector is the actual vector for turning its axis
  mytarget_heading = averageHeading();
  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  myheading = averageHeading();
  // This gives us the relative heading with respect to the target angle
  myrelative_heading = relativeHeading(myheading, mytarget_heading);

  while (myExit == false) {
    if (turner < 360) {
      motors.setLeftSpeed(100);
      motors.setRightSpeed(-100);
      delay(50);
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      delay(100);

      myheading = averageHeading();
      myrelative_heading = relativeHeading(myheading, mytarget_heading);
   
      turner = turner + 1 ;

      if (turner > 10) {
        if (abs(myrelative_heading) < 5) {
          Serial.println("Checking for exit.");
          turner = 5000;  // Dead
          exitFunc(counter);
          myExit = true;
          break;
        }
      }
    }
    positionVal = reflectanceSensors.readLine(sensorValues);

    if (!digitalRead(MZ80_PIN)) {
      digitalWrite(LED_PIN, HIGH);

      if (isObject == 0) {
        counter = counter + 1;
        isObject = 1;
        Serial.print("Number of Objects:");
        Serial.println(counter);
      }
    }
    else {
      isObject = 0;
      digitalWrite(LED_PIN, LOW);
    }
  }
}




void exitFunc(int counter) {

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW); 
  delay(1000);

  Serial.println("Exit turning.");
  delay(1000);

  isLedOn = 1;

  for (int i = 1; i < counter * 2 + 1; i++) {

    if (isLedOn) {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      isLedOn = 0;
      delay(1000);

    } 
    else {
      digitalWrite(LED_PIN, HIGH);
      isLedOn = 1;
      delay(100);
    }
  }
}




// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template<typename T> float heading(LSM303::vector<T> v) {
  float x_scaled = 2.0 * (float)(v.x - compass.m_min.x) / (compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled = 2.0 * (float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled) * 180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}


// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to) {
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}


// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading() {
  LSM303::vector<int32_t> avg = { 0, 0, 0 };

  for (int i = 0; i < 10; i++) {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}




unsigned int mostLeftSensor() {
  if (sensorValues[0] < 600)
    return 1;
  else
    return 0;
}

unsigned int leftSensor() {
  if (sensorValues[1] < 600)
    return 1;
  else
    return 0;
}

unsigned int midLeftSensor() {
  if (sensorValues[2] < 600)
    return 1;
  else
    return 0;
}

unsigned int midRightSensor() {
  if (sensorValues[3] < 600)
    return 1;
  else
    return 0;
}

unsigned int rightSensor() {
  if (sensorValues[4] < 600)
    return 1;
  else
    return 0;
}

unsigned int mostRightSensor() {
  if (sensorValues[5] < 600)
    return 1;
  else
    return 0;
}

void turnRight() {
  motors.setSpeeds(200, -200);
}

void go() {
  motors.setSpeeds(400, 400);
}




void CompassCalibration() {
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = { 32767, 32767, 32767 }, running_max = { -32767, -32767, -32767 };
  unsigned char index;

  // Serial.begin(9600);

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS);  // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);     // 220 Hz compass update rate

  // button.waitForButton();

  Serial.println("starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  for (index = 0; index < CALIBRATION_SAMPLES; index++) {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  //button.waitForButton();
}
