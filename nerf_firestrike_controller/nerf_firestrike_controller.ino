/*
 *  Project     Nerf Gun Controller
 *  @author     John Evans & David Madison
 *  @link       github.com/burntcustard/ArduinoMotionController
 *  @license    GPLv3 - Copyright (c) 2017 David Madison (Original)
 *              GPLv3 - Copyright (c) 2018 John Evans (Modifications)
 *
 *  MPU/aiming code mostly by David Madison:
 *    http://www.partsnotincluded.com/altctrl/mccree-hammershot-programming/
 *    
 *  Modifications by John include automatic gyroscope calibration & (much) more.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>


///////////////////
// CONFIGURABLES //
///////////////////

// Debug Flags (DEBUG must be uncommented for any others be enabled)
//#define DEBUG
//#define DEBUG_GYRO_CALIBRATION
//#define DEBUG_ACCEL
//#define DEBUG_GYRO
//#define DEBUG_AIMING
//#define DEBUG_TOUCH

// Physical Buttons for Mouse Clicks (used w/ internal pull-up resistor)
#define TRIGGER_1 10
#define TRIGGER_2 9

// Capacitive Touch Sensitive Input Pins & Keyboard Keys
const uint8_t TouchPins[] = { 15, 16, 14 };  // Keyboard pins followed by any other(s)
const uint8_t TouchDebounce = 8;  // Higher makes detection slower but less flickery.
const int KeyCodes[] = { 'e', 'r' }; // Keycodes to press when input touched.

// Thumbstick/Joystick:
#define VRx A0  // X (should be on an analog pin)
#define VRy A1  // Y (should be on an analog pin)
#define SW 4    // Button
const int joystickThreshold = 50;
char* joystickKeys[] = { 'w','a','s','d' };
char swKey = ' ';

// General Aim Sensitivity
const float OverwatchSens = 4;  // User mouse sensitivity in Overwatch
const float AimSpeedFactor = 1.2; // Factor to multiply aimspeed by

// Tuning Options
const long    IMU_UpdateRate = 5;  // Time between each IMU (mouse) update in ms.
const uint8_t GyroCalibrationTime = 100; // Time in ms that gyro is calibrated for at startup.
const uint8_t FS_Sel = 1; // Gyroscope range select, 500°/s
const int16_t GyroZeroThreshold = 120 / (1 << FS_Sel); // Level below which to null inputs, adjusted for range
const int16_t Aim_ExponentialThreshold = 5; // Threshold before exponential aiming kicks in, in °/s
const float   Aim_ExponentFactor = 0.0; // Factor to multiply the exponent product by, scaled by sensitivity

// Aim drift compensation, WIP, may not work as intended, <±0.1 recommended
const float xyAimDrift[] = { 0.008, -0.01 };


//////////////////////
// Global Variables //
//////////////////////

// I2C address for the MPU:
const uint8_t MPU_addr = 0x68;
// Gyroscope ranges (in degrees per second):
const int16_t Gyro_FullScaleRange[4] = {250, 500, 1000, 2000};
long timestamp;
long lastUpdate;

// Gyro auto-calibration values
int16_t GyroCalX = 0;
int16_t GyroCalY = 0;
int16_t GyroCalZ = 0;

// Physical Buttons for Mouse Clicks (used w/ internal pull-up resistor)
boolean trigger1Active = false;
boolean trigger2Active = false;

// Capacitive Touch Sensitive Input Pins & Keyboard Keys
const uint8_t TouchPinsLength = sizeof(TouchPins)/sizeof(uint8_t);
const uint8_t KeyCodesLength = sizeof(KeyCodes)/sizeof(int);
int8_t touchesActive[TouchPinsLength];

// Thumbstick/Joystick:
int joystickCenterX = 0;
int joystickCenterY = 0;
int joystickX = 0;
int joystickY = 0;


///////////////////
// INITIAL SETUP //
///////////////////

void setup() {
  Mouse.begin();
  Keyboard.begin();

  #ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) {
      ; // Wait for serial port to connect
    }
  #endif
  
  // Set buttons to use internal pullup resistors (would
  // need changing if buttons are hooked up differently):
  pinMode(TRIGGER_1, INPUT_PULLUP);
  pinMode(TRIGGER_2, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  
  mpuStart();

  // Center joystick values:
  joystickCenterX = analogRead(VRx);
  joystickCenterY = analogRead(VRy);
}


///////////////
// MAIN LOOP //
///////////////

void loop() {
  timestamp = millis();  // Milliseconds since program began.

  // Handling motion sensing with as-consistently-as-possible timings:
  if (timestamp >= lastUpdate + IMU_UpdateRate) {
    lastUpdate = timestamp;
    handleIMU();
  }

  triggerInputs();  // Mouse 1 / mouse 2.
  keyboardInput();  // Touch inputs (mostly simulated keyboard press/releases).
  joystickInput();  // Thumbstick X/Y & button.
}


void mpuStart() {
  Wire.begin();
  TWBR = 12; // 400 kHz bus speed

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(1);     // use GyX oscillator
  Wire.endTransmission(false); // Keep bus active

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG registers
  Wire.write(FS_Sel << 3);  // Gyroscope scale
  Wire.endTransmission(true);

  calibrateGyro();
}


/**
 * Automatically figures out accurate gyro calibration values.
 * Device must be completely stationary while this is running.
 * Modification(s) to David's MPU/aiming code were made to
 * accomodate this, use ctr+f "auto-calibration" to find them.
 */
void calibrateGyro() {

  int16_t aX, aY, aZ, temperature, gX, gY, gZ;
  int16_t *IMU_Data[7] = {&aX, &aY, &aZ, &temperature, &gX, &gY, &gZ};
  const int calibrationAmount = GyroCalibrationTime / IMU_UpdateRate;
  
  for (int i = 0; i < calibrationAmount; i++) {
    
    // Get data from MPU (not using readMPU() because it has unnecessary stuff)
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, (uint8_t) 14, (uint8_t) true); // request a total of 14 registers
    for (int i = 0; i < 7; i++) {
      *IMU_Data[i] = Wire.read() << 8 | Wire.read(); // A-XYZ, TMP, and G-XYZ
    }

    // Subtract (because they're later added to correct gyro offset) values
    GyroCalX -= gX;
    GyroCalY -= gY;
    GyroCalZ -= gZ;
    
    #ifdef DEBUG_GYRO_CALIBRATION
      Serial.print(i); Serial.print(": ");
      Serial.print(gX); Serial.print(" ");
      Serial.print(gY); Serial.print(" ");
      Serial.println(gZ);
    #endif
    
    delay(IMU_UpdateRate);
  }
  
  GyroCalX = round(GyroCalX / calibrationAmount);
  GyroCalY = round(GyroCalY / calibrationAmount);
  GyroCalZ = round(GyroCalZ / calibrationAmount);
  
  #ifdef DEBUG_GYRO_CALIBRATION
    Serial.print("Calibrated values: ");
    Serial.print(GyroCalX); Serial.print(" ");
    Serial.print(GyroCalY); Serial.print(" ");
    Serial.println(GyroCalZ);
  #endif
}


void handleIMU() {
  
  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  if (touchesActive[TouchPinsLength - 1]) {
    aiming(gz, -gx);  // Might need to switch x/y/z
  }

  // Motion stuff goes here (try jumping with up accel and punch w/ forward accel?)
  //ultimate(ax, ay, az);
  //flashbang(gy, ax);
}


void readMPU(int16_t &aX, int16_t &aY, int16_t &aZ, int16_t &gX, int16_t &gY, int16_t &gZ) {

  int16_t MPU_temperature;
  int16_t *IMU_Data[7] = {&aX, &aY, &aZ, &MPU_temperature, &gX, &gY, &gZ};
  const int16_t GyroCalibration[3] = {
    // Not sure what the FS_Sel bit was for here but it was breaking the auto-calibration!
    GyroCalX, // / (1 << FS_Sel),
    GyroCalY, // / (1 << FS_Sel),
    GyroCalZ, // / (1 << FS_Sel),
  };

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_addr, (uint8_t) 14, (uint8_t) true); // request a total of 14 registers

  for (int i = 0; i < 7; i++) {
    *IMU_Data[i] = Wire.read() << 8 | Wire.read(); // A-XYZ, TMP, and G-XYZ
  }

  // Calibrate gyro axes and check for overflow
  for (int i = 4; i < 7; i++) {
    int32_t gyroTemp = (int32_t) * IMU_Data[i] + (int32_t) GyroCalibration[i - 4];

    if (gyroTemp > 32767) {
      *IMU_Data[i] = 32767;
    }
    else if (gyroTemp < -32768) {
      *IMU_Data[i] = -32768;
    }
    else {
      *IMU_Data[i] += GyroCalibration[i - 4];
      if (abs(*IMU_Data[i]) < GyroZeroThreshold) {
        *IMU_Data[i] = 0;
      }
    }
  }

  reorientMPU(aX, aY, aZ, gX, gY, gZ);  // Moved to after overflow checks for auto-calibration.

  #ifdef DEBUG_ACCEL
    Serial.print(aX);
    Serial.print('\t');
    Serial.print(aY);
    Serial.print('\t');
    Serial.print(aZ);
    Serial.print('\t');
    Serial.print(F("XYZ Acc"));
    Serial.print('\n');
  #endif
  
  #ifdef DEBUG_GYRO
    Serial.print(gX);
    Serial.print('\t');
    Serial.print(gY);
    Serial.print('\t');
    Serial.print(gZ);
    Serial.print('\t');
    Serial.print(F("XYZ Gy"));
    Serial.print('\n');
  #endif
}


void reorientMPU(int16_t &aX, int16_t &aY, int16_t &aZ, int16_t &gX, int16_t &gY, int16_t &gZ) {
  // Reorient inputs from the position of the MPU to the expected coordinate system.
  // McCree Hammershot: Twist board +90° about Y. Z = X, X = -Z, Y doesn't change.

  int16_t tempAxis;

  // Reorient accelerometer
  tempAxis = aZ;
  aZ = aX;
  aX = -tempAxis;

  // Reorient gyroscope
  tempAxis = gZ;
  gZ = gX;
  gX = -tempAxis;
}


/**
 * TODO: Test ticks per degree/aiming stuff with games other than Overwatch,
 *       and/or adjust for my default of 3.0 sensitivity within Overwatch.
 */
void aiming(int16_t gyX, int16_t gyY) {
  const float IMU_AngleChange = (float) Gyro_FullScaleRange[FS_Sel] / (1000.0 / (float) IMU_UpdateRate); // Convert °/s to degree change per time read
  const float IMU_Scale = IMU_AngleChange / 32768.0; // Convert angular change to int16_t scale
  const float OverwatchTPD = 54550.0 / 360.0; // Ticks per degree in Overwatch, at 1.0 sens.
  
  const float OverwatchConversion = (OverwatchTPD * AimSpeedFactor / OverwatchSens) * IMU_Scale; // 'Magic' variable representing the degree change in OW mouse ticks
  const float OW_ExponentialThreshold = (Aim_ExponentialThreshold / Gyro_FullScaleRange[FS_Sel]) * OverwatchConversion * 32768.0; // exponential aim threshold, in (overwatch ticks / max overwatch ticks per sample)
  const float Aim_ExponentFactorScale = Aim_ExponentFactor * (OverwatchSens / 4); // Set using 4.0 sensitivity(?)

  if (gyX == 0 && gyY == 0) {
    #ifdef DEBUG_AIMING
        Serial.println(F("Aim inputs 0"));
    #endif
    return; // No mouse movement
  }

  int16_t * xyInputs[2] = {&gyX, &gyY};
  float xyScaled[2];

  for (int i = 0; i < 2; i++) {
    if (*xyInputs[i] != 0) {
      xyScaled[i] = OverwatchConversion * (float) -*xyInputs[i]; // Flip gyro inputs to match mouse axis, then calculate

      // Exponent function. Linear baseline + (squared baseline * minimizing factor)
      if(abs(xyScaled[i]) >= OW_ExponentialThreshold){    
        float mouseSign = 1; // Faster than x/|x|
        if(xyScaled[i] < 0){
          mouseSign = -1; 
        }
        
        xyScaled[i] +=
          abs(xyScaled[i] - OW_ExponentialThreshold) * abs(xyScaled[i] - OW_ExponentialThreshold) *
          Aim_ExponentFactorScale *
          mouseSign;

        // Handle X/Y drift over time from incorrect sensor data or rounding errors
        if (mouseSign == 1) {
          xyScaled[i] *= (1 + xyAimDrift[i]);
        } else {
          xyScaled[i] *= (1 - xyAimDrift[i]);
        }
      }
    }
  }
  
  #ifdef DEBUG_AIMING
    Serial.print(xyScaled[0]);
    Serial.print('\t');
    Serial.print(xyScaled[1]);
    Serial.print('\t');
    Serial.print(F("XY Aiming"));
    Serial.print('\n');
  #endif

  sendMouse(xyScaled[0], xyScaled[1]);
}


void sendMouse(float mX, float mY){
  static float xyRemainder[2];

  float * flt_mouseXY[2] = {&mX, &mY};
  int32_t int_mouseXY[2] = {(int32_t) mX, (int32_t) mY};
    
  // Store floating point remainders and insert them if > 1
  for (int i = 0; i < 2; i++) {
    if (*flt_mouseXY[i] != 0) {
      float remainderTemp = *flt_mouseXY[i] - int_mouseXY[i];
      
      if (abs(remainderTemp) >= 0.05) {
        xyRemainder[i] += remainderTemp;
      }

      if (xyRemainder[i] > 1) {
        int_mouseXY[i] += 1;
        xyRemainder[i]--;
      }
      else if (xyRemainder[i] < -1) {
        int_mouseXY[i] -= 1;
        xyRemainder[i]++;
      }
    }
  }

  // Calculate +/- signs beforehand to save cycles
  int8_t mouseSign[2] = {1, 1};
  for(int i = 0; i < 2; i++){
    // Single if statement faster than x/|x|
    if(int_mouseXY[i] < 0){
      mouseSign[i] = -1;
    }
  }

  // Send mouse outputs in iterations, to avoid the int8_t limit of the Mouse.move function
  while(int_mouseXY[0]!= 0 || int_mouseXY[1] != 0){
    int8_t mouseUpdate[2];
    
    for(int i = 0; i < 2; i++){
      if(abs(int_mouseXY[i]) >= 127){
        mouseUpdate[i] = 127 * mouseSign[i];
        int_mouseXY[i] = (abs(int_mouseXY[i]) - 127) * (int32_t) mouseSign[i];
      }
      else if(abs(int_mouseXY[i]) > 0){
        mouseUpdate[i] = (int8_t) int_mouseXY[i];
        int_mouseXY[i] = 0;
      }
      else{
        mouseUpdate[i] = 0;
      }
    }
    Mouse.move(mouseUpdate[0], mouseUpdate[1]);
  }
}


void triggerInputs() {
  boolean t1 = digitalRead(TRIGGER_1);
  boolean t2 = digitalRead(TRIGGER_2);

  if (!trigger1Active && t1 == HIGH) {
    Mouse.press(MOUSE_LEFT);
    trigger1Active = true;
  } else if (trigger1Active && t1 == LOW) {
    Mouse.release(MOUSE_LEFT);
    trigger1Active = false;
  }
  if (!trigger2Active && t2 == HIGH) {
    Mouse.press(MOUSE_RIGHT);
    trigger2Active = true;
  } else if (trigger2Active && t2 == LOW) {
    Mouse.release(MOUSE_RIGHT);
    trigger2Active = false;
  }
}


#ifdef DEBUG_TOUCH
  void printKeyInfo(int i) {
    if (i < KeyCodesLength) {
      Serial.print((char) KeyCodes[i]); // May be issues with Arduino key codes that are >1 char.
    } else {
      Serial.print("NO_KEYCODE");
    }
    Serial.print(" (pin "); Serial.print((int) TouchPins[i]); Serial.print(")");
  }
#endif


void keyboardInput() {

  for (int i = 0; i < TouchPinsLength; i++) {
    boolean currentTouchPressed = !digitalRead(TouchPins[i]);

    if (currentTouchPressed) {
      if (touchesActive[i] == 0) {
        if (i < KeyCodesLength) {
          Keyboard.press(KeyCodes[i]);
        }
        #ifdef DEBUG_TOUCH
          printKeyInfo(i); Serial.println(" pressed");
        #endif
      }
      touchesActive[i] = TouchDebounce;
    } else {
      if (touchesActive[i] > 0) {
        touchesActive[i]--;
      }
      if (touchesActive[i] == 1) {
        if (i < KeyCodesLength) {
          Keyboard.release(KeyCodes[i]);
        }
        #ifdef DEBUG_TOUCH
          printKeyInfo(i); Serial.println(" released");
        #endif
      }
    }
  }
}


void joystickInput() {  
  // X and Y are flipped because joystick is mounted sideways
  joystickY =  (analogRead(VRx) - joystickCenterX);
  joystickX = -(analogRead(VRy) - joystickCenterY);
  
  if (joystickY > joystickThreshold) {
    Keyboard.press(joystickKeys[0]);
  } else {
    Keyboard.release(joystickKeys[0]);
  }
  if (joystickX < -joystickThreshold) {
    Keyboard.press(joystickKeys[1]);
  } else {
    Keyboard.release(joystickKeys[1]);
  }
  if (joystickY < -joystickThreshold) {
    Keyboard.press(joystickKeys[2]);
  } else {
    Keyboard.release(joystickKeys[2]);
  }
  if (joystickX > joystickThreshold) {
    Keyboard.press(joystickKeys[3]);
  } else {
    Keyboard.release(joystickKeys[3]);
  }

  // Diagonals need to be a bit more sensitive because diagonal distance is further?
  // Not sure if this actually works >.<
  const float normalizedThreshold = 0.7071 * joystickThreshold;
  if (joystickY > normalizedThreshold &&
      joystickX < -normalizedThreshold) {
    Keyboard.press(joystickKeys[0]);
    Keyboard.press(joystickKeys[1]);
  }
  if (joystickY < -normalizedThreshold &&
      joystickX < -normalizedThreshold) {
    Keyboard.press(joystickKeys[1]);
    Keyboard.press(joystickKeys[2]);
  }
  if (joystickY < -normalizedThreshold &&
      joystickX > normalizedThreshold) {
    Keyboard.press(joystickKeys[2]);
    Keyboard.press(joystickKeys[3]);
  }
  if (joystickY > normalizedThreshold &&
      joystickX > normalizedThreshold) {
    Keyboard.press(joystickKeys[3]);
    Keyboard.press(joystickKeys[0]);
  }

  if (digitalRead(SW) == LOW) {
    Keyboard.press(swKey);
  } else {
    Keyboard.release(swKey);
  }
}

