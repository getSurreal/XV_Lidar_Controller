/*
  XV Lidar Controller v1.2.2
 
 Copyright 2014 James LeRoy getSurreal
 https://github.com/getSurreal/XV_Lidar_Controller
 http://www.getsurreal.com/products/xv-lidar-controller
 
 See README for additional information 
 
 The F() macro in the Serial statements tells the compiler to keep your strings in PROGMEM
 */

#include <TimerThree.h> // used for ultrasonic PWM motor control
#include <PID.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <SerialCommand.h>

struct EEPROM_Config {
  byte id;
  char version[6];
  int motor_pwm_pin;    // pin connected to mosfet for motor speed control
  double rpm_setpoint;  // desired RPM (uses double to be compatible with PID library)
  double rpm_min;
  double rpm_max;
  double pwm_max;       // max analog value.  probably never needs to change from 1023
  double pwm_min;       // min analog pulse value to spin the motor
  int sample_time;      // how often to calculate the PID values

  // PID tuning values
  double Kp;
  double Ki;
  double Kd;

  boolean motor_enable;  // to spin the laser or not.  No data when not spinning
  boolean raw_data;  // to retransmit the seiral data to the USB port
  boolean show_dist;  //  controlled by ShowDist and HideDist commands
  boolean show_rpm;  // controlled by ShowRPM and HideRPM commands
  unsigned int show_angle;  // controlled by ShowAngle (0 - 359, 360 shows all)
} 
xv_config;

const byte EEPROM_ID = 0x05;  // used to validate EEPROM initialized

double pwm_val = 500;  // start with ~50% power
double pwm_last;
double motor_rpm;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;  
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillis = millis();

PID rpmPID(&motor_rpm, &pwm_val, &xv_config.rpm_setpoint, xv_config.Kp, xv_config.Ki, xv_config.Kd, DIRECT);

uint8_t inByte = 0;  // incoming serial byte
uint16_t data_status = 0;
uint16_t data_4deg_index = 0;
uint16_t data_loop_index = 0;
uint8_t motor_rph_high_byte = 0; 
uint8_t motor_rph_low_byte = 0;
uint8_t data0, data2;
uint16_t dist, quality;
uint16_t motor_rph = 0;
uint16_t angle;

SerialCommand sCmd;

const int ledPin = 11;
boolean ledState = LOW;

void setup() {
  EEPROM_readAnything(0, xv_config);
  if( xv_config.id != EEPROM_ID) { // verify EEPROM values have been initialized
    initEEPROM();
  }
  pinMode(xv_config.motor_pwm_pin, OUTPUT); 
  Serial.begin(115200);  // USB serial
  Serial1.begin(115200);  // XV LDS data 

  Timer3.initialize(30); // set PWM frequency to 32.768kHz  

  rpmPID.SetOutputLimits(xv_config.pwm_min,xv_config.pwm_max);
  rpmPID.SetSampleTime(xv_config.sample_time);
  rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  rpmPID.SetMode(AUTOMATIC);

  initSerialCommands();
  pinMode(ledPin, OUTPUT);
}

void loop() {

  sCmd.readSerial();  // check for incoming serial commands

  // read byte from LIDAR and relay to USB
  if (Serial1.available() > 0) {
    inByte = Serial1.read();  // get incoming byte:
    if (xv_config.raw_data) {
      Serial.print(inByte, BYTE);  // relay
    }
    decodeData(inByte);
  }

  if (xv_config.motor_enable) {  
    rpmPID.Compute();
    if (pwm_val != pwm_last) {
      Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);  // replacement for analogWrite()
      pwm_last = pwm_val;
    }
    motorCheck();
  }
}

void decodeData(unsigned char inByte) {
  switch (data_status) {
  case 0: // no header
    if (inByte == 0xFA) {
      data_status = 1;
      data_loop_index = 1;
    }
    break;

  case 1: // find 2nd FA
    if (data_loop_index == 22) { // Theres 22 bytes in each packet. Time to start over
      if (inByte == 0xFA) {
        data_status = 2;
        data_loop_index = 1;
      } 
      else { // if not FA search again
        data_status = 0;
      }
    }
    else {
      data_loop_index++;
    }
    break;

  case 2: // read data out
    if (data_loop_index == 22) { // Theres 22 bytes in each packet. Time to start over
      if (inByte == 0xFA) {
        data_loop_index = 1;
      } 
      else { // if not FA search again
        data_status = 0;
      }
    }
    else {
      readData(inByte);
      data_loop_index++;
    }
    break;
  }

}
void readData(unsigned char inByte) {
  switch (data_loop_index) {
  case 1: // 4 degree index
    data_4deg_index = inByte - 0xA0;
    angle = data_4deg_index * 4;  // 1st angle in the set of 4
    if (angle == 0) {
      if (ledState) {
        ledState = LOW;
      } 
      else {
        ledState = HIGH;
      }
      digitalWrite(ledPin, ledState);
      if (xv_config.show_dist) {
        curMillis = millis();
        Serial.print(F("Time Interval: "));
        Serial.println(curMillis - lastMillis);
        lastMillis = curMillis;
      }
    }
    //Serial.print(int(data_4deg_index));
    //Serial.println(F(" "));
    break;

  case 2: // speed in RPH low byte
    motor_rph_low_byte = inByte;
    break;

  case 3: // speed in RPH high byte
    motor_rph_high_byte = inByte;
    motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
    motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
    if (xv_config.show_rpm and angle == 0) {
      Serial.print(F("RPM: "));
      Serial.print(motor_rpm);
      Serial.print(F("  PWM: "));   
      Serial.println(pwm_val);
    }
    break;

  case 4:
    data0 = inByte; // first half of distance data
    break;

  case 5:
    if ((inByte & 0x80) >> 7) {  // check for Invalid Flag
      dist = 0;
    } 
    else {
      dist =  data0 | (( inByte & 0x3F) << 8);
    }
    break;

  case 6:
    data2 = inByte; // first half of quality data
    break;

  case 7:
    quality = (inByte << 8) | data2; 
    if (xv_config.show_dist) {
      if (xv_config.show_angle == 360 or xv_config.show_angle == angle) {
        Serial.print(angle);
        Serial.print(F(": "));
        Serial.print(int(dist));
        Serial.print(F(" ("));
        Serial.print(quality);
        Serial.println(F(")"));
      }
    }   
    break;

  case 8:
    angle = data_4deg_index * 4 + 1; // 2nd angle in the set
    data0 = inByte;
    break;

  case 9:
    if ((inByte & 0x80) >> 7) {  // check for Invalid Flag
      dist = 0;
    } 
    else {
      dist =  data0 | (( inByte & 0x3F) << 8);
    }
    break;

  case 10:
    data2 = inByte; // first half of quality data
    break;

  case 11:
    quality = (inByte << 8) | data2; 
    if (xv_config.show_dist) {
      if (xv_config.show_angle == 360 or xv_config.show_angle == angle) {
        Serial.print(angle);
        Serial.print(F(": "));
        Serial.print(int(dist));
        Serial.print(F(" ("));
        Serial.print(quality);
        Serial.println(F(")"));
      }
    }
    break;

  case 12:
    angle = data_4deg_index * 4 + 2; // 3rd angle in the set
    data0 = inByte;
    break;

  case 13:
    if ((inByte & 0x80) >> 7) {  // check for Invalid Flag
      dist = 0;
    } 
    else {
      dist =  data0 | (( inByte & 0x3F) << 8);
    }
    break;

  case 14:
    data2 = inByte; // first half of quality data
    break;

  case 15:
    quality = (inByte << 8) | data2; 
    if (xv_config.show_dist) {
      if (xv_config.show_angle == 360 or xv_config.show_angle == angle) {
        Serial.print(angle);
        Serial.print(F(": "));
        Serial.print(int(dist));
        Serial.print(F(" ("));
        Serial.print(quality);
        Serial.println(F(")"));
      }
    }
    break;

  case 16:
    angle = data_4deg_index * 4 + 3;  // 4th angle in the set
    data0 = inByte;
    break;

  case 17:
    if ((inByte & 0x80) >> 7) {  // check for Invalid Flag
      dist = 0;
    } 
    else {
      dist =  data0 | (( inByte & 0x3F) << 8);
    }
    break;

  case 18:
    data2 = inByte; // first half of quality data
    break;

  case 19:
    quality = (inByte << 8) | data2; 
    if (xv_config.show_dist) {
      if (xv_config.show_angle == 360 or xv_config.show_angle == angle) {
        Serial.print(angle);
        Serial.print(F(": "));
        Serial.print(int(dist));
        Serial.print(F(" ("));
        Serial.print(quality);
        Serial.println(F(")"));
      }
    }
    break;

  default: // others do checksum
    break;
  }  
}

void initEEPROM() {
  xv_config.id = 0x05;
  strcpy(xv_config.version, "1.2.2");
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

  xv_config.rpm_setpoint = 300;  // desired RPM
  xv_config.rpm_min = 200;
  xv_config.rpm_max = 300;
  xv_config.pwm_min = 100;
  xv_config.pwm_max = 1023;
  xv_config.sample_time = 20;
  xv_config.Kp = 2.0;
  xv_config.Ki = 1.0;
  xv_config.Kd = 0.0;

  xv_config.motor_enable = true;
  xv_config.raw_data = true;
  xv_config.show_dist = false;
  xv_config.show_rpm = false;
  xv_config.show_angle = 360;

  EEPROM_writeAnything(0, xv_config);
}

void initSerialCommands() {
  sCmd.addCommand("help",       help);
  sCmd.addCommand("Help",       help);
  sCmd.addCommand("ShowConfig",  showConfig);
  sCmd.addCommand("SaveConfig", saveConfig);
  sCmd.addCommand("ResetConfig",initEEPROM);

  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);

  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("ShowDist",  showDist);
  sCmd.addCommand("HideDist",  hideDist);
  sCmd.addCommand("ShowAngle",  showAngle);
  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowRaw",  showRaw);
}

void showRPM() {
  xv_config.show_rpm = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.print(F("Showing RPM data"));
  Serial.println(F(" "));
}

void hideRPM() {
  xv_config.show_rpm = false;
  Serial.println(F(" "));
  Serial.print(F("Hiding RPM data"));
  Serial.println(F(" "));
}

void showDist() {
  xv_config.show_dist = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.print(F("Showing Distance data"));
  Serial.println(F(" "));
}

void hideDist() {
  xv_config.show_dist = false;
  Serial.println(F(" "));
  Serial.print(F("Hiding Distance data"));
  Serial.println(F(" "));
}

void showAngle() {
  showDist(); 
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);    // Converts a char string to a int
    if (sVal < 0 or sVal > 360) {
      syntax_error = true;
    }
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: ShowAngle 0 (0 - 359 or 360 for all)")); 
    Serial.println(F(" "));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Showing Only Angle: "));
    Serial.println(sVal);
    Serial.println(F(" "));
    xv_config.show_angle = sVal;
  }
}

void motorOff() {
  xv_config.motor_enable = false;
  Timer3.pwm(xv_config.motor_pwm_pin, 0);
  Serial.println(F(" "));
  Serial.print(F("Motor off"));
  Serial.println(F(" "));
}

void motorOn() {
  xv_config.motor_enable = true;
  Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);
  rpm_err = 0;  // reset rpm error
  Serial.println(F(" "));
  Serial.print(F("Motor on"));
  Serial.println(F(" "));
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval){
    if ((motor_rpm < xv_config.rpm_min or motor_rpm > xv_config.rpm_max) and pwm_val > 1000) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
      motorOff(); 
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    motor_check_timer = millis();
  }
}

void hideRaw() {
  xv_config.raw_data = false;
  Serial.println(F(" "));
  Serial.print(F("Raw lidar data disabled"));
  Serial.println(F(" "));
}

void showRaw() {
  xv_config.raw_data = true;
  hideDist();
  hideRPM();
  Serial.println(F(" "));
  Serial.print(F("Lidar data enabled"));
  Serial.println(F(" "));
}

void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
    if (sVal < xv_config.rpm_min) {
      sVal = xv_config.rpm_min;
      Serial.println(F(" "));
      Serial.print(F("RPM too low. Setting to minimum "));
      Serial.print(xv_config.rpm_min);
      Serial.println(F(" "));
    }
    if (sVal > xv_config.rpm_max) {
      sVal = xv_config.rpm_max;
      Serial.println(F(" "));
      Serial.print(F("RPM too high. Setting to maximum "));
      Serial.print(xv_config.rpm_max);
      Serial.println(F(" "));
    }
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: SetRPM 300")); 
    Serial.println(F(" "));
  }
  else {
    xv_config.rpm_setpoint = sVal;
    Serial.println(F(" "));
    Serial.print(F("New RPM setpoint: "));
    Serial.println(sVal);
    Serial.print(F("Old RPM setpoint"));
    Serial.println(xv_config.rpm_setpoint);
    Serial.println(F(" "));
  }
}

void setKp() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: SetKp 1.0")); 
    Serial.println(F(" "));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kp to: "));
    Serial.println(sVal);
    Serial.println(F(" "));
    xv_config.Kp = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setKi() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: SetKi 0.5")); 
    Serial.println(F(" "));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Ki to: "));
    Serial.println(sVal);    
    Serial.println(F(" "));
    xv_config.Ki = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setKd() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: SetKd 0.001")); 
    Serial.println(F(" "));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kd to: "));
    Serial.println(sVal);    
    Serial.println(F(" "));
    xv_config.Kd = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setSampleTime() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);    // Converts a char string to an integer
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.print(F("Incorrect syntax.  Example: SetSampleTime 20")); 
    Serial.println(F(" "));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Sample time to: "));
    Serial.println(sVal);    
    Serial.println(F(" "));
    xv_config.sample_time = sVal;
    rpmPID.SetSampleTime(xv_config.sample_time);
  }
}

void help() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(xv_config.version);
  Serial.print(F("GetSurreal.com"));

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.println(F("List of available commands (case sensitive)"));
  Serial.println(F("  ShowConfig    - Show the running configuration"));
  Serial.println(F("  SaveConfig    - Save the running configuration to EEPROM"));
  Serial.println(F("  ResetConfig   - Restore the original configuration"));
  Serial.println(F("  SetRPM        - Set the desired rotation speed (min: 200, max: 300)"));
  Serial.println(F("  SetKp         - Set the proportional gain"));
  Serial.println(F("  SetKi         - Set the integral gain"));
  Serial.println(F("  SetKd         - Set the derivative gain"));
  Serial.println(F("  SetSampleTime - Set the frequency the PID is calculated (ms)"));
  Serial.println(F("  ShowRPM       - Show the rotation speed"));
  Serial.println(F("  HideRPM       - Hide the rotation speed"));
  Serial.println(F("  ShowDist      - Show the distance data"));
  Serial.println(F("  HideDist      - Hide the distance data"));
  Serial.println(F("  ShowAngle     - Show distance data for a specific angle (0 - 359 or 360 for all)"));
  Serial.println(F("  MotorOff      - Stop spinning the lidar"));
  Serial.println(F("  MotorOn       - Enable spinning of the lidar"));
  Serial.println(F("  HideRaw       - Stop outputting the raw data from the lidar"));
  Serial.println(F("  ShowRaw       - Enable the output of the raw lidar data"));

  Serial.println(F(" "));
  Serial.println(F(" "));
}

void showConfig() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(xv_config.version);
  Serial.print(F("GetSurreal.com"));

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("PWM pin: "));
  Serial.println(xv_config.motor_pwm_pin);

  Serial.print(F("Target RPM: "));
  Serial.println(xv_config.rpm_setpoint);

  Serial.print(F("Max PWM: "));
  Serial.println(xv_config.pwm_max);
  Serial.print(F("Min PWM: "));
  Serial.println(xv_config.pwm_min);

  Serial.print(F("PID Kp: "));
  Serial.println(xv_config.Kp);
  Serial.print(F("PID Ki: "));
  Serial.println(xv_config.Ki);
  Serial.print(F("PID Kd: "));
  Serial.println(xv_config.Kd);
  Serial.print(F("SampleTime: "));
  Serial.println(xv_config.sample_time);

  Serial.print(F("Motor Enable: "));
  Serial.println(xv_config.motor_enable);
  Serial.print(F("Show Raw Data: "));
  Serial.println(xv_config.raw_data);
  Serial.print(F("Show Dist Data: "));
  Serial.println(xv_config.show_dist);
  Serial.print(F("Show RPM Data: "));
  Serial.println(xv_config.show_rpm);
  Serial.print(F("Show Angle: "));
  Serial.println(xv_config.show_angle);

  Serial.println(F(" "));
  Serial.println(F(" "));

}

void saveConfig() {
  EEPROM_writeAnything(0, xv_config);
  Serial.print(F("Config Saved."));
}



