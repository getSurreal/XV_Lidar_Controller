/*
  XV Lidar Controller v1.2.0
 
 Copyright 2014 James LeRoy getSurreal
 https://github.com/getSurreal/XV_Lidar_Controller
 http://www.getsurreal.com/products/xv-lidar-controller
 
 See README for additional information 
 */

#include <TimerThree.h> // used for ultrasonic PWM motor control
#include <PID.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <SerialCommand.h>

struct EEPROM_Config {
  byte id;
  int motor_pwm_pin;  // pin connected to mosfet for motor speed control
  double rpm_setpoint;  // desired RPM (uses double to be compatible with PID library)
  double pwm_max;  // max analog value.  probably never needs to change from 1023
  double pwm_min;  // min analog pulse value to spin the motor
  int sample_time;  // how often to calculate the PID values

  // PID tuning values
  double Kp;
  double Ki;
  double Kd;

  boolean motor_enable;  // to spin the laser or not.  No data when not spinning
  boolean raw_data;  // to retransmit the seiral data to the USB port
} 
xv_config;

const byte EEPROM_ID = 0x03;  // used to validate EEPROM initialized

double pwm_val;
double pwm_last;
double motor_rpm;

boolean debug_motor_rpm = false;  // controlled by ShowRPM and HideRPM commands
boolean debug_dist = false;  //  controlled by ShowDist and HideDist commands
int debug_angle = 360;  // controlled by ShowAngle (0 - 359, 360 shows all)

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
    if (data_4deg_index == 0) {
      angle = 0;
      if (debug_dist) {
        Serial.print("Timestamp: ");
        Serial.println(millis());
      }
    }
    //Serial.print(int(data_4deg_index));
    //Serial.print(" ");
    break;
    
  case 2: // speed in RPH low byte
    motor_rph_low_byte = inByte;
    break;
    
  case 3: // speed in RPH high byte
    motor_rph_high_byte = inByte;
    motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
    motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
    if (debug_motor_rpm) {
      Serial.print("RPM: ");
      Serial.print(motor_rpm);
      Serial.print("  PWM: ");   
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
    if (debug_dist) {
      if (debug_angle == 360 or debug_angle == angle) {
        Serial.print(angle);
        Serial.print(": ");
        Serial.print(int(dist));
        Serial.print(" (");
        Serial.print(quality);
        Serial.println(")");
      }
    }
    angle++;    
    break;
    
  case 8:
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
    if (debug_dist) {
      if (debug_angle == 360 or debug_angle == angle) {
        Serial.print(angle);
        Serial.print(": ");
        Serial.print(int(dist));
        Serial.print(" (");
        Serial.print(quality);
        Serial.println(")");
      }
    }
    angle++;    
    break;
    
  case 12:
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
    if (debug_dist) {
      if (debug_angle == 360 or debug_angle == angle) {
        Serial.print(angle);
        Serial.print(": ");
        Serial.print(int(dist));
        Serial.print(" (");
        Serial.print(quality);
        Serial.println(")");
      }
    }
    angle++;    
    break;
    
  case 16:
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
    if (debug_dist) {
      if (debug_angle == 360 or debug_angle == angle) {
        Serial.print(angle);
        Serial.print(": ");
        Serial.print(int(dist));
        Serial.print(" (");
        Serial.print(quality);
        Serial.println(")");
      }
    }
    angle++;    
    break;

  default: // others do checksum
    break;
  }  
}

void initEEPROM() {
  xv_config.id = 0x03;
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

  xv_config.rpm_setpoint = 300;  // desired RPM
  xv_config.pwm_max = 1023;
  xv_config.pwm_min = 550;
  xv_config.sample_time = 20;
  xv_config.Kp = 1.0;
  xv_config.Ki = 0.5;
  xv_config.Kd = 0.00;

  xv_config.motor_enable = true;
  xv_config.raw_data = true;
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
  debug_motor_rpm = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println("Showing RPM data");
}

void hideRPM() {
  debug_motor_rpm = false;
  Serial.println("Hiding RPM data");
}

void showDist() {
  debug_dist = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println("Showing Distance data");
}

void hideDist() {
  debug_dist = false;
  Serial.println("Hiding Distance data");
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
    Serial.println("Incorrect syntax.  Example: ShowAngle 0 (0 - 359 or 360 for all)"); 
  }
  else {
    Serial.print("Showing Only Angle: ");
    Serial.println(sVal);
    debug_angle = sVal;
  }
}

void motorOff() {
  xv_config.motor_enable = false;
  Timer3.pwm(xv_config.motor_pwm_pin, 0);
  Serial.println("Motor off");
}

void motorOn() {
  xv_config.motor_enable = true;
  Timer3.pwm(xv_config.motor_pwm_pin, xv_config.pwm_min);
  Serial.println("Motor on");
}

void hideRaw() {
  xv_config.raw_data = false;
  Serial.println(" ");
  Serial.println("Raw lidar data disabled");
  Serial.println(" ");
}

void showRaw() {
  xv_config.raw_data = true;
  hideDist();
  hideRPM();
  Serial.println("Lidar data enabled");
}

void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
    if (sVal < 200) {
      sVal = 200;
      Serial.println("Setting to minimum 200");
    }
    if (sVal > 300) {
      sVal = 300;
      Serial.println("Setting to maximum 300");
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
    Serial.println("Incorrect syntax.  Example: SetRPM 300"); 
  }
  else {
    Serial.print("Setting RPM to: ");
    Serial.println(sVal);
    Serial.println(xv_config.rpm_setpoint);
    xv_config.rpm_setpoint = sVal;
    Serial.println(xv_config.rpm_setpoint);    
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
    Serial.println("Incorrect syntax.  Example: SetKp 1.0"); 
  }
  else {
    Serial.print("Setting Kp to: ");
    Serial.println(sVal);
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
    Serial.println("Incorrect syntax.  Example: SetKi 0.5"); 
  }
  else {
    Serial.print("Setting Ki to: ");
    Serial.println(sVal);    
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
    Serial.println("Incorrect syntax.  Example: SetKd 0.001"); 
  }
  else {
    Serial.print("Setting Kd to: ");
    Serial.println(sVal);    
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
    Serial.println("Incorrect syntax.  Example: SetSampleTime 20"); 
  }
  else {
    Serial.print("Setting Sample time to: ");
    Serial.println(sVal);    
    xv_config.sample_time = sVal;
    rpmPID.SetSampleTime(xv_config.sample_time);
  }
}

void help() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println("List of available commands (case sensitive)");
  Serial.println("  ShowConfig    - Show the running configuration");
  Serial.println("  SaveConfig    - Save the running configuration to EEPROM");
  Serial.println("  ResetConfig   - Restore the original configuration");
  Serial.println("  SetRPM        - Set the desired rotation speed (min: 200, max: 300)");
  Serial.println("  SetKp         - Set the proportional gain");
  Serial.println("  SetKi         - Set the integral gain");
  Serial.println("  SetKd         - Set the derivative gain");
  Serial.println("  SetSampleTime - Set the frequency the PID is calculated (ms)");
  Serial.println("  ShowRPM       - Show the rotation speed");
  Serial.println("  HideRPM       - Hide the rotation speed");
  Serial.println("  ShowDist      - Show the distance data");
  Serial.println("  HideDist      - Hide the distance data");
  Serial.println("  ShowAngle     - Show distance data for a specific angle (0 - 359 or 360 for all)");
  Serial.println("  MotorOff      - Stop spinning the lidar");
  Serial.println("  MotorOn       - Enable spinning of the lidar");
  Serial.println("  HideRaw       - Stop outputting the raw data from the lidar");
  Serial.println("  ShowRaw       - Enable the output of the raw lidar data");
}

void showConfig() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.print("PWM pin: ");
  Serial.println(xv_config.motor_pwm_pin);

  Serial.print("Target RPM: ");
  Serial.println(xv_config.rpm_setpoint);

  Serial.print("Max PWM: ");
  Serial.println(xv_config.pwm_max);
  Serial.print("Min PWM: ");
  Serial.println(xv_config.pwm_min);

  Serial.print("PID Kp: ");
  Serial.println(xv_config.Kp);
  Serial.print("PID Ki: ");
  Serial.println(xv_config.Ki);
  Serial.print("PID Kd: ");
  Serial.println(xv_config.Kd);
  Serial.print("SampleTime: ");
  Serial.println(xv_config.sample_time);

  Serial.print("Motor Enable: ");
  Serial.println(xv_config.motor_enable);
  Serial.print("Relay: ");
  Serial.println(xv_config.raw_data);
}

void saveConfig() {
  EEPROM_writeAnything(0, xv_config);
  Serial.println("Config Saved.");
}

