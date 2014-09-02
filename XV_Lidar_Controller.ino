/*
  XV Lidar Controller
 
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
  boolean relay;  // to retransmit the seiral data to the USB port
} 
xv_config;

const byte EEPROM_ID = 0x02;  // used to validate EEPROM initialized

double pwm_val;
double pwm_last;
double motor_rpm;

boolean debug_motor_rpm = false;  // controlled by ShowRPM and HideRPM commands

PID rpmPID(&motor_rpm, &pwm_val, &xv_config.rpm_setpoint, xv_config.Kp, xv_config.Ki, xv_config.Kd, DIRECT);

int inByte = 0;  // incoming serial byte
unsigned char data_status = 0;
unsigned char data_4deg_index = 0;
unsigned char data_loop_index = 0;
unsigned char motor_rph_high_byte = 0; 
unsigned char motor_rph_low_byte = 0;
int motor_rph = 0;

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
    if (xv_config.relay) {
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
    if (data_loop_index == 22) {
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
    if (data_loop_index == 22) {
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
    //      Serial.print(data_4deg_index, HEX);  
    //      Serial.print(": ");  
    break;
  case 2: // speed in RPH low byte
    motor_rph_low_byte = inByte;
    break;
  case 3: // speed in RPH high byte
    motor_rph_high_byte = inByte;
    motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
    motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
    if (debug_motor_rpm) {
      //      Serial.print("Motor RPH HEX: ");
      //      Serial.print(motor_rph_low_byte, HEX);   
      //      Serial.println(motor_rph_high_byte, HEX);   
      Serial.print("RPM: ");
      Serial.print(motor_rpm);
      Serial.print("  PWM: ");   
      Serial.println(pwm_val);
    }
    break;
  default: // others do checksum
    break;
  }  
}

void initEEPROM() {
  xv_config.id = 0x02;
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

  xv_config.rpm_setpoint = 300;  // desired RPM
  xv_config.pwm_max = 1023;
  xv_config.pwm_min = 600;
  xv_config.sample_time = 20;

  xv_config.Kp = 1.0;
  xv_config.Ki = 0.5;
  xv_config.Kd = 0.00;

  xv_config.motor_enable = true;
  xv_config.relay = true;
  EEPROM_writeAnything(0, xv_config);
}

void initSerialCommands() {
  sCmd.addCommand("help",       help);
  sCmd.addCommand("Help",       help);
  sCmd.addCommand("GetConfig",  getConfig);
  sCmd.addCommand("SaveConfig", saveConfig);
  sCmd.addCommand("ResetConfig",initEEPROM);

  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);

  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);
  sCmd.addCommand("RelayOff", relayOff);
  sCmd.addCommand("RelayOn",  relayOn);

  // XV Commands  
  sCmd.addCommand("GetPrompt",  getPrompt);
  sCmd.addCommand("GetVersion", getVersion);
}

void showRPM() {
  debug_motor_rpm = true;
  Serial.println("Showing RPM data");
}

void hideRPM() {
  debug_motor_rpm = false;
  Serial.println("Hiding RPM data");
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

void relayOff() {
  xv_config.relay = false;
  Serial.println("Lidar data disabled");
}

void relayOn() {
  xv_config.relay = true;
  Serial.println("Lidar data enabled");
}

void setRPM() {
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

void getPrompt() {
  Serial1.print(0x1B, BYTE);
}

void getVersion() {
  Serial1.print("GetVersion");
}

void help() {
  Serial.println("List of available commands (case sensitive)");
  Serial.println("  GetConfig");
  Serial.println("  SaveConfig");
  Serial.println("  ResetConfig");
  Serial.println("  SetRPM");
  Serial.println("  SetKp");
  Serial.println("  SetKi");
  Serial.println("  SetKp");
  Serial.println("  SetSampleTime");
  Serial.println("  ShowRPM");
  Serial.println("  HideRPM");
  Serial.println("  MotorOff");
  Serial.println("  MotorOn");
  Serial.println("  RelayOff");
  Serial.println("  RelayOn");
}

void getConfig() {
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
  Serial.println(xv_config.relay);
}

void saveConfig() {
  EEPROM_writeAnything(0, xv_config);
  Serial.println("Config Saved.");
}



