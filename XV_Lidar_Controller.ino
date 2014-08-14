/*
  XV Lidar Controller
 
 Copyright 2014 James LeRoy getSurreal
 https://github.com/getSurreal/XV_Lidar_Controller
 http://www.getsurreal.com/arduino/xv_lidar_controller
 
 See README for additional information 
 */

#include <TimerThree.h> // used for ultrasonic PWM motor control
#include <PID.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <SerialCommand.h>

#define DEBUG_MOTOR_RPM false

struct EEPROM_Config
{
  byte id;
  int motor_pwm_pin;
  double rpm_setpoint;  // desired RPM
  double pwm_max;
  double pwm_min;

  double Kp;
  double Ki;
  double Kd;
} 
xv_config;

const byte EEPROM_ID = 0x99;  // used to validate EEPROM initialized

double pwm_val = xv_config.pwm_min;  // start slow
double pwm_last;
double motor_rpm;

PID myPID(&motor_rpm, &pwm_val, &xv_config.rpm_setpoint,xv_config.Kp,xv_config.Ki,xv_config.Kd, DIRECT);

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
  Timer3.pwm(xv_config.motor_pwm_pin, pwm_val); // replacement for analogWrite()

  myPID.SetOutputLimits(xv_config.pwm_min,xv_config.pwm_max);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
  
  initSerialCommands();
}

void loop() {
  sCmd.readSerial();  // check for incoming serial commands
  
  // read byte from LIDAR and retransmit to USB
  if (Serial1.available() > 0) {
    inByte = Serial1.read();  // get incoming byte:
    Serial.print(inByte, BYTE);  // retransmit
    decodeData(inByte);
  }
  myPID.Compute();
  if (pwm_val != pwm_last) {
    Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);
    pwm_last = pwm_val;
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
#if DEBUG_MOTOR_RPM
    Serial.print(motor_rph_low_byte, HEX);   
    Serial.println(motor_rph_high_byte, HEX);   
    Serial.print(motor_rpm);
    Serial.print("  ");   
    Serial.println(pwm_val);
#endif
    break;
  default: // others do checksum
    break;
  }  
}

void initEEPROM() {
  xv_config.id = 0x99;
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

  xv_config.rpm_setpoint = 300;  // desired RPM
  xv_config.pwm_max = 1023;
  xv_config.pwm_min = 600;

  xv_config.Kp = 1.0;
  xv_config.Ki = 0.5;
  xv_config.Kd = 0.00;
  EEPROM_writeAnything(0, xv_config);
}

void initSerialCommands() {
  sCmd.addCommand("MOTOROFF",    motorOff);
}

void motorOff() {
  myPID.SetMode(MANUAL);
  Timer3.pwm(xv_config.motor_pwm_pin, 0);
  pwm_last = 0;
}
