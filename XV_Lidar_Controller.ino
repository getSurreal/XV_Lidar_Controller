/*
  XV Lidar Controller
  
  Copyright 2014 James LeRoy getSurreal
  https://github.com/getSurreal/XV_Lidar_Controller
  http://www.getsurreal.com/arduino/xv_lidar_controller
 
  See README for additional information 
*/

#include <TimerThree.h> // used for ultrasonic PWM motor control
#include <PID.h>

#define DEBUG_MOTOR_RPM false

const int motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

double rpm_setpoint = 300;  // desired RPM
double pwm_max = 1023;
double pwm_min = 600;
double pwm_val = pwm_min;  // start slow
double pwm_last;
double motor_rpm;

double Kp = 1.0;
double Ki = 0.5;
double Kd = 0.00;

PID myPID(&motor_rpm, &pwm_val, &rpm_setpoint,Kp,Ki,Kd, DIRECT);

int inByte = 0;  // incoming serial byte
unsigned char data_status = 0;
unsigned char data_4deg_index = 0;
unsigned char data_loop_index = 0;
unsigned char motor_rph_high_byte = 0; 
unsigned char motor_rph_low_byte = 0;
int motor_rph = 0;


void setup() {
  pinMode(motor_pwm_pin, OUTPUT); 
  Serial.begin(115200);  // USB serial
  Serial1.begin(115200);  // XV LDS data 

  Timer3.initialize(30); // set PWM frequency to 32.768kHz  
  Timer3.pwm(motor_pwm_pin, pwm_val); // replacement for analogWrite()

  myPID.SetOutputLimits(pwm_min,pwm_max);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // read byte from LIDAR and retransmit to USB
  if (Serial1.available() > 0) {
    inByte = Serial1.read();  // get incoming byte:
    Serial.print(inByte, BYTE);  // retransmit
    decodeData(inByte);
  }
  myPID.Compute();
  if (pwm_val != pwm_last) {
    Timer3.pwm(motor_pwm_pin, pwm_val);
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
