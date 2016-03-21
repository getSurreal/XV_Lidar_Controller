/*
  XV Lidar Controller v1.3.0

  Copyright 2014-2016 James LeRoy getSurreal
  https://github.com/getSurreal/XV_Lidar_Controller
  http://www.getsurreal.com/products/xv-lidar-controller

  Contributions by:
    Doug Hilton mailto: six.speed (at) yahoo (dot) com

  See README for additional information

  The F() macro in the Serial statements tells the compiler to keep your strings in PROGMEM
*/

#include "libraries/TimerThree/TimerThree.h" // used for ultrasonic PWM motor control
#include "libraries/PID/PID.h"
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "libraries/SerialCommand/SerialCommand.h"

const int N_ANGLES = 360;                                       // # of angles (0..359)
const int SHOW_ALL_ANGLES = N_ANGLES;                           // value means 'display all angle data, 0..359'

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
  boolean show_interval;  // true = show time interval, once per revolution, at angle=0
  boolean show_errors;  // Show CRC, signal strength and invalid data errors
  boolean aryAngles[N_ANGLES]; // array of angles to display
}
xv_config;

const byte EEPROM_ID = 0x06;  // used to validate EEPROM initialized

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

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int OFFSET_DATA_DISTANCE_LSB = 0;
const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

int Packet[PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                          // index into 'Packet' array
const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"



/* REF: https://github.com/Xevel/NXV11/wiki
  The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
  It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
  only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
  data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
  interrupted by the supports of the cover) .
*/
const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
/*
  The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
  This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
  size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
  reflections (glass... ).
*/
const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = eState_Find_COMMAND;
PID rpmPID(&motor_rpm, &pwm_val, &xv_config.rpm_setpoint, xv_config.Kp, xv_config.Ki, xv_config.Kd, DIRECT);

uint8_t inByte = 0;  // incoming serial byte
uint8_t motor_rph_high_byte = 0;
uint8_t motor_rph_low_byte = 0;
uint16_t aryDist[N_DATA_QUADS] = {0, 0, 0, 0};   // thre are (4) distances, one for each data quad
// so the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[N_DATA_QUADS] = {0, 0, 0, 0}; // same with 'quality'
uint16_t motor_rph = 0;
uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

SerialCommand sCmd;

boolean ledState = LOW;

#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)  // if Teensy 2.0
const int ledPin = 11;

#elif defined(__AVR_ATmega32U4__)  // if Leonardo (no LED for Pro Micro)
const int ledPin = 13;

#elif defined(__MK20DX256__)  // if Teensy 3.1
const int ledPin = 13;
#endif


// initialization (before 'loop')
void setup() {
  EEPROM_readAnything(0, xv_config);
  if ( xv_config.id != EEPROM_ID) { // verify EEPROM values have been initialized
    initEEPROM();
  }
  pinMode(xv_config.motor_pwm_pin, OUTPUT);
  Serial.begin(115200);                            // USB serial
#if defined(__AVR_ATmega32U4__)
  Serial1.begin(115200);                           // XV LDS data

#elif defined(__MK20DX256__) // if Teensy 3.1
  Serial1.begin(115200);  // XV LDS data
#endif

  Timer3.initialize(30);                           // set PWM frequency to 32.768kHz

  rpmPID.SetOutputLimits(xv_config.pwm_min, xv_config.pwm_max);
  rpmPID.SetSampleTime(xv_config.sample_time);
  rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  rpmPID.SetMode(AUTOMATIC);

  initSerialCommands();
  pinMode(ledPin, OUTPUT);

  eState = eState_Find_COMMAND;
  for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // Initialize
    Packet[ixPacket] = 0;
  ixPacket = 0;
}

void loop() {
  byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

  sCmd.readSerial();  // check for incoming serial commands
  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    if (xv_config.raw_data)
      Serial.write(inByte);                 // relay

    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          processSpeed();                             // process the speed
          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }
          if (xv_config.show_dist) {                           // the 'ShowDistance' command is active
            for (int ix = 0; ix < N_DATA_QUADS; ix++) {
              if (xv_config.aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
                if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...
                  if (xv_config.show_errors) {                           // if we're supposed to show data errors...
                    Serial.print(F("A,"));
                    Serial.print(startingAngle + ix);
                    Serial.print(F(","));
                    if (aryInvalidDataFlag[ix] & INVALID_DATA_FLAG)
                      Serial.println(F("I"));
                    if (aryInvalidDataFlag[ix] & STRENGTH_WARNING_FLAG)
                      Serial.println(F("S"));
                  }
                }
                else {                                         // show clean data
                  Serial.print(F("A,"));
                  Serial.print(startingAngle + ix);
                  Serial.print(F(","));
                  Serial.print(int(aryDist[ix]));
                  Serial.print(F(","));
                  Serial.println(aryQuality[ix]);
                }
              }  // if (xv_config.aryAngles[startingAngle + ix])
            }  // for (int ix = 0; ix < N_DATA_QUADS; ix++)
          }  // if (xv_config.show_dist)
        }  // if (eValidatePacket() == 0
        else if (xv_config.show_errors) {                                // we have encountered a CRC error
          Serial.println(F("C,CRC"));
        }
        // initialize a bunch of stuff before we switch back to State 1
        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          aryDist[ix] = 0;
          aryQuality[ix] = 0;
          aryInvalidDataFlag[ix] = 0;
        }
        for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
          Packet[ixPacket] = 0;
        ixPacket = 0;
        eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte
      }  // if (ixPacket == PACKET_LENGTH)
    }  // if (eState == eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  if (xv_config.motor_enable) {
    rpmPID.Compute();
    if (pwm_val != pwm_last) {
      Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);  // replacement for analogWrite()
      pwm_last = pwm_val;
    }
    motorCheck();
  }  // if (xv_config.motor_enable)
}  // loop
/*
   processIndex - Process the packet element 'index'
   index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
      (packet 89, readings 356 to 359).
   Enter with: N/A
   Uses:       Packet
               ledState gets toggled if angle = 0
               ledPin = which pin the LED is connected to
               ledState = LED on or off
               xv_config.show_dist = true if we're supposed to show distance
               curMillis = milliseconds, now
               lastMillis = milliseconds, last time through this subroutine
               xv_config.show_interval = true ==> display time interval once per revolution, at angle 0
   Calls:      digitalWrite() - used to toggle LED pin
               Serial.print
   Returns:    The first angle (of 4) in the current 'index' group
*/
uint16_t processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);

    if (xv_config.show_rpm) {
      Serial.print(F("R,"));
      Serial.print((int)motor_rpm);
      Serial.print(F(","));
      Serial.println((int)pwm_val);
    }

    curMillis = millis();
    if (xv_config.show_interval) {
      Serial.print(F("T,"));                                // Time Interval in ms since last complete revolution
      Serial.println(curMillis - lastMillis);
    }
    lastMillis = curMillis;

  } // if (angle == 0)
  return angle;
}
/*
   processSpeed- Process the packet element 'speed'
   speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value
      in RPM represented in fixed point, with 6 bits used for the decimal part).
   Enter with: N/A
   Uses:       Packet
               angle = if 0 then enable display of RPM and PWM
               xv_config.show_rpm = true if we're supposed to display RPM and PWM
   Calls:      Serial.print
*/
void processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}
/*
   Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
     byte 0 : <distance 7:0>
     byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
     byte 2 : <signal strength 7:0>
     byte 3 : <signal strength 15:8>
*/
/*
   processDistance- Process the packet element 'distance'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
                                       so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
   Calls:      N/A
   Exits with: 0 = okay
   Error:      1 << 7 = INVALID_DATA_FLAG is set
               1 << 6 = STRENGTH_WARNING_FLAG is set
*/
byte processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}
/*
   processSignalStrength- Process the packet element 'signal strength'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               quality[] = signal quality
   Calls:      N/A
*/
void processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

/*
   eValidatePacket - Validate 'Packet'
   Enter with: 'Packet' is ready to check
   Uses:       CalcCRC
   Exits with: 0 = Packet is okay
   Error:      non-zero = Packet is no good
*/
byte eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b))
    return VALID_PACKET;                       // okay
  else
    return INVALID_PACKET;                     // non-zero = bad CRC
}

/*
   initEEPROM
*/
void initEEPROM() {
  xv_config.id = 0x06;
  strcpy(xv_config.version, "1.3.0");

#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)  // if Teensy 2.0
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

#elif defined(__AVR_ATmega32U4__)  // if Leonardo or Pro Micro
  xv_config.motor_pwm_pin = 5;  // pin connected N-Channel Mosfet

#elif defined(__MK20DX256__)  // if Teensy 3.1
  xv_config.motor_pwm_pin = 33;  // pin connected N-Channel Mosfet
#endif

  xv_config.rpm_setpoint = 200;  // desired RPM
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
  xv_config.show_interval = false;
  xv_config.show_errors = false;
  for (int ix = 0; ix < N_ANGLES; ix++)
    xv_config.aryAngles[ix] = true;

  EEPROM_writeAnything(0, xv_config);
}
/*
   initSerialCommands
*/
void initSerialCommands() {
  sCmd.addCommand("help",        help);
  sCmd.addCommand("Help",        help);
  sCmd.addCommand("ShowConfig",  showConfig);
  sCmd.addCommand("SaveConfig",  saveConfig);
  sCmd.addCommand("ResetConfig", initEEPROM);

  sCmd.addCommand("SetAngle",      setAngle);
  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);

  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);

  sCmd.addCommand("ShowRaw",  showRaw);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowDist",  showDist);
  sCmd.addCommand("HideDist",  hideDist);
  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("ShowErrors", showErrors);
  sCmd.addCommand("HideErrors", hideErrors);
  sCmd.addCommand("ShowInterval", showInterval);
  sCmd.addCommand("HideInterval", hideInterval);
  sCmd.addCommand("ShowAll", showAll);
  sCmd.addCommand("HideAll", hideAll);

}
/*
   showAll - Show Dist, Errors, RPM, and Interval data
*/
void showAll() {
  showDist();
  showErrors();
  showRPM();
  showInterval();
}
/*
   hideAll - Hide Dist, Errors, RPM, and Interval data
*/
void hideAll() {
  hideDist();
  hideErrors();
  hideRPM();
  hideInterval();
}
/*
   showInterval - enable display of Time interval (which happens once per revolution, at angle 0
*/
void showInterval() {
  xv_config.show_interval = true;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing time interval (ms per revolution)"));
  }
}
/*
   hideInterval - suppress display of Time interval
*/
void hideInterval() {
  xv_config.show_interval = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding time interval"));
  }
}
/*
   showErrors
*/
void showErrors() {
  xv_config.show_errors = true;                                  // enable error display
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing errors"));
  }
}
/*
   hideErrors
*/
void hideErrors() {                                    // disable error display
  xv_config.show_errors = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding errors"));
  }
}
/*
   showRPM
*/
void showRPM() {
  xv_config.show_rpm = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing RPM data"));
  }
}
/*
   hideRPM
*/
void hideRPM() {
  xv_config.show_rpm = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding RPM data"));
  }
}

void showDist() {
  hideRaw();
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Code,Angle,Distance(mm),Signal strength"));
  }
  xv_config.show_dist = true;
}

void hideDist() {
  xv_config.show_dist = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding Distance data"));
  }
}
/*
   doSetAngle - Multi-angle range(s) implementation - DSH
   Command: SetAngles ddd, ddd-ddd, etc.
   Enter with: N/A
   Uses:       xv_config.aryAngles (an array of 360 booleans) is set to appropriate values
   Calls:      showDist
   Exits with: N/A
   TEST THIS STRING:  SetAngles 16-20, 300-305, 123-124, 10
*/
void setAngle() {
  char c, *arg;
  boolean syntax_error = false;
  int doing_from_to, from, to, ix, lToken, n_groups = 0;

  for (ix = 0; ix < N_ANGLES; ix++)                      // initialize
    xv_config.aryAngles[ix] = false;
  doing_from_to = 0;                                     // state = doing 'from'
  // Make sure that there is at least 1 angle or group of angles present
  do {
    arg = sCmd.next();                                   // get the next token
    if (arg == NULL) {                                   // it's empty -- just exit
      sCmd.readSerial();
      arg = sCmd.next();
      break;
    }
    // see if the token has an embedded "-", meaning from - to
    lToken = strlen(arg);                                // get the length of the current token
    for (ix = 0; ix < lToken; ix++) {
      c = arg[ix];
      if (c == ',') {                                    // optional trailing comma
        doing_from_to = 0;
        break;
      }
      else if (c == '-') {                               // optional '-' means "from - to"
        to = 0;
        doing_from_to = 1;                               // from now on, we're doing 'to'
      }
      else if (c == ' ') {                               // ignore blanks
        Serial.println(F("{ }"));
      }
      else if ((c >= '0') && (c <= '9')) {
        if (doing_from_to == 0) {
          from *= 10;
          from += c - '0';
          to = from;                                      // default to = from
          n_groups++;                                     // count the number of active groups (s/b >= 1)
        }
        else {
          to *= 10;
          to += c - '0';
        }
      }
      else {
        syntax_error = true;
        n_groups = 0;
        break;
      }
    }  // for (ix = 0; ix < lToken; ix++)
    // validate 'from' and 'to' and set 'xv_config.aryAngles' with correct values
    if ((from >= 0) && (from < N_ANGLES) && (to >= 0) && (to < N_ANGLES)) {
      if (to >= from) {
        for (ix = from; ix <= to; ix++) {
          xv_config.aryAngles[ix] = true;
        }
      }
      else {
        syntax_error = true;
        break;
      }
    }
    else {
      syntax_error = true;
      break;
    }
    from = 0;
    to = 0;
    doing_from_to = 0;
  }  // do
  while (arg != NULL);
  if (n_groups == 0)
    syntax_error = true;

  // Handle syntax errors
  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax"));
    Serial.println(F("  Example: SetAngle 0, 15-30, 45-50, 10"));
    Serial.println(F("  Example: SetAngle 0-359 to show all angles."));
    Serial.println(F("Notes: Use a space after each comma"));
    Serial.println(F("       No particular order is required"));
    Serial.println(F("       In a from-to pair, the 1st value must be lowest. From-to pairs can overlap ranges."));
  }
  else {                                                  // no errors detected, display the angles and start
    // We're ready to process multiple angles
    Serial.println(F(""));
    Serial.print(F("Angles:"));
    for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
      if (xv_config.aryAngles[ix]) {
        Serial.print(ix, DEC);
        Serial.print(F(","));
      }
    }
    Serial.println(F(""));
    showDist();
  }  // if not (syntax_error)
}

void motorOff() {
  xv_config.motor_enable = false;
  Timer3.pwm(xv_config.motor_pwm_pin, 0);
  Serial.println(F(" "));
  Serial.println(F("Motor off"));
}

void motorOn() {
  xv_config.motor_enable = true;
  Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);
  rpm_err = 0;  // reset rpm error
  Serial.println(F(" "));
  Serial.println(F("Motor on"));
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
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
  //Serial.println(F(" "));
  //Serial.println(F("Raw lidar data disabled"));
}

void showRaw() {
  xv_config.raw_data = true;
  hideDist();
  hideRPM();
  //Serial.println(F(" "));
  //Serial.println(F("Lidar data enabled"));
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
      Serial.println(xv_config.rpm_min);
    }
    if (sVal > xv_config.rpm_max) {
      sVal = xv_config.rpm_max;
      Serial.println(F(" "));
      Serial.print(F("RPM too high. Setting to maximum "));
      Serial.println(xv_config.rpm_max);
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
    Serial.println(F("Incorrect syntax.  Example: SetRPM 200"));
  }
  else {
    Serial.print(F("Old RPM setpoint:"));
    Serial.println(xv_config.rpm_setpoint);
    xv_config.rpm_setpoint = sVal;
    //Serial.println(F(" "));
    Serial.print(F("New RPM setpoint: "));
    Serial.println(sVal);
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
    Serial.println(F("Incorrect syntax.  Example: SetKp 1.0"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kp to: "));
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
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKi 0.5"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Ki to: "));
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
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKd 0.001"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kd to: "));
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
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetSampleTime 20"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Sample time to: "));
    Serial.println(sVal);
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
  Serial.print(F("GetSurreal.com *"));

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.println(F("List of available commands (case sensitive)"));

  Serial.println(F(" "));
  Serial.println(F("Control commands"));
  Serial.println(F("  ShowConfig    - Show the running configuration"));
  Serial.println(F("  SaveConfig    - Save the running configuration to EEPROM"));
  Serial.println(F("  ResetConfig   - Restore the original configuration"));
  Serial.println(F("  SetAngle      - Show distance data for a multiple angles (Ex: SetAngle 0, 15-30, 45-50, 10)"));
  Serial.println(F("  SetRPM        - Set the desired rotation speed (min: 180, max: 349)"));
  Serial.println(F("  MotorOff      - Stop spinning the lidar"));
  Serial.println(F("  MotorOn       - Enable spinning of the lidar"));

  Serial.println(F(" "));
  Serial.println(F("Data commands"));
  Serial.println(F("  ShowRaw       - Enable the output of the raw lidar data (default)"));
  Serial.println(F("  HideRaw       - Stop outputting the raw data from the lidar"));
  Serial.println(F("  ShowDist      - Show angles with distance data"));
  Serial.println(F("  HideDist      - Hide the distance data"));
  Serial.println(F("  ShowErrors    - Show all error types (CRC, Signal Strength, and Invalid"));
  Serial.println(F("  HideErrors    - Hide angles with errors"));
  Serial.println(F("  ShowRPM       - Show the rotation speed"));
  Serial.println(F("  HideRPM       - Hide the rotation speed"));
  Serial.println(F("  ShowInterval  - Show time interval per revolution in ms, at angle=0"));
  Serial.println(F("  HideInterval  - Hide time interval"));
  Serial.println(F("  ShowAll       - Show the distance, errors, RPMs and interval data"));
  Serial.println(F("  HideAll       - Hide the distance, errors, RPMs and interval data"));

  Serial.println(F(" "));
  Serial.println(F("PID commands"));
  Serial.println(F("  SetKp         - Set the proportional gain"));
  Serial.println(F("  SetKi         - Set the integral gain"));
  Serial.println(F("  SetKd         - Set the derivative gain"));
  Serial.println(F("  SetSampleTime - Set the frequency the PID is calculated (ms)"));

  Serial.println(F(" "));
  Serial.println(F("Output comma-separated format:"));
  Serial.println(F("  A,<Angle>,<Distance in mm>,<Strength>"));
  Serial.println(F("  C,CRC error was generated by LIDAR"));
  Serial.println(F("  R,<RPMs>,<PWM value>"));
  Serial.println(F("  T,<Time interval in milliseconds>"));

  Serial.println(F(" "));
  Serial.println(F("Errors:"));
  Serial.println(F("  CRC = CRC Error"));
  Serial.println(F("    I = LIDAR reports Invalid data for this angle"));
  Serial.println(F("    S = LIDAR reports Poor signal strength for this angle"));
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
  Serial.print(F("Show Time Interval: "));
  Serial.println(xv_config.show_interval);
  Serial.print(F("Show Angle(s): "));
  for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
    if (xv_config.aryAngles[ix]) {
      Serial.print(ix, DEC);
      Serial.print(F(","));
    }
  }
  Serial.println(F(" "));
  Serial.println(F(" "));
}

void saveConfig() {
  EEPROM_writeAnything(0, xv_config);
  Serial.println(F("Config Saved."));
}

