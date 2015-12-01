/*
  XV Lidar Controller v1.3.0
 
 Copyright 2014 James LeRoy getSurreal
 https://github.com/getSurreal/XV_Lidar_Controller
 http://www.getsurreal.com/products/xv-lidar-controller

 NOTE: USES THE ARDUINO 1.6.6 DEVELOPMENT PLATFORM
 
 Modified to add CRC checking - Doug Hilton, WD0UG November, 2015 mailto: six.speed (at) yahoo (dot) com
 Modified to add ShowErrors / HideErrors - DSH
 Modified to add show multiple angles - DSH
 Modified SerialCommand.h to increase # of characters allowed to be input and command length - DSH
           #define SERIALCOMMAND_BUFFER 100		// ORIGINAL: 32   MODIFIED BY DSH
           #define SERIALCOMMAND_MAXCOMMANDLENGTH 20	// ORIGINAL:
 ****************************************************************************************
 NOTE: DON'T FORGET TO COPY THE NEW SerialCommand.h TO YOUR ARDUINO LIBRARY DIRECTORY!!!
 ****************************************************************************************
 Modified to add ShowCSV and HideCSV command - DSH
 Modified EEPROM configuration to add new values and delete old ones - DSH
 Modified to add ShowAll/HideAll command - DSH
 Modified to let ShowInterval and ShowRPM run without ShowAngle
 Modified to check that xv_config is not too big for EEPROM
 Modified to delete ShowDist/HideDist commands, since that functionality is subsumed by ShowAngle(s)
 
 See README for additional information 
 
 The F() macro in the Serial statements tells the compiler to keep your strings in PROGMEM

 The Teensy 20. board (REF: http://www.pjrc.com/teensy/index.html)
  Processor:    ATMEGA32U4 8-bit AVR 16MHz
  Flash memory: 32256 
  RAM memory:   2560
  EEPROM:       1024
  I/O:          25, 5 Volt
  Analog In:    12
  PWM:          7
  UART,I2C,SPI: 1,1,1
 
 */

#include <TimerThree.h> // used for ultrasonic PWM motor control
#include <PID.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <SerialCommand.h>
// ******************************************************************
// WARNING: If you change 'xv_config', you must change '_EEPROM_ID' *
//          When it is changed, the EEPROM data will be initialized *
// ******************************************************************
const byte _EEPROM_ID = 0x07;                        // used to validate EEPROM initialized (Current as of 26NOV2015 - DSH)
const byte _EEPROM_SAFETY_CHECK = (-1 - _EEPROM_ID); // used to validate that we haven't written past the end of the EEPROM
// You may change 'szCURRENT_EEPROM_VERSION' to indicate a new software version, but it is not checked, just displayed
const char szCURRENT_EEPROM_VERSION[6] = "1.3.0";    // as of 24NOV2015 - DSH

const int _N_ANGLES = 360;                           // # of angles in a circle (0..359)
const int  _EEPROM_SIZE = 1024;                      // Size of EEPROM, in bytes
// These constants will let us read and write individual variables to the EEPROM - DSH
const byte _BITS_PER_BYTE = 8;
const byte _MASK_BYTE = (1 << _BITS_PER_BYTE) - 1;                // (value: 0xFF)
const unsigned int _MASK_WORD_LSB = _MASK_BYTE;                   // Mask off bottom 8 bits of INTEGER (value:0xFF)
const unsigned int _MASK_WORD_MSB = _MASK_BYTE << _BITS_PER_BYTE; // Mask off top 8 bits of INTEGER (value:0xFF00)

const int _LEN_BYTE = 1;
const int _LEN_BOOLEAN = 1;
const int _LEN_INTEGER = 2;
const int _LEN_DOUBLE = 4;
/*
 * An enumerated list of "What we're displaying" commands
 * WARNING: IF YOU ADD A VALUE, FIX VARIOUS swich (eOUT) STATEMENTS IN THE CODE BELOW!!!
 * WARNING: MUST BE LESS THAN 256 VALUES (masked by 0xFF)
 */
const byte _eOUT_UNKNOWN = 0;                 // Unknown or uninitialized (value: 0)
const byte _eOUT_RAW = _eOUT_UNKNOWN + 1;     // ShowRaw = retransmit the seiral data to the USB port (value: 1)
const byte _eOUT_ANGLE = _eOUT_RAW + 1;       // ShowAngle = display angle data (value: 2)
const byte _eOUT_MAX = _eOUT_ANGLE + 1;       // WARNING: This value must fit in 8 bits (range: 0-255) (value: 3)
/*
 * An enumerated list of special output requirements, e.g., Show Errors, Show RPM, Show Interval, Show CSV format
 * NOTE: These are bit-values, so they can be individually set/cleared individually
 */
const unsigned int _eSHOW_RPM = 0;                        // Show R.P.M.
const unsigned int _eSHOW_CSV = _eSHOW_RPM + 1;           // Show in .csv format
const unsigned int _eSHOW_ERRORS = _eSHOW_CSV + 1;        // Show Errors
const unsigned int _eSHOW_INTERVAL = _eSHOW_ERRORS + 1;   // Show Interval
const unsigned int _eSHOW_MAX = _eSHOW_INTERVAL + 1;      // WARNING: This value is a bit shift number, so it must be 0-7 (fits into top byte of 'eOUT')
/*
 * The top 8-bits of eOUT indicate special output requirements
 */
const unsigned int _OUT_MOD_BASE = 1 << _BITS_PER_BYTE;   // Start the display modifiers in the top 8-bits (value: 0x100)
/*      MSB                 LSB
   7 6 5 4 3 2 1 0    7 6 5 4 3 2 1 0
   x x x x x x x x    n n n n n n n n
   | | | | | | | + RPM
   | | | | | | + CSV
   | | | | } + ERROR
   | | | | + Interval
   | | | + RFU 1
   | | + RFU 2
   | + RFU 3
   + RFU 4           
*/             
// NOTE: if you add or change these, don't forget to fix up 'showRaw'
const unsigned int _OUT_MOD_SHOW_RPM =      _OUT_MOD_BASE;              // 1 = show RPM (value: 0x100) manipulated by showRPM/hideRPM
const unsigned int _OUT_MOD_SHOW_CSV =      _OUT_MOD_SHOW_RPM * 2;      // 1 = show CSV format (value: 0x200)
const unsigned int _OUT_MOD_SHOW_ERRORS =   _OUT_MOD_SHOW_CSV * 2;      // 1 = show errors (value: 0x400)
const unsigned int _OUT_MOD_SHOW_INTERVAL = _OUT_MOD_SHOW_ERRORS * 2;   // 1 = show interval (value: 0x800)
const unsigned int _OUT_MOD_SHOW_RFU_1 =    _OUT_MOD_SHOW_INTERVAL * 2; // 1 = reserved future use 1 (value: 0x1000)
const unsigned int _OUT_MOD_SHOW_RFU_2 =    _OUT_MOD_SHOW_RFU_1 * 2;    // 1 = reserved future use 2 (value: 0x2000)
const unsigned int _OUT_MOD_SHOW_RFU_3 =    _OUT_MOD_SHOW_RFU_2 * 2;    // 1 = reserved future use 3 (value: 0x4000)
const unsigned int _OUT_MOD_SHOW_RFU_4 =    _OUT_MOD_SHOW_RFU_3 * 2;    // 1 = reserved future use 4 (value: 0x8000)

#define _MASK_eOUT_SHOW _MASK_WORD_LSB                  // mask 'eOUT' for what we're showing (value:0xFF)
#define _MASK_eOUT_MODIFIER _MASK_WORD_MSB              // mask 'eOUT' for display modifiers (value:0xFF00)

const int _EE_LEN_ID = _LEN_BYTE;                       // value:1
const int _EE_LEN_VERSION = 6;                          // value:6 (arbitrary, but currently contains, e.g., string "1.3.0" (plus 0-terminator)
const int _EE_LEN_MOTOR_PWM_PIN = _LEN_BYTE;            // value:1
const int _EE_LEN_RPM_SETPOINT = _LEN_DOUBLE;           // value:4
const int _EE_LEN_RPM_MIN = _LEN_DOUBLE;                // value:4
const int _EE_LEN_RPM_MAX = _LEN_DOUBLE;                // value:4
const int _EE_LEN_PWM_MAX = _LEN_DOUBLE;                // value:4
const int _EE_LEN_PWM_MIN = _LEN_DOUBLE;                // value:4
const int _EE_LEN_SAMPLE_TIME = _LEN_INTEGER;           // value:2
const int _EE_LEN_Kp = _LEN_DOUBLE;                     // value:4
const int _EE_LEN_Ki = _LEN_DOUBLE;                     // value:4
const int _EE_LEN_Kd = _LEN_DOUBLE;                     // value:4
const int _EE_LEN_MOTOR_ENABLE = _LEN_BOOLEAN;          // value:1
const int _EE_LEN_SHOW_ALL_ANGLES = _LEN_BOOLEAN;       // value:1
const int _EE_LEN_eOUT = _LEN_INTEGER;                  // value:2
const int _EE_LEN_aryAngles = _N_ANGLES * _LEN_BOOLEAN; // value:360
const int _EE_LEN_SAFETY = _LEN_BYTE;                   // value:1

const int _EE_OFFSET_ID = 0;                                                            // value:0
const int _EE_OFFSET_VERSION = _EE_OFFSET_ID + _EE_LEN_ID;                              // value:1
const int _EE_OFFSET_MOTOR_PWM_PIN = _EE_OFFSET_VERSION + _EE_LEN_VERSION ;             // value:7
const int _EE_OFFSET_RPM_SETPOINT = _EE_OFFSET_MOTOR_PWM_PIN + _EE_LEN_MOTOR_PWM_PIN ;  // value:8
const int _EE_OFFSET_RPM_MIN = _EE_OFFSET_RPM_SETPOINT + _EE_LEN_RPM_SETPOINT ;         // value:12
const int _EE_OFFSET_RPM_MAX = _EE_OFFSET_RPM_MIN + _EE_LEN_RPM_MIN ;                   // value:16
const int _EE_OFFSET_PWM_MAX = _EE_OFFSET_RPM_MAX + _EE_LEN_RPM_MAX ;                   // value:20
const int _EE_OFFSET_PWM_MIN = _EE_OFFSET_PWM_MAX + _EE_LEN_PWM_MAX ;                   // value:24
const int _EE_OFFSET_SAMPLE_TIME = _EE_OFFSET_PWM_MIN + _EE_LEN_PWM_MIN ;               // value:28
const int _EE_OFFSET_Kp = _EE_OFFSET_SAMPLE_TIME + _EE_LEN_SAMPLE_TIME ;                // value:30
const int _EE_OFFSET_Ki = _EE_OFFSET_Kp + _EE_LEN_Kp ;                                  // value:34
const int _EE_OFFSET_Kd = _EE_OFFSET_Ki + _EE_LEN_Ki ;                                  // value:38
const int _EE_OFFSET_MOTOR_ENABLE = _EE_OFFSET_Kd + _EE_LEN_Kd ;                        // value:42
const int _EE_OFFSET_SHOW_ALL_ANGLES = _EE_OFFSET_MOTOR_ENABLE + _EE_LEN_MOTOR_ENABLE ; // value:43
const int _EE_OFFSET_eOUT = _EE_OFFSET_SHOW_ALL_ANGLES + _EE_LEN_SHOW_ALL_ANGLES ;      // value:44
const int _EE_OFFSET_aryAngles = _EE_OFFSET_eOUT + _EE_LEN_eOUT ;                       // value:46
const int _EE_OFFSET_SAFETY = _EE_OFFSET_aryAngles + _EE_LEN_aryAngles ;                // value:406
const int _EEPROM_Config_LEN = _EE_OFFSET_SAFETY + _EE_LEN_SAFETY ;                     // value:407
/*
 * Default values for EEPROM
 */
const byte _DEFAULT_motor_pwm_pin = 9;            // pin connected N-Channel Mosfet
const double _DEFAULT_rpm_setpoint = 200;         // desired RPM (arrived at this value by testing)
const double _DEFAULT_rpm_min = 200;
const double _DEFAULT_rpm_max = 300;
const double _DEFAULT_pwm_min = 100;
const int _DEFAULT_pwm_max = 1023;
const int _DEFAULT_sample_time = 20;  
const double _DEFAULT_Kp = 2.0;                   // PID tuning values
const double _DEFAULT_Ki = 1.0;
const double _DEFAULT_Kd = 0.0;
const boolean _DEFAULT_bMotorEnable = true;       // default is to turn the motor ON  
const boolean _DEFAULT_bShowAllAngles = false;    // default 
const byte _DEFAULT_eOUT = _eOUT_RAW;             // default to "RAW" display
const boolean _DEFAULT_bANGLE = false;            // default for entries in 'aryAngle'
const double _DEFAULT_pwm_val = 500;              // 50% duty cycle
const unsigned long _DEFAULT_motor_check_interval = 200;
const unsigned int _DEFAULT_rpm_err_thresh = 10;
/* 
 *  Define the structure of the record that gets written to the EEPROM
 */
struct EEPROM_Config {
  byte id;                                // value == _EEPROM_ID
  char szVersion[_EE_LEN_VERSION];        // e.g., "1.3.0" plus 0-terminator
  byte motor_pwm_pin;                     // pin connected to mosfet for motor speed control, e.g., 9
  double rpm_setpoint;                    // desired RPM (uses double to be compatible with PID library)
  double rpm_min;
  double rpm_max;
  double pwm_max;                         // max analog value.  probably never needs to change from 1023
  double pwm_min;                         // min analog pulse value to spin the motor
  int sample_time;                        // how often to calculate the PID values
  // PID tuning values
  double Kp;
  double Ki;
  double Kd;
  boolean bMotorEnable;                   // true = spin the motor.  NOTE: No data is available when not spinning
  boolean bShowAllAngles;                 // true = shows all angles
  unsigned int eOUT;                      // What we're displaying now (2 8-bit nibbles)
  // Bottom 8-bits = a value indicating what we're supposed to print, i.e., { unknown; raw; distance; angle }
  // Top 8-bits = a bit-mask with display modifiers, e.g., {show RPM; show CSV format; show errors; show interval}
  boolean aryAngles[_N_ANGLES];           // each entry = true if we're supposed to display the corresponding angle
  byte safety;                            // a value to make sure that EEPROM data hasn't been overwritten
} 
xv_config;

double pwm_val = _DEFAULT_pwm_val;        // start with ~50% power
double pwm_last;
double motor_rpm;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = _DEFAULT_motor_check_interval;  
unsigned int rpm_err_thresh = _DEFAULT_rpm_err_thresh;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillis = millis();

// Some Packet-related constants (DSH)
const unsigned char _COMMAND = 0xFA;       // Start of new packet
const int _INDEX_LO = 0xA0;                // lowest index value
const int _INDEX_HI = 0xF9;                // highest index value

const int _N_DATA_QUADS = 4;               // there are 4 groups of data elements
const int _N_ELEMENTS_PER_QUAD = 4;        // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int _OFFSET_TO_START = 0;
const int _OFFSET_TO_INDEX = _OFFSET_TO_START + 1;
const int _OFFSET_TO_SPEED_LSB = _OFFSET_TO_INDEX + 1;
const int _OFFSET_TO_SPEED_MSB = _OFFSET_TO_SPEED_LSB + 1;
const int _OFFSET_TO_4_DATA_READINGS = _OFFSET_TO_SPEED_MSB + 1;
const int _OFFSET_TO_CRC_L = _OFFSET_TO_4_DATA_READINGS + (_N_DATA_QUADS * _N_ELEMENTS_PER_QUAD);
const int _OFFSET_TO_CRC_M = _OFFSET_TO_CRC_L + 1;
const int _PACKET_LENGTH = _OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int _OFFSET_DATA_DISTANCE_LSB = 0;
const int _OFFSET_DATA_DISTANCE_MSB = _OFFSET_DATA_DISTANCE_LSB + 1;
const int _OFFSET_DATA_SIGNAL_LSB = _OFFSET_DATA_DISTANCE_MSB + 1;
const int _OFFSET_DATA_SIGNAL_MSB = _OFFSET_DATA_SIGNAL_LSB + 1;

int Packet[_PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                           // index into 'Packet' array
const int _VALID_PACKET = 0;
const int _IN_VALID_PACKET = _VALID_PACKET + 1;
const byte _INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"

boolean bEEPROM_Initialized = false;        // true if EEPROM was upgraded to latest software version

/* REF: https://github.com/Xevel/NXV11/wiki
The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be 
only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the 
data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is 
interrupted by the supports of the cover) .
*/
const byte _STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
/*
The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance. 
This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected 
size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic 
reflections (glass... ).
*/
const byte _BAD_DATA_MASK = (_INVALID_DATA_FLAG | _STRENGTH_WARNING_FLAG);

const byte _eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte _eState_Build_Packet = _eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = _eState_Find_COMMAND;

PID rpmPID(&motor_rpm, &pwm_val, &xv_config.rpm_setpoint, xv_config.Kp, xv_config.Ki, xv_config.Kd, DIRECT);

uint8_t inByte = 0;  // incoming serial byte
uint8_t motor_rph_high_byte = 0; 
uint8_t motor_rph_low_byte = 0;
uint16_t aryDist[_N_DATA_QUADS] = {0,0,0,0};     // thre are (4) distances, one for each data quad
// NOTE: the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[_N_DATA_QUADS] = {0,0,0,0};  // same with 'quality'
uint16_t motor_rph = 0;
uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)
int iCurrentAngle = 0;
SerialCommand sCmd;                              // create an instance of the serial device handler object

const int ledPin = 11;                           // blink the LED
boolean ledState = LOW;                          // Toggle LOW/HIGH
byte aryInvalidDataFlag[_N_DATA_QUADS] = {0,0,0,0};  // non-zero = _INVALID_DATA_FLAG or _STRENGTH_WARNING_FLAG is set
int eShow;                                              // What to display {UNKNOWN; RAW; DISTANCE; ANGLE}
int oldAngle = 0;

// initialization (before 'loop')
void setup() {
  Serial.begin(115200);                           // USB serial
  // Software consistency checks (catchable programmer mistakes)
  if ((_eOUT_MAX != 3) || (_eOUT_MAX > _MASK_BYTE)) { // limit decimal value to what will fit in 1 byte
    Serial.println(F("SOFTWARE ERROR: _eOUT_MAX invalid. HALT!"));
    while (1);                                     // HANG!!!!!!
  }
  if (_eSHOW_MAX >= _BITS_PER_BYTE) {              // limit binary value to what can be expressed in 8-bits
    Serial.println(F("SOFTWARE ERROR: _eSHOW_MAX invalid. HALT!"));
    while (1);                                     // HANG!!!!
  }
  if (_EEPROM_Config_LEN >= _EEPROM_SIZE) {
    Serial.println(F("SOFTWARE ERROR: _EEPROM_Config_LEN >= _EEPROM_SIZE. HALT!"));
    while (1);                                     // HANG!!!!
  }
  EEPROM_readAnything(_EE_OFFSET_ID, xv_config);   // read the saved EEPROM configuration
  if(xv_config.id != _EEPROM_ID) {                 // If EEPROM ID has changed...
    Serial.println(F("EEPROM version is different from last time. Initializing EEPROM"));
    initEEPROM();                                  // ...initialize EEPROM to new layout
    //showConfig();                                  // for DEBUGGING: display EEPROM configuration data
  }  
  pinMode(xv_config.motor_pwm_pin, OUTPUT);        // set the pin that drives the LIDAR motor
  Serial1.begin(115200);                           // XV LDS data   
  Timer3.initialize(30);                           // set PWM frequency to 32.768kHz  
  rpmPID.SetOutputLimits(xv_config.pwm_min,xv_config.pwm_max);
  rpmPID.SetSampleTime(xv_config.sample_time);
  rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  rpmPID.SetMode(AUTOMATIC);
  initSerialCommands();                            // Add commands to te 'sCmd' object
  pinMode(ledPin, OUTPUT);
  eState = _eState_Find_COMMAND;                   // Initialize the 'state' for the main loop
  for (ixPacket = 0; ixPacket < _PACKET_LENGTH; ixPacket++)  // Initialize
    Packet[ixPacket] = 0;
  ixPacket = 0;   
  if (xv_config.bMotorEnable == false)             // Turn on the motor
    forceMotorOn();                                // enable the motor
  forceRaw();                                      // Default mode = 'show raw data'
  hideRaw();                                       // DSH ONLY!!!!!!!!!!!!   
}

// Main loop (forever)
void loop() {  
  sCmd.readSerial();                                      // check for incoming serial commands
  if (Serial1.available() > 0) {                          // If a byte is available...
    inByte = Serial1.read();                              // ...fetch incoming byte
    eShow = (xv_config.eOUT & _MASK_eOUT_SHOW);           // what are we supposed to display?
    if (eShow == _eOUT_RAW)                               // if we're doing 'RAW' data...
      Serial.print(inByte, BYTE);                         // ... echo input to output
    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == _eState_Find_COMMAND) {                 // flush input until we receive a COMMAND (value: 0xFA)
      if(inByte == _COMMAND) {
        eState++;                                         // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;                      // store 1st byte of data into 'Packet'
      }
    }
    else {                                                // eState == _eState_Build_Packet
      Packet[ixPacket++] = inByte;                        // keep storing input into 'Packet'
      if (ixPacket == _PACKET_LENGTH) {                   // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == _VALID_PACKET) {         // Check packet CRC
          startingAngle = processIndex();                 // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...        
          processSpeed();                                 // process the speed
          // process each of the (4) sets of data in the packet        
          for (int ix = 0; ix < _N_DATA_QUADS; ix++)      // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < _N_DATA_QUADS; ix++) {    // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0) {
              processSignalStrength(ix);
            } // if (aryInvalidDataFlag[ix] == 0)
          } // for (int ix = 0; ix < _N_DATA_QUADS; ix++)          
          switch (eShow) {            
            //case _eOUT_DIST :                                   // eShow = display distance data
              //break;
            case _eOUT_UNKNOWN :                                // eShow = Unknown or uninitialized
            case _eOUT_ANGLE :                                  // eShow = display angle data
              for (int ix = 0; ix < _N_DATA_QUADS; ix++) {
                // We can't count on being able to display speed data at angle = 0
                // so we look for the angle to go from a high value to a lower value
                // which is safe to assume that it's either 0 or the closest value that is valid
                iCurrentAngle = startingAngle + ix;             // convert unsigned to signed integer for compare
                if (iCurrentAngle < oldAngle)                   // the angle has gone from, e.g., 359 to 0
                  if (xv_config.eOUT & _OUT_MOD_SHOW_RPM)       // if we're supposed to be showing RPM...
                    if (xv_config.bMotorEnable)                 // if the motor is spinning...
                      showSpeed();                              // ...display RPM once per rev, at angle 0 (or as close as possible)
                oldAngle = iCurrentAngle;
                if (xv_config.aryAngles[startingAngle + ix]) {  // if we're supposed to display that angle                    
                  if (aryInvalidDataFlag[ix] == 0) {            // make sure that the 'Invalid Data' flag is clear
                    if (eShow == _eOUT_ANGLE) {                 // only display this stuff if we're specifically displaying angles
                      if (xv_config.eOUT & _OUT_MOD_SHOW_CSV) {   // Display in CSV format?
                        Serial.print(startingAngle + ix);
                        Serial.print(F(","));
                        Serial.print(int(aryDist[ix]));
                        Serial.print(F(","));
                        Serial.println(aryQuality[ix]);
                      }
                      else {
                        Serial.print(startingAngle + ix);
                        Serial.print(F(": "));
                        Serial.print(int(aryDist[ix]));
                        Serial.print(F(" ("));
                        Serial.print(aryQuality[ix]);
                        Serial.println(F(")"));                         
                      }                                            
                    } // if (eShow == _eOUT_ANGLE)
                  } // if (aryInvalidDataFlag[ix] == 0)
                  else if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS) { // If we're supposed to show errors...
                    if (xv_config.bMotorEnable) {                   // and if the motor is spinning...
                      if (eShow == _eOUT_ANGLE) {                 // only display this stuff if we're specifically displaying angles
                        if (xv_config.eOUT & _OUT_MOD_SHOW_CSV) {     // Display in CSV format?
                          Serial.print(startingAngle + ix);
                          Serial.print(F(","));
                          if (aryInvalidDataFlag[ix] & _INVALID_DATA_FLAG)
                            Serial.println(F("I,"));
                          if (aryInvalidDataFlag[ix] & _STRENGTH_WARNING_FLAG) 
                            Serial.println(F("S,"));                                                
                        }
                        else {                                        // display in normal format
                          Serial.print(startingAngle + ix);
                          Serial.print(F(": "));
                          if (aryInvalidDataFlag[ix] & _INVALID_DATA_FLAG)
                            Serial.println(F("{I}"));
                          if (aryInvalidDataFlag[ix] & _STRENGTH_WARNING_FLAG) 
                            Serial.println(F("{S}"));                        
                        }                        
                      } // if (eShow == _eOUT_ANGLE)
                    } // if (xv_config.bMotorEnable)
                  }  // else if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS)
                }  // if (xv_config.aryAngles[startingAngle + ix])
              }  // for (int ix = 0; ix < _N_DATA_QUADS; ix++)
            break;
          } // switch (eShow)
      }  // if (eValidatePacket() == 0
      else {                                                  // Packet did not pass CRC check!
        if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS) {          // Show errors?
          /* DEBUGGING ONLY: Dump the contents of 'Packet'
          for (int ix = 0; ix < _PACKET_LENGTH; ix++) {       // a full error display (dump Packet to screen)
            if (Packet[ix] < 0x10)
              Serial.print(F("0"));
            Serial.print(Packet[ix], HEX);
            Serial.print(F(" "));
          }
          */
          Serial.println(F("CRC"));                           // an abbreviated error display
        } // if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS)
      } // if not (eValidatePacket() == 0
      // initialize a bunch of stuff before we switch back to State 1
      for (int ix = 0; ix < _N_DATA_QUADS; ix++) {
        aryDist[ix] = 0;
        aryQuality[ix] = 0;
        aryInvalidDataFlag[ix] = 0;
      }
      for (ixPacket = 0; ixPacket < _PACKET_LENGTH; ixPacket++)  // clear out this packet
        Packet[ixPacket] = 0;
      ixPacket = 0;      
      eState = _eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte        
      }  // if (ixPacket == _PACKET_LENGTH)
    }  // if not (eState == _eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  //
  if (xv_config.bMotorEnable) {  
    rpmPID.Compute();
    if (pwm_val != pwm_last) {
      Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);  // replacement for analogWrite()
      pwm_last = pwm_val;
    }
    motorCheck();
  }  // if (xv_config.bMotorEnable)  
}  // loop
/*
 * processIndex - Process the packet element 'index'
 * index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9 
 *    (packet 89, readings 356 to 359).
 * Enter with: N/A
 * Uses:       Packet
 *             ledState gets toggled if angle = 0
 *             ledPin = which pin the LED is connected to
 *             ledState = LED on or off
 *             xv_config.eOUT = output type
 *             xv_config.bShowInterval
 *             curMillis = milliseconds, now
 *             lastMillis = milliseconds, last time through this subroutine
 * Calls:      digitalWrite() - used to toggle LED pin
 *             Serial.print
 * Returns:    The first angle (of 4) in the current 'index' group
 */
uint16_t processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[_OFFSET_TO_INDEX] - _INDEX_LO;
  angle = data_4deg_index * _N_DATA_QUADS;     // 1st angle in the set of 4  
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    } 
    else {
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);
    if (((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_UNKNOWN) 
     || ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_ANGLE)) {
      curMillis = millis();
      if (xv_config.eOUT & _OUT_MOD_SHOW_INTERVAL) {
        if (xv_config.bMotorEnable) {                 // if the motor is spinning...
          Serial.print(F("Time interval ms:"));
          Serial.println(curMillis - lastMillis);
        }
      } // if (xv_config.eOUT & _OUT_MOD_SHOW_INTERVAL)
      lastMillis = curMillis;
    } // if (((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_DIST)...
  } // if (angle == 0)
  return angle;
}
/*
 * processSpeed- Keep the motor spinning
 * speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value 
 *    in RPM represented in fixed point, with 6 bits used for the decimal part).
 * Enter with: N/A
 * Uses:       Packet
 *             motor_rph_low_byte, motor_rph_high_byte
 *             motor_rph, motor_rpm
 * Calls:      Serial.print
 */
void processSpeed() {
  motor_rph_low_byte = Packet[_OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[_OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}
/*
 * showSpeed - Display the motor RPM once per revolution (at angle 0, or as close to it as possible)
 * Enter with: N/A
 * Uses:       xv_config.eOUT
 *             motor_rpm
 *             pwm_val
 * Calls:      processSpeed
 */
void showSpeed() {
  if (xv_config.eOUT & _OUT_MOD_SHOW_CSV) {             // is display format "CSV"?
    Serial.print(F("RPM:"));
    Serial.print(motor_rpm);
    Serial.print(F(",PWM:"));   
    Serial.println(pwm_val);                
  }
  else {                                                // normal display format
    Serial.print(F("RPM: "));
    Serial.print(motor_rpm);
    Serial.print(F("  PWM: "));   
    Serial.println(pwm_val);        
  }
  processSpeed();                                         // keep the motor spinning
}
/*
 * Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
 *   byte 0 : <distance 7:0>
 *   byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
 *   byte 2 : <signal strength 7:0>
 *   byte 3 : <signal strength 15:8>
 */
/*
 * processDistance- Process the packet element 'distance'
 * Enter with: iQuad = which one of the (4) readings to process, value = 0..3
 * Uses:       Packet
 *             dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
 *                                     so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
 * Calls:      N/A
 * Exits with: 0 = okay
 * Error:      1 << 7 = _INVALID_DATA_FLAG is set
 *             1 << 6 = _STRENGTH_WARNING_FLAG is set
 */
byte processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = _OFFSET_TO_4_DATA_READINGS + (iQuad * _N_DATA_QUADS) + _OFFSET_DATA_DISTANCE_LSB;    
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags    
  if (dataM & _BAD_DATA_MASK)             // if either _INVALID_DATA_FLAG or _STRENGTH_WARNING_FLAG is set...
    return dataM & _BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}
/*
 * processSignalStrength- Process the packet element 'signal strength'
 * Enter with: iQuad = which one of the (4) readings to process, value = 0..3
 * Uses:       Packet
 *             quality[] = signal quality
 * Calls:      N/A
 */
void processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = _OFFSET_TO_4_DATA_READINGS + (iQuad * _N_DATA_QUADS) + _OFFSET_DATA_SIGNAL_LSB;    
  dataL = Packet[iOffset];                  // signal strength LSB  
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

/*
 * eValidatePacket - Validate 'Packet'
 * Enter with: 'Packet' is ready to check
 * Uses:       CalcCRC
 * Exits with: 0 = Packet is okay
 * Error:      non-zero = Packet is no good
 */
byte eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = _PACKET_LENGTH - 2;
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
  b1b = Packet[_OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[_OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b)) 
    return _VALID_PACKET;                       // okay
  else
    return _IN_VALID_PACKET;                     // non-zero = bad CRC
}

/*
 * initEEPROM - Initialize the EEPROM to known values
 */
void initEEPROM() {
  int ix;
  /*
  byte *ptr_xv_config = &xv_config.id;
  for (ix = 0; ix < _EEPROM_Config_LEN; ix++)    // erase the EEPROM
    EEPROM_writeAnything(ix, dummy);  

  for (int ix = 0; ix < _EEPROM_Config_LEN; ix++) {
    if (*ptr_xv_config < 0x10)
      Serial.print(F("0"));
    Serial.print(*ptr_xv_config++, HEX);
    Serial.print(F(" "));
    if ((ix & 0x0F) == 0x0F)                      // CRLF after 16 bytes displayed
      Serial.println(F(" "));
  }
  Serial.println(F(" "));
  */
  xv_config.id = _EEPROM_ID;                              // set it to current value
  xv_config.safety = _EEPROM_SAFETY_CHECK;                // set "overwrite check" safety byte to known value  
  strcpy(xv_config.szVersion, szCURRENT_EEPROM_VERSION);  // Store the current software version
  xv_config.motor_pwm_pin = _DEFAULT_motor_pwm_pin;       // pin connected N-Channel Mosfet
  xv_config.rpm_setpoint = _DEFAULT_rpm_setpoint;         // desired RPM (arrived at this value by testing)
  xv_config.rpm_min = _DEFAULT_rpm_min;                   // set the rest of the default values...
  xv_config.rpm_max = _DEFAULT_rpm_max;
  xv_config.pwm_min = _DEFAULT_pwm_min;
  xv_config.pwm_max = _DEFAULT_pwm_max;
  xv_config.sample_time = _DEFAULT_sample_time;
  // PID tuning values
  xv_config.Kp = _DEFAULT_Kp;
  xv_config.Ki = _DEFAULT_Ki;
  xv_config.Kd = _DEFAULT_Kd;
  xv_config.bMotorEnable = _DEFAULT_bMotorEnable;
  xv_config.bShowAllAngles = _DEFAULT_bShowAllAngles;
  xv_config.eOUT = _DEFAULT_eOUT;
  for (ix = 0; ix < _N_ANGLES; ix++)                      // initialize the array of displayable angles
    xv_config.aryAngles[ix] = _DEFAULT_bANGLE;
  EEPROM_writeAnything(0, xv_config);                     // write the entire configuration to the EEPROM
  bEEPROM_Initialized = true;                             // remember that we updated EEPROM (used by 'showConfig')
}
/*
 * initSerialCommands - add operator commands to the 'sCmd' object
 */
void initSerialCommands() {
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
  sCmd.addCommand("ShowAngles",  showAngles);
  sCmd.addCommand("HideAngles",  hideDist);
  sCmd.addCommand("ShowAngle",  showAngles);
  sCmd.addCommand("HideAngle",  hideDist);
  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowRaw",  showRaw);
  sCmd.addCommand("ShowErrors", showErrors);
  sCmd.addCommand("HideErrors", hideErrors);
  sCmd.addCommand("ShowError", showErrors);
  sCmd.addCommand("HideError", hideErrors);
  sCmd.addCommand("ShowCSV", showCSV);
  sCmd.addCommand("HideCSV", hideCSV);
  sCmd.addCommand("ShowInterval", showInterval);
  sCmd.addCommand("HideInterval", hideInterval);
  sCmd.addCommand("ShowAll", showAll);
  sCmd.addCommand("HideAll", hideAll);
}
/*
 * showAll - enable all the display modifier commands, i.e., RPM, Errors, Interval
 */
void showAll() {
  showRPM();                                        // enable showRPM
  showErrors();                                     // enable showErrors
  showInterval();                                   // enable showInterval
}
/*
 * hideAll - disable all the display modifier commands, i.e., RPM, Errors, Interval
 */
void hideAll() {
  hideRPM();                                        // disable showRPM
  hideErrors();                                     // disable showErrors
  hideInterval();                                   // disable showInterval
}
/*
 * hideDist - Hide distance data from LIDAR
 */
void hideDist() {
  int eShow = (xv_config.eOUT & _MASK_eOUT_SHOW);
  if (eShow == _eOUT_ANGLE) {                               // you can only hide distance if it is currently active
    xv_config.eOUT &= -1 - _MASK_eOUT_SHOW;                 // keep top 8-bits and turn off bottom 8 bits before setting command
    xv_config.eOUT |= _eOUT_UNKNOWN;                        // set "What we're displaying" to "unknown"       
    EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);
    xv_config.bShowAllAngles = true;                        // set default back to 'show all angles'
    EEPROM_writeAnything(_EE_OFFSET_SHOW_ALL_ANGLES, xv_config.bShowAllAngles);
    Serial.println(F(" "));
    Serial.println(F("Hiding Distance data"));
  }
}
/*
 * showAngles - Multi-angle range(s) implementation - DSH
 * Command: ShowAngle(s) ddd, ddd-ddd, etc. | ALL | all | 360
 * Enter with: N/A
 * Uses:       aryAngles (an array of 360 booleans) gets set to appropriate values
 *             xv_config.bShowAllAngles gets set to true if all angles (0-359) are requested
 * Calls:      showDist
 * Exits with: N/A
 * TEST THIS STRING:  ShowAngles 16-20, 300-305, 123-124, 10
 */
void showAngles() {
  char c, *arg;
  boolean bOneshot = false, bFoundAngle = false, syntax_error = false, bDoAll = false;
  int doing_from_to, from, to, ix, lToken, n_groups = 0;
  unsigned int current_eOUT = xv_config.eOUT;  // save the current value so we can restore the top 8-bits (modifiers)
  //unsigned int currentCommand = current_eOUT & _MASK_eOUT_SHOW; // isolate the command (bottom 8-bits)
  const int lenAll = 4;                                 // length of szAll, including terminating 0
  const char szAll[lenAll] = "ALL";                     // the word "ALL" (in upper-case) is used to match "ShowAngle ALL"
  const char sz360[lenAll] = "360";                     // the value "360" is used to match "ShowAngle 360"
  
  doing_from_to = 0;                                    // state = doing 'from'
  // Make sure that there is at least 1 angle or group of angles present, else try to use angles from EEPROM
  do {
    arg = sCmd.next();                                  // get the next token
    if (arg == NULL) {                                  // it's empty -- just exit
      sCmd.readSerial();
      arg = sCmd.next();
      break;
    }
    lToken = strlen(arg);                               // get the length of the current token
    // is 'arg' = the string "all" ?    
    if (lToken == lenAll - 1) {                         // we have a candidate for match, now check it in detail
      bDoAll = true;
      for (ix = 0; ix < lenAll; ix++) {                 // eschew 'strncmp' because it's stupid about "all" vs. "allxxx"
        if ((arg[ix] & (-1 - ' ')) != szAll[ix]) {      // force to upper-case for match
          bDoAll = false;
          break;      
        }
      } // for (ix = 0; ix < lenAll; ix++)      
      if (bDoAll == false) {                            // it's not "all" -- check for "360" which means the same thing
        bDoAll = true;
        for (ix = 0; ix < lenAll; ix++) {               // eschew 'strncmp' because it's stupid about "360" vs. "3600"
          if (arg[ix] != sz360[ix]) {
            bDoAll = false;
            break;      
          }
        }  // for (ix = 0; ix < lenAll; ix++)      
      } // if (bDoAll == false)
    } // if (lToken == lenAll - 1)
    else 
      bDoAll = false;

    // 'bDoAll' = true means we're supposed to display all angles, else, continue parsing input string
    if (bDoAll)
      break;                                             // break out of the do loop
    // see if the token has an embedded "-", meaning from - to
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
    // validate 'from' and 'to' and set 'aryAngles' with correct values
    if ((from >= 0) && (from < _N_ANGLES) 
    && (to >= 0) && (to < _N_ANGLES)) {
      if (to >= from) {
        if (bOneshot == false) {
          bOneshot = true;        
          for (ix = 0; ix < _N_ANGLES; ix++)              // initialize aryAngles (just once)
            xv_config.aryAngles[ix] = false; 
        }
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
  
  // Either we have 1 or more angles, else we got "ALL" or "360" or we have a syntax error
  if (bDoAll) {
    if (xv_config.bShowAllAngles == false) {                // only rewrite EEPROM if it needs it
      xv_config.bShowAllAngles = true;                      // value if we're showing all angles
      for (ix = 0; ix < _N_ANGLES; ix++)
        xv_config.aryAngles[ix] = true;
      EEPROM_writeAnything(_EE_OFFSET_aryAngles, xv_config.aryAngles);
      EEPROM_writeAnything(_EE_OFFSET_SHOW_ALL_ANGLES, xv_config.bShowAllAngles);
    }
  } // if (bDoAll)
  else {                                                    // if no angles specified, try to use EEPROM values
    bFoundAngle = false;
    if (n_groups == 0) {                                    // no angle groups were specified...
      for (ix = 0; ix < _N_ANGLES; ix++) {                  // ...are any angles stored in EEPROM?
        if (xv_config.aryAngles[ix]) {
          bFoundAngle = true;                               // we found at least 1 angle in EEPROM
          break;
        }
      } // for (ix = 0; ix < _N_ANGLES; ix++)
      if (bFoundAngle == false)                             // no angles found in EEPROM...error
        syntax_error = true;
    } // if (n_groups == 0)
  } // if (bDoAll == false)

  // Handle syntax errors
  if (syntax_error) {
    xv_config.eOUT = (current_eOUT & _MASK_eOUT_MODIFIER);  // restore any modifier bits
    xv_config.eOUT |= _eOUT_UNKNOWN;                        // set "What we're displaying" to "unknown"
    EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);
    // NOTE: We're leaving any previously-set angles and 'bShowAllAngles' alone
    //       Maybe the operator just keyed in a bad value, but no need to punish...
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax. Examples:")); 
    Serial.println(F("  ShowAngle(s) 0, 15-30, 45-50, 10")); 
    Serial.println(F("  ShowAngle(s) all or 360"));     
    Serial.println(F("Syntax rules:")); 
    Serial.println(F("  Use a space after each comma. No particular order (of angles) is required."));     
    Serial.println(F("  In a from-to pair, the 1st value must be lowest."));
  }
  else {                                                    // We're ready to process multiple angles    
    xv_config.eOUT = (current_eOUT & _MASK_eOUT_MODIFIER);  // restore any modifier bits
    xv_config.eOUT |= _eOUT_ANGLE;                          // set "What we're displaying" to "Angle"
    EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);
    if (bDoAll == false) {                                  // if we haven't already written 'bShowAllAngles' and 'aryAngles'...
      xv_config.bShowAllAngles = true;                      // count the angles and set 'bShowAllAngles'
      for (ix = 0; ix < _N_ANGLES; ix++) {
        if (xv_config.aryAngles[ix] == false) {
          xv_config.bShowAllAngles = false;
          break;
        }    
      }
      EEPROM_writeAnything(_EE_OFFSET_SHOW_ALL_ANGLES, xv_config.bShowAllAngles);      
      if (n_groups != 0)
        EEPROM_writeAnything(_EE_OFFSET_aryAngles, xv_config.aryAngles);
    } // if (bDoAll == false)
    
    Serial.println(F(" "));
    Serial.print(F("Angles:"));
    if (xv_config.bShowAllAngles) 
      Serial.print(F("all"));        
    else {
      for (int ix = 0; ix < _N_ANGLES; ix++) {              // display the angle array
        if(xv_config.aryAngles[ix]) {
          Serial.print(ix, DEC);
          Serial.print(F(","));
        }
        else
          xv_config.bShowAllAngles = false;
      } // for (int ix = 0; ix < _N_ANGLES; ix++)
      Serial.println(F(" "));      
    } // if not (xv_config.bShowAllAngles)
    
    Serial.println(F(" "));
    Serial.println(F("Showing Distance data <Angle>: <dist mm> (signal strength)}"));  
    if (!(xv_config.bMotorEnable))                          // Turn the motor ON if it's OFF
      forceMotorOn();
  } // if not (syntax_error)
}
/*
 * forceRaw
 * Update 'eOUT' word as follows:
 * Top 8-bits:    Preserve modifiers {RPM; CSV; ERRORS; INTERVAL, and maybe RFU_1; RFU_2; RFU_3; RFU_4}
 * Bottom 8-bits (the command): Set it to '_eOUT_RAW'
 */
void forceRaw() {
  xv_config.eOUT &= _MASK_eOUT_MODIFIER;                  // Preserve the MOD bits (top 8-bits)
  xv_config.eOUT |= _eOUT_RAW;                            // Set "What we're displaying" to display "raw" data
  EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);
}
/*
 * showRaw - Show "raw" LIDAR data (feed data directly to user)
 */
void showRaw() {
  if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {    // Only do this if it's not already active
    forceRaw();                                             // enable "RAW" mode
    if (!(xv_config.bMotorEnable))                          // Turn the motor ON if it's OFF
      forceMotorOn();                                       // start up the motor
  }
}
/*
 * hideRaw - Hide "raw" LIDAR data
 */
void hideRaw() {
  if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {    // Only do this if we're displaying RAW data
    xv_config.eOUT &= _MASK_eOUT_MODIFIER;                  // Preserve the MOD bits (top 8-bits)
    xv_config.eOUT |= _eOUT_UNKNOWN;                        // Set "What we're displaying" to "unknown"
    EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);  // write 'eOUT' to EEPROM 
    Serial.println(F(" "));
    Serial.println(F("Raw LIDAR data disabled"));
  }
}
/*
 * motorOff - turn the motor OFF
 */
void motorOff() {
  if (xv_config.bMotorEnable) {                               // you can only turn the motor OFF if it's ON
    xv_config.bMotorEnable = false;
    EEPROM_writeAnything(_EE_OFFSET_MOTOR_ENABLE, xv_config.bMotorEnable);
    Timer3.pwm(xv_config.motor_pwm_pin, 0);
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {    // can't display messages if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Motor off"));
    }
  } // if (xv_config.bMotorEnable)
}
/*
 * forceMotorOn - turn the motor on silently
 */
void forceMotorOn() {
  xv_config.bMotorEnable = true;
  EEPROM_writeAnything(_EE_OFFSET_MOTOR_ENABLE, xv_config.bMotorEnable);
  Timer3.pwm(xv_config.motor_pwm_pin, pwm_val);
  rpm_err = 0;                                              // reset rpm error
}
/*
 * motorOn - turn the motor ON
 */
void motorOn() {
  if (xv_config.bMotorEnable == false) {                      // you can only turn the motor ON if it's OFF    
    forceMotorOn();
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {    // can't display messages if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Motor on"));
    }
  }
}
/*
 * motorCheck - Make sure the motor RPMs are good else shut it down
 */
void motorCheck() {
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
void setLowRPM(double sVal) {
  xv_config.rpm_setpoint = sVal;
  EEPROM_writeAnything(_EE_OFFSET_RPM_SETPOINT, xv_config.rpm_setpoint);
}
/*
 * setRPM - set motor RPM
 */
void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;
    
  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);                                         // Converts a char string to a float
    if (sVal < xv_config.rpm_min) {
      sVal = xv_config.rpm_min;
      if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {  // can't do this if we're in RAW mode
        Serial.println(F(" "));
        Serial.print(F("RPM too low. Setting to minimum "));
        Serial.println(xv_config.rpm_min);              
      }
    } // if (sVal < xv_config.rpm_min)
    if (sVal > xv_config.rpm_max) {
      sVal = xv_config.rpm_max;
      if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {  // can't do this if we're in RAW mode
        Serial.println(F(" "));
        Serial.print(F("RPM too high. Setting to maximum "));
        Serial.println(xv_config.rpm_max);        
      }
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
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Incorrect syntax.  Example: SetRPM 200"));       
    }
  }
  else {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("Old RPM setpoint:"));
      Serial.println(xv_config.rpm_setpoint);                
    }
    xv_config.rpm_setpoint = sVal;
    EEPROM_writeAnything(_EE_OFFSET_RPM_SETPOINT, xv_config.rpm_setpoint);
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("New RPM setpoint: "));
      Serial.println(sVal);          
    }
  }
}
/*
 * setKp
 */
void setKp() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);                                         // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Incorrect syntax.  Example: SetKp 1.0")); 
    }
  }
  else {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("Setting Kp to: "));
      Serial.println(sVal);
    }
    xv_config.Kp = sVal;
    EEPROM_writeAnything(_EE_OFFSET_Kp, xv_config.Kp);
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}
/*
 * setKi
 */
void setKi() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);                                         // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Incorrect syntax.  Example: SetKi 0.5")); 
    }
  }
  else {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("Setting Ki to: "));
      Serial.println(sVal);    
    }
    xv_config.Ki = sVal;
    EEPROM_writeAnything(_EE_OFFSET_Ki, xv_config.Ki);
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}
/*
 * setKd
 */
void setKd() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);                                         // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Incorrect syntax.  Example: SetKd 0.001")); 
    }
  }
  else {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("Setting Kd to: "));
      Serial.println(sVal);    
    }
    xv_config.Kd = sVal;
    EEPROM_writeAnything(_EE_OFFSET_Kd, xv_config.Kd);
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}
/*
 * setSampleTime - set sample time (in ms)
 */
void setSampleTime() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);                                         // Converts a char string to an integer
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.println(F("Incorrect syntax.  Example: SetSampleTime 20")); 
    }
  }
  else {
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      Serial.println(F(" "));
      Serial.print(F("Setting Sample time to: "));
      Serial.println(sVal);    
    }
    xv_config.sample_time = sVal;
    EEPROM_writeAnything(_EE_OFFSET_SAMPLE_TIME, xv_config.sample_time);
    rpmPID.SetSampleTime(xv_config.sample_time);
  }
}
/*
 * help - show HELP
 */
void help() {
  if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(xv_config.szVersion);
  Serial.print(F("GetSurreal.com"));

  Serial.println(F(" "));
  Serial.println(F(" "));
  Serial.println(F("NOTE: All commands are case-insensitive"));
  Serial.println(F(" "));
  Serial.println(F("What to display:"));
  Serial.println(F("  ShowRaw       - Enable the output of the raw LIDAR data (default)"));
  Serial.println(F("  HideRaw       - Stop outputting the raw data from the LIDAR"));
  //Serial.println(F("  ShowDist      - Show the distance data"));
  //Serial.println(F("  HideDist      - Hide the distance data"));
  Serial.println(F("  ShowAngle(s)  - Show distance data for multiple angles (Ex: ShowAngle 0, 15-30, 45-50, 10)"));
  Serial.println(F("  HideAngle(s)  - Hide distance data for all angles"));
  Serial.println(F(" "));
  Serial.println(F("Display modifiers:"));
  Serial.println(F("  ShowAll       - Show Errors + Interval + RPM"));
  Serial.println(F("  HideAll       - Suppress Errors + Interval + RPM"));
  Serial.println(F("  ShowError(s)  - Show all error messages"));
  Serial.println(F("  HideError(s)  - Hide error messages"));
  Serial.println(F("  ShowInterval  - Show time interval per revolution in ms"));
  Serial.println(F("  HideInterval  - Hide time interval"));
  Serial.println(F("  ShowRPM       - Show the motor speed (RPM)"));
  Serial.println(F("  HideRPM       - Hide the motor speed"));
  Serial.println(F("  ShowCSV       - Show data in comma-separated variable (.csv) format"));
  Serial.println(F("  HideCSV       - Hide csv format"));
  Serial.println(F(" "));  
  Serial.println(F("Other commands:"));
  Serial.println(F("  MotorOff      - Stop spinning the LIDAR"));
  Serial.println(F("  MotorOn       - Enable spinning of the LIDAR"));
  Serial.println(F("  SetRPM        - Set the desired rotation speed (min: 200 (default); max: 300)"));
  Serial.println(F("  ShowConfig    - Show the running configuration"));
  Serial.println(F("  SaveConfig    - Save the running configuration to EEPROM"));
  Serial.println(F("  ResetConfig   - Restore the original configuration"));
  Serial.println(F("  SetKp         - Set the proportional gain"));
  Serial.println(F("  SetKi         - Set the integral gain"));
  Serial.println(F("  SetKd         - Set the derivative gain"));
  Serial.println(F("  SetSampleTime - Set the frequency the PID is calculated (ms)"));
  Serial.println(F(" "));
  Serial.println(F("Output comma-separated variable (CSV) format:"));
  Serial.println(F("  <Angle>,<Distance in mm>,<Strength>"));
  Serial.println(F("  Time interval ms: <Time interval in milliseconds>"));
  Serial.println(F(" "));
  Serial.println(F("Errors:"));
  Serial.println(F("  CRC means CRC error in LIDAR packet"));
  Serial.println(F("  {I}nvalid = LIDAR reports invalid data for this angle"));
  Serial.println(F("  {S}ignal = LIDAR reports poor signal strength for this angle"));
  Serial.println(F(" "));
}
/*
 * showConfig - show the current EEPROM configuration
 */
void showConfig() {
  int kx = 0;
  byte *ptr_xv_config = &xv_config.id;
  
  if ((xv_config.eOUT & _MASK_eOUT_SHOW) == _eOUT_RAW)    // if we're currently in RAW mode...
    hideRaw();
  Serial.println(F(" "));
  Serial.println(F(" "));
  if (bEEPROM_Initialized) {
    Serial.println(F("EEPROM was recently updated"));
    bEEPROM_Initialized = false;
  }
  if (!(xv_config.safety == _EEPROM_SAFETY_CHECK)) {
    Serial.println(F(" EEPROM is invalid ('safety'). Contact the programmer."));
    for (int ix = 0; ix < _EEPROM_Config_LEN; ix++) {
      if (*ptr_xv_config < 0x10)
        Serial.print(F("0"));
      Serial.print(*ptr_xv_config++, HEX);
      Serial.print(F(" "));
      if ((ix & 0x0F) == 0x0F)                      // CRLF after 16 bytes displayed
        Serial.println(F(" "));
    }
    Serial.println(F(" "));
    while(1);                                       // HANG!!!!!!
  }    
  Serial.println(F(" "));
  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.print(xv_config.szVersion);
  Serial.print(F(" "));
  Serial.println(F("GetSurreal.com"));  
  Serial.println(F(" "));
  Serial.print(F("EEPROM id: "));                    // check 1st and last bytes of EEPROM
  Serial.print(xv_config.id, HEX);
  Serial.print(F("  EEPROM safety: "));
  Serial.println(xv_config.safety, HEX);

  Serial.print(F("PWM pin: "));
  Serial.println(xv_config.motor_pwm_pin);
  Serial.print(F("Target RPM: "));
  Serial.println(xv_config.rpm_setpoint);
  Serial.print(F("Min RPM: "));
  Serial.println(xv_config.rpm_min);
  Serial.print(F("Max RPM: "));
  Serial.println(xv_config.rpm_max);
  Serial.print(F("Min PWM: "));
  Serial.println(xv_config.pwm_min);
  Serial.print(F("Max PWM: "));
  Serial.println(xv_config.pwm_max);
  Serial.print(F("PID Kp: "));
  Serial.println(xv_config.Kp);
  Serial.print(F("PID Ki: "));
  Serial.println(xv_config.Ki);
  Serial.print(F("PID Kd: "));
  Serial.println(xv_config.Kd);
  Serial.print(F("SampleTime: "));
  Serial.println(xv_config.sample_time);
  
  Serial.print(F("Motor:"));  
  if (xv_config.bMotorEnable)
    Serial.println(F("ON"));
  else
    Serial.println(F("OFF"));
  /*
   * What to display, e.g., {Unknown; Raw; Distance; Angle}
   */
  Serial.print(F("What to display:"));    
  switch (xv_config.eOUT & _MASK_eOUT_SHOW) {
    case _eOUT_UNKNOWN :
      Serial.println(F("(uninitialized)"));
      break;
    case _eOUT_RAW :
      Serial.println(F("Show Raw Data"));
      break;
    case _eOUT_ANGLE :
      Serial.println(F("Show Angle"));
      break;
    default :
      Serial.println(F("ERROR @ showConfig: EEPROM 'eOUT' value is invalid: Contact programmer"));
      while(1);                                          // HANG!!!!!!!!!!
  }  
  /*  
   * Display the modifiers, e.g., {RPM, CSV, Errors, Interval}
   */
  Serial.print(F("Show RPM Data:"));
  if (xv_config.eOUT & _OUT_MOD_SHOW_RPM)
    Serial.println(F("Yes"));
  else
    Serial.println(F("No"));
  
  Serial.print(F("Display format:"));  
  if (xv_config.eOUT & _OUT_MOD_SHOW_CSV)  
    Serial.println(F("comma-separated variables (CSV)"));
  else
    Serial.println(F("normal"));    

  Serial.print(F("Show errors:"));
  if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS)
    Serial.println(F("Yes"));
  else
    Serial.println(F("No"));
  Serial.print(F("Show interval:"));
  if (xv_config.eOUT & _OUT_MOD_SHOW_INTERVAL)
    Serial.println(F("Yes"));
  else
    Serial.println(F("No"));
  /*    
   * Show all angles?
   */
  Serial.print(F("Show all angles:"));
  if (xv_config.bShowAllAngles)
    Serial.println(F("Yes"));
  else {
    Serial.println(F("No, just these angles:"));
    for (int ix = 0; ix < _N_ANGLES; ix++) {
      if (xv_config.aryAngles[ix] == true) {
        Serial.print(ix);
        Serial.print(F(", "));
        kx++;
        if (kx == 20) {                            // Max # of angles per line
          Serial.println(F(" "));                  // then do a CR/LF
          kx = 0;
        }
      }
    }  // for (int ix = 0; ix < _N_ANGLES; ix++)
    Serial.println(F(" "));  
  }   
}
/*
 * saveConfig - save the current configuration to EEPROM
 */
void saveConfig() {    
  EEPROM_writeAnything(0, xv_config);
  if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
    Serial.println(F("Config Saved."));  
  }  
}
/*
 * The next section deals with enabling and disabling the display modifiers, i.e., show RPM, CSV, Errors, Interval
 * These functions are controlled by bits in the msb 8-bits of 'eOUT'
 * NOTE: All these functions are suppressed if we're in RAW mode (we don't want to destroy pass-thru data stream)
 */
/* 
 * showRPM - interject the RPM data into the output stream
 * NOTE: Can't do tis if displaying Raw data
 */
void showRPM() {
  if (!(xv_config.eOUT & _OUT_MOD_SHOW_RPM))                // if we're not already showing RPM data...    
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode      
      xv_config.eOUT |= _OUT_MOD_SHOW_RPM;                  // ...enable RPM display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);        
      Serial.println(F(" "));
      Serial.println(F("Showing RPM data"));
    }  
}
/*
 * showCSV - show the data in comma-separated variable (.csv) format
 * NOTE: Can't do tis if displaying Raw data
 */
void showCSV() {
  if (!(xv_config.eOUT & _OUT_MOD_SHOW_CSV))                // if we're not already using CSV format ...
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT |= _OUT_MOD_SHOW_CSV;                  // ...enable CSV format
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);    
      Serial.println(F(" "));
      Serial.println(F("Showing data in CSV format"));            
    }
}
/*
 * showErrors - show errors
 * NOTE: Can't do tis if displaying Raw data
 */
void showErrors() {
  if (!(xv_config.eOUT & _OUT_MOD_SHOW_ERRORS))              // if we're not already showing errors...    
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT |= _OUT_MOD_SHOW_ERRORS;               // ...enable error display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);    
      Serial.println(F(" "));
      Serial.println(F("Showing errors"));
    }
}
/*
 * showInterval
 * NOTE: Can't do tis if displaying Raw data
 */
void showInterval() {
  if (!(xv_config.eOUT & _OUT_MOD_SHOW_INTERVAL))           // if we're not already showing invervals...
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT |= _OUT_MOD_SHOW_INTERVAL;             // ...enable interval display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);    
      Serial.println(F(" "));
      Serial.println(F("Show interval"));
    }
}
/* 
 * hideRPM
 * NOTE: Can't do tis if displaying Raw data
 */
void hideRPM() {
  if (xv_config.eOUT & _OUT_MOD_SHOW_RPM)                   // if we're showing RPM data...
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT &= -1 -_OUT_MOD_SHOW_RPM;              // ...disable RPM display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);
      Serial.println(F(" "));
      Serial.println(F("Hiding RPM data"));
    }
}
/*
 * hideCSV - stop displaying in CSV format
 * NOTE: Can't do tis if displaying Raw data
 */
void hideCSV() {
  if (xv_config.eOUT & _OUT_MOD_SHOW_CSV)                   // if we're using CSV format...
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT &= -1 - _OUT_MOD_SHOW_CSV;             // .. disable interval display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);        
      Serial.println(F(" "));
      Serial.println(F("Showing data in normal format"));                
    }    
} 
/*
 * hideErrors
 * NOTE: Can't do tis if displaying Raw data
 */
void hideErrors() {                                 
  if (xv_config.eOUT & _OUT_MOD_SHOW_ERRORS)                // if we're showing errors...        
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT &= -1 - _OUT_MOD_SHOW_ERRORS;          // .. disable error display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);    
      Serial.println(F(" "));
      Serial.println(F("Hiding errors"));
    }
}
/*
 * hideInterval
 * NOTE: Can't do tis if displaying Raw data
 */
void hideInterval() {                                 
  if (xv_config.eOUT & _OUT_MOD_SHOW_INTERVAL)              // if we're showing invervals...
    if ((xv_config.eOUT & _MASK_eOUT_SHOW) != _eOUT_RAW) {  // can't do this if we're in RAW mode
      xv_config.eOUT &= -1 - _OUT_MOD_SHOW_INTERVAL;        // .. disable interval display
      EEPROM_writeAnything(_EE_OFFSET_eOUT, xv_config.eOUT);        
      Serial.println(F(" "));
      Serial.println(F("Hiding interval display"));    
    }  
}
/**/
