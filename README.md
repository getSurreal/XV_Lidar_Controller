XV Lidar Controller
===================

Control the Neato XV Lidar with an Arduino compatible board

Used as an interface board to connect directly to the Neato XV Lidar and control the rotation speed through Pulse Width Modulation (PWM).

Copyright 2014 James LeRoy getSurreal.com
v1.3.0 - Updated 2016/03/20
* http://www.getsurreal.com/products/xv-lidar-controller
* https://github.com/getSurreal/XV_Lidar_Controller

Based on the following work: 
* Nicolas "Xevel" Saugnier https://github.com/bombilee/NXV11/
* Cheng-Lung Lee https://github.com/bombilee/NXV11/tree/master/ArduinoMegaAdapter

Contributions from:
* Doug Hilton mailto: six.speed (at) yahoo (dot) com

##Description
The XV Lidar Controller receives the serial data from the XV Lidar looking for the RPM data embedded in the stream and uses a PID controller to regulate the speed to 300RPMs.  The data received from the Lidar is relayed to the USB connection for some upstream host device (PC, BeagleBone, Raspberry Pi) to process the data.

##Requirements

###Hardware
* Neato XV Lidar - Available on eBay
* XV Lidar Controller Board v1.2 with Teensy 2.0 by getSurreal http://www.getsurreal.com/xv-lidar-controller
* XV Lidar Controller Board v1.3 with Arduino Pro Micro Clone by getSurreal http://www.getsurreal.com/xv-lidar-controller
* Firmware https://github.com/getSurreal/XV_Lidar_Controller


###Software to build from source
* Arduino IDE v1.6.6 - v1.6.8
* For Teensy 2.0 board - Teensyduino v1.27 - v1.28 http://www.pjrc.com/teensy/teensyduino.html


##Usage
Connect to the USB port at 115200 baud.  When sending commands use the newline character to signify the end of a command.

##Commands
  
###Control commands
* ShowConfig    - Show the running configuration
* SaveConfig    - Save the running configuration to EEPROM
* ResetConfig   - Restore the original configuration
* SetAngle      - Show distance data for a multiple angles (Ex: SetAngle 0, 15-30, 45-50, 10)
* SetRPM        - Set the desired rotation speed (min: 180, max: 349)
* MotorOff      - Stop spinning the lidar
* MotorOn       - Enable spinning of the lidar

  
###Data commands
* ShowRaw       - Enable the output of the raw lidar data (default)
* HideRaw       - Stop outputting the raw data from the lidar
* ShowDist      - Show angles with distance data
* HideDist      - Hide the distance data
* ShowErrors    - Show all error types (CRC, Signal Strength, and Invalid
* HideErrors    - Hide angles with errors
* ShowRPM       - Show the rotation speed
* HideRPM       - Hide the rotation speed
* ShowInterval  - Show time interval per revolution in ms, at angle=0
* HideInterval  - Hide time interval
* ShowAll       - Show the distance, errors, RPMs and interval data
* HideAll       - Hide the distance, errors, RPMs and interval data

  
###PID commands
* SetKp         - Set the proportional gain
* SetKi         - Set the integral gain
* SetKd         - Set the derivative gain
* SetSampleTime - Set the frequency the PID is calculated (ms)

  
###Output comma-separated format:
* A,{Angle},{Distance in mm},{Signal Strength}
* C,CRC
* R,{RPMs},{PWM value}
* T,{Time interval in milliseconds between each angle 0}

  
##Errors:
* CRC = Data id not pass CRC check
*   I = LIDAR reports Invalid data for this angle
*   S = LIDAR reports Poor signal strength for this angle
  

