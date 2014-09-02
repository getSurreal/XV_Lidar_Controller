XV Lidar Controller
===================

Control the Neato XV Lidar with an Arduino compatible board

Used as an interface board to connect directly to the Neato XV Lidar and control the rotation speed through Pulse Width Modulation (PWM).

v1.1.1 Copyright 2014 James LeRoy getSurreal.com
* http://www.getsurreal.com/arduino/xv_lidar_controller
* https://github.com/getSurreal/XV_Lidar_Controller

Based on the following work: 
* Nicolas "Xevel" Saugnier https://github.com/bombilee/NXV11/
* Cheng-Lung Lee https://github.com/bombilee/NXV11/tree/master/ArduinoMegaAdapter


##Description
The XV Lidar Controller receives the serial data from the XV Lidar looking for the RPM data embedded in the stream and uses a PID controller to regulate the speed to 300RPMs.  The data received from the Lidar is relayed to the USB connection for some upstream host device (PC, BeagleBone, Raspberry Pi) to process the data.

##Requirements

###Hardware
* Neato XV Lidar - Available on eBay
* Teensy 2.0 http://www.pjrc.com/teensy
* XV Lidar Controller Board by getSurreal http://www.getsurreal.com/xv-lidar-controller
* Firmware https://github.com/getSurreal/XV_Lidar_Controller


###Software to build from source
* Arduino IDE (v1.0.5 tested. Newer or older may work)
* Teensyduino - Software add-on to run Arduino sketches on the Teensy (v1.19 tested. Newer or older may work)
 http://www.pjrc.com/teensy/teensyduino.html
* Copy the included libraries to the Arduino libraries directory

##Usage
Connect to the Teensy USB port at 115200 baud.  When sending commands use newline to signify the end of a command.  Before sending most commands it's helpful to stop the streaming of serial data with RelayOff.

##Commands
Commands are case sensitive.
* Help
* GetConfig
* SaveConfig
* ResetConfig
* MotorOff
* MotorOn
* RelayOff
* RelayOn
* SetRPM
* SetKp
* SetKi
* SetKd
* SetSampleTime
* ShowRPM
* HideRPM
