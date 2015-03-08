XV Lidar Controller
===================

Control the Neato XV Lidar with an Arduino compatible board

Used as an interface board to connect directly to the Neato XV Lidar and control the rotation speed through Pulse Width Modulation (PWM).

Copyright 2014 James LeRoy getSurreal.com
v1.2.2 - Updated 2015/02/23
* http://www.getsurreal.com/products/xv-lidar-controller
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
* Arduino IDE (v1.0.6 tested. Newer or older may work)
* Teensyduino - Software add-on to run Arduino sketches on the Teensy (v1.19 tested. Newer or older may work)
 http://www.pjrc.com/teensy/teensyduino.html
* Copy the included libraries to the Arduino libraries directory

##Usage
Connect to the Teensy USB port at 115200 baud.  When sending commands use the newline character to signify the end of a command.

##Commands
Commands are case sensitive.
* Help - Show the list of commands available
* ShowConfig    - Show the running configuration
* SaveConfig    - Save the running configuration to EEPROM
* ResetConfig   - Restore the original configuration
* SetRPM        - Set the desired rotation speed (min: 200, max: 300)
* SetKp         - Set the proportional gain
* SetKi         - Set the integral gain
* SetKd         - Set the derivative gain
* SetSampleTime - Set the frequency the PID is calculated (ms)
* ShowRPM       - Show the rotation speed
* HideRPM       - Hide the rotation speed
* ShowDist      - Show the distance data
* HideDist      - Hide the distance data
* ShowAngle     - Show distance data for a specific angle (0 - 359 or 360 for all)
* MotorOff      - Stop spinning the lidar
* MotorOn       - Enable spinning of the lidar
* HideRaw       - Stop outputting the raw data from the lidar
* ShowRaw       - Enable the output of the raw lidar data

