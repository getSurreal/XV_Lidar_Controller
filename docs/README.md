# Introduction

The XV Lidar Controller is designed to be an interface between Neato XV Lidar and your project. The board connects directly to the Neato XV Lidar, receives the serial data from the XV Lidar, controls of the rotation speed with a PID loop by reading the the RPM data embedded in the stream and relays all the Lidar data through the USB connection for an upstream host device \(PC, Raspberry Pi, BeagleBone, etc.\) for processing the data.

There are a number of projects that process the data from the XV Lidar, such as ROS, Breezy Slam, EZ-Robot and more. Anything that can process the XV Lidar Data via USB can use the XV Lidar Controller as its connection to the lidar.

