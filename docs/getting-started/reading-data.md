# Reading Data

This is one method for taking a look at the data, but this is by no means the only way.  We're going to use the Arduino tools because there's a great community of support.

Here are some reasons for reading the data like this.

* You just got the lidar and want to see some clear text data with angles and distances coming from the lidar.
* You want to do some troubleshooting.
* You want to check out some of the built in API commands on the controller.

### Software Requirements

Each version of Arduino has a corresponding version of Teensyduino.  Be sure to use the correct combination.

[Arduino IDE - https:\/\/www.arduino.cc\/en\/Main\/Software](https://www.arduino.cc/en/Main/Software)

[Teensyduino by PJRC - http:\/\/www.pjrc.com\/teensy\/td\_download.html](http://www.pjrc.com/teensy/td_download.html)

### Get Connected

* Plug the 2 pin motor connector and the 4 pin serial connector to the XV Lidar Controller.

* Plug a USB cable from the XV Lidar Controller to your computer where you installed the software.


### View the Data

Launch the Arduino IDE.

1. Set the Board to Teensy 2.0
2. Set the serial port to the port that appears after you plugin to the Teensy
3. Open the Serial Monitor
4. At the bottom window of the Serial Monitor change the settings to “NewLine” and “115200 baud”
5. The Serial monitor should be displaying lots of garbled data\*
6. Type RelayOff and click “Send” – This will stop all the garbled data
7. Type ShowRPM and click “Send” – Streaming RPM data should now start with values close to 300

\*Garbled Data – The data streaming from the XV Lidar is not in a human readable format. The data must be parsed to read the distance data. For examples on how to decode the data stream take a look at the [XV Lidar Controller firmware](https://github.com/getSurreal/XV_Lidar_Controller) fo an C++\/Arduino example or for a python example take a look code in the [Visualizing Data Getting Started guide](/getting-started/viewing-data.md).

