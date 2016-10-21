# Troubleshooting

### The lidar never starts spinning.

Verify you can easily hand turn the lidar.  If not, see "Cannot hand turn the lidar" below.

Verify the lidar motor is good.  See "Testing the lidar motor" below.

### The lidar starts spinning and then shuts down after a few seconds.

This typically means that the lidar controller is not receiving data from the lidar.  This is a failsafe by design.  Once the lidar starts spinning it will start transmitting data via the serial connection.  The xv lidar controller looks for this data to extract the RPM information in order to steadily spin the lidar.  If there's no data then it shutsdown the motor to prevent possible damage in case there is an object obstructing the lidar.  Make sure all the wire connections are secure and inspect the wires for any damage.

### Cannot hand turn the lidar.

Sometimes the belt will get out of alignment.  Remove the top 4 screws from the lidar and adjust the belt.

### Testing the lidar motor.

The most common failure of the XV Lidar is the motor.  Eventually the brushes in the motor will fail and need to be replaced.  Disconnect the belt from the pully attached to the motor.  Find a 3-12v battery source with wire leads.  Touch the battery leads to the motor connectors for just a brief moment to see if the motor will spin.  If it spins the motor, you're good.  If not look to replace the motor.  There are some vendors on eBay selling replacement motors.

### Not getting readings from far distances

The lidar's max range 6m.  Six meters is in perfect conditions with the right type of of object that will reflect the laser back.  To get the best performance out of your lidar ensure that the laser and lens are dust free.

### Not getting readings from close distances

The lidar's minimum range is 15cm.  Objects closer than 15cm are too close to reflect the laser back to the lens at the proper angle.

