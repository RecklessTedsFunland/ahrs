# ROS Node: AHRS

**Original Authors:** Willow Garage

**Current Author:** Kevin Walchko

**License:** GPL Ver. 2

**Language:** C++

**Website:** http://github.com/walchko/ahrs

This implements AHRS code written by [Midgwick](http://www.github.com) designed to take the
readings from accelerometers, gyros, and magnetometers and calculate the pose (roll,
pitch, and yaw) of the sensor.

The data come from the MiniIMU9 Ver 1 from [Pololu](http://www.pololu.com). The IMU has:

* L3G4200D 3-axis gyro
* LSM303DLM 3-axis accelerometer and 3-axis magnetometer

The node currently grabs IMU data at *XXX* Hz and filters it with a Chebyshev Type II
Infinite Impulse Response (IIR) filter. The Type II has a flat pass band response and a 
ripple in the stop band. The filter is designed for:

* Sample Freq: 20 Hz
* Pass band: XX
* Stop band: YY
* Stop band Attenuation: 60 dB
* Order: 5


### Command Line

	rosrun ahrs ahrs

#### Published Topics: 
**something:** 

"/imu/something" some data

### To Do

* optimize for a certain Hz

## Viewer Node: IMU

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/MiniIMU9/viewer

A QGLViewer that takes inputs from MiniIMU9 and displays the pose of the sensor.

### Command Line

	rosrun viewer ahrs_view -c /my/imu

* c: imu

#### Example:

 	rosrun viewer ahrs_view -c "/imu/data"

### To Do

* something

