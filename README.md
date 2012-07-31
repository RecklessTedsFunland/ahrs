# ROS Node: AHRS

**Original Authors:** Willow Garage

**Current Author:** Kevin Walchko

**License:** GPL Ver. 2

**Language:** C++

**Website:** http://github.com/walchko/ahrs

This implements AHRS code written by [Midgwick](http://github.com/walchko/ahrs/tree/master/docs/madgwick.pdf) designed to take the
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

The filter is implemeted as:

![image](http://upload.wikimedia.org/wikipedia/commons/thumb/c/c3/Biquad_filter_DF-I.svg/500px-Biquad_filter_DF-I.svg.png)

## Octave

In the octave directory are several m-files used to design the IIR filter. From the
Octave commandline type:

1. load_imu
2. makefilter

This will load the data from test.txt which is just a csv file. Then the the order of 
the filter is determined and the filter is designed.

## Command Line

	rosrun ahrs ahrs

## Subscribed Topics:

** IMU Message **

"/imu" is the IMU message from the soccer robot package 
([Imu.msg](http://github.com/walchko/soccer/blob/master/msg/Imu.msg))


#### Published Topics: 

**something:** 

"/imu/something" some data

## To Do

* optimize for a certain Hz
* clean up octave scripts

# Viewer Node: IMU Viewer

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/MiniIMU9/viewer

A QGLViewer that takes inputs from MiniIMU9 and displays the pose of the sensor.

## Command Line

	rosrun viewer ahrs_view -c /my/imu

* c: imu

## Example:

 	rosrun viewer ahrs_view -c "/imu/data"

## To Do

* something

