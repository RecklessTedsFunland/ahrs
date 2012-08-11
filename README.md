# ROS Node: AHRS

**Original Author:** Midgwick

**Current Author:** Kevin Walchko

**License:** GPL Ver. 2

**Language:** C++

**Website:** http://github.com/walchko/ahrs

This implements AHRS code written by [Midgwick](http://github.com/walchko/ahrs/tree/master/docs/madgwick.pdf) designed to take the
readings from accelerometers, gyros, and magnetometers and calculate the pose (roll,
pitch, and yaw) of the sensor.

The node does:

* Digital noise filtering
* Bias correction
* Calculates the roll, pitch, and yaw
* Calculates the compass heading (yaw is wrt North)

## IMU

![IMU](http://i1268.photobucket.com/albums/jj568/mars_university/blog/MinIMU-9-Ver1.png)

The data comes from the MiniIMU9 Ver 1 from [Pololu](http://www.pololu.com) which
exposes an I2C interface to get the data. The Inertial Measurement Unit (IMU) has
the following sensors:

* L3G4200D 3-axis gyro (16b reading)
* LSM303DLM 3-axis accelerometer and 3-axis magnetometer (both 12b readings)

## Digital Filter

The AHRS node currently grabs IMU data at 20 Hz (but that is changeable) and filters it with a Chebyshev Type II
Infinite Impulse Response (IIR) filter. The Type II has a flat pass band response and a 
equiripple in the stop band. The filter is designed for:

* Sample Freq: 20 Hz
* Pass band: 0.8
* Stop band: 0.9
* Stop band Attenuation: 40 dB
* Order: 5

The filter is implemented as the following difference equation:

![image](http://i1268.photobucket.com/albums/jj568/mars_university/blog/filter-direct.png)

## Octave

The digital filter was developed using Octave. In the octave directory are several m-files used to design the IIR filter. From the
Octave command line type:

1. load_imu
2. imu_filter

This will load the data from test.txt which is just a csv file. Then the the order of 
the filter is determined and the filter is designed.

![filter_response](http://i1268.photobucket.com/albums/jj568/mars_university/blog/filter-response.png "title")

## Command Line

	rosrun ahrs ahrs --no-filter --no-mags

**no-filter** Turns off the IIR filter

**no-mags** Turns off the magnetometers when determining the heading. This is useful when 
the AHRS magnetometers have not been compensated for hard iron distortion or the 
magnetic fields from motors are interfering. The AHRS will also no longer have a sense
where magnetic North is, so yaw will be reported w.r.t. some initial arbitrary heading. 

## Subscribed Topics:

**imu** is the IMU message from the soccer robot package 
([Imu.msg](http://github.com/walchko/soccer/blob/master/msg/Imu.msg))

**imu_reset** (bool) resets the AHRS when true sent


## Published Topics: 

**imu_out** ([sensor_msgs/Imu](http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html)) 

Output is the filtered data with the magnetometer, gyros and accelerometers
data fused into an orientation quaternion 

## To Do

* optimize for a specific sampling rate (Hz)
* clean up octave scripts
* turn IIR filter on or off
* turn off and on the magnetometers
* have it run through a scripted routine where it out puts accel data when the 
IMU is rotated through 6 different orientations. Routine displays a count down (
3,2,1) to re-orient the IMU and then it averages 1 min of data in that orientation
before the next count down begins. The average accel data is printed to screen.

------------------------------------------------------------------------------------

# Viewer Node: IMU Viewer

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/MiniIMU9/viewer

![window](http://i1268.photobucket.com/albums/jj568/mars_university/ahrs.png)

A [QGLViewer](www.libqglviewer.com) that takes inputs from MiniIMU9 and displays 
the pose of the sensor in real-time. The viewer window takes the following
input commands:

* ESC: close window
* Mouse Drag: spin
* Mouse Scroll: zoom in and out

Inertial axis with a cube representing the IMU orientation.

## Subscribed Topics

**imu_out** ([sensor_msgs/Imu](http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html)) 

## Command Line

	rosrun ahrs ahrs_view

## To Do

* Still just getting this working, need to get things working completely

