# ROS Node: AHRS

**Original Author:** Midgwick

**Current Author:** Kevin Walchko

**License:** GPL Ver. 2

**Language:** C++

**Website:** http://github.com/walchko/ahrs

This implements AHRS code written by [Midgwick](http://github.com/walchko/ahrs/tree/master/docs/madgwick.pdf) designed to take the
readings from accelerometers, gyros, and magnetometers and calculate the pose (roll,
pitch, and yaw) of the sensor.

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

	rosrun ahrs ahrs

## Subscribed Topics:

**IMU Message**

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

A QGLViewer that takes inputs from MiniIMU9 and displays the pose of the sensor in
real-time.

## Command Line

	rosrun ahrs ahrs_view

## Example:

 	rosrun ahrs ahrs_view
 	
![imu_pose_image]()

## To Do

* something

# Test

![test](http://youtu.be/cxBDABA8aSM)

<iframe width="420" height="315" src="http://www.youtube.com/embed/cxBDABA8aSM" frameborder="0" allowfullscreen></iframe>

<object width="480" height="385"><param name="movie" value="http://www.youtube.com/v/jJECepNeCJ0&amp;hl=en_US&amp;fs=1"></param><param name="allowFullScreen" value="true"></param><param name="allowscriptaccess" value="always"></param><embed src="http://www.youtube.com/v/jJECepNeCJ0&amp;hl=en_US&amp;fs=1" type="application/x-shockwave-flash" allowscriptaccess="always" allowfullscreen="true" width="480" height="385"></embed></object>