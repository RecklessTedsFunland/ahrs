/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 8/1/2012
 *********************************************************************
 *
 * Simple 3D viewer, see https://github.com/walchko/ahrs for more info.
 *
 * Change Log:
 *  1 Aug 2012 Created
 *
 **********************************************************************
 *
 *
 *
 */


//----------- C++ -------------
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
#include <iostream>

//------------ Boost -----------
#include <boost/thread/mutex.hpp>

//------------ ROS --------------
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

////////////////////////////////////////////////////////////////
#include <qapplication.h>
#include <QGLViewer/qglviewer.h>


const float XUP[3] = {1,0,0}, XUN[3] = {-1, 0, 0},
      		YUP[3] = {0,1,0}, YUN[3] = { 0,-1, 0},
      		ZUP[3] = {0,0,1}, ZUN[3] = { 0, 0,-1},
      		ORG[3] = {0,0,0};


/**
 * Displays a simple 3D cube to represent the pose of the sensor.
 */
class Viewer : public QGLViewer {
public:

	Viewer(ros::NodeHandle& n) {
		sub = n.subscribe("imu_out", 100, &Viewer::imu_callback,this);
		
		angle = y = z = 0.0;
		x = 1.0;
	}
	//virtual ~Viewer();
	
protected :
	
	void Draw_Box (const float size)
	{
		glBegin (GL_QUADS);
	
		  glColor3f  ( 0.0,  0.7, 0.1);     // top - green
		  glVertex3f (-size,  size, size);
		  glVertex3f ( size,  size, size);
		  glVertex3f ( size, -size, size);
		  glVertex3f (-size, -size, size);
	
		  glColor3f  ( 0.9,  1.0,  0.0);    // bottom  - yellow
		  glVertex3f (-size,  size, -size);
		  glVertex3f ( size,  size, -size);
		  glVertex3f ( size, -size, -size);
		  glVertex3f (-size, -size, -size);
	
		  glColor3f  ( 0.2, 0.2,  1.0);     // left - blue 
		  glVertex3f (-size, size,  size);
		  glVertex3f ( size, size,  size);
		  glVertex3f ( size, size, -size);
		  glVertex3f (-size, size, -size);
	
		  glColor3f  ( 0.7,  0.0,  0.1);    // front - red
		  glVertex3f (-size, -size,  size);
		  glVertex3f ( size, -size,  size);
		  glVertex3f ( size, -size, -size);
		  glVertex3f (-size, -size, -size);
	
		glEnd();
	}
	
	
	float roll(const geometry_msgs::Quaternion& q){ 
		float q0,q1,q2,q3;
		q0 = q.w;
		q1 = q.x;
		q2 = q.y;
		q3 = q.z;
		return 180.0/M_PI*atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); 
	}
	
	float pitch(const geometry_msgs::Quaternion& q){ 
		float q0,q1,q2,q3;
		q0 = q.w;
		q1 = q.x;
		q2 = q.y;
		q3 = q.z;
		return -180.0/M_PI*asin(2.0*q1*q3+2.0*q0*q2);
	}
	
	float yaw(const geometry_msgs::Quaternion& q){ 
		float q0,q1,q2,q3;
		q0 = q.w;
		q1 = q.x;
		q2 = q.y;
		q3 = q.z;
		return 180.0/M_PI*atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0); 
	}
	
	geometry_msgs::Vector3 rollPitchYaw(const geometry_msgs::Quaternion& q){ 
		geometry_msgs::Vector3 v;
		double q0,q1,q2,q3;
		q0 = q.w;
		q1 = q.x;
		q2 = q.y;
		q3 = q.z;
		
		v.x = 180.0/M_PI*atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); 
		v.y = -180.0/M_PI*asin(2.0*q1*q3+2.0*q0*q2);
		v.z = 180.0/M_PI*atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0);
		
		return v;
	}

  virtual void draw(){

    glPushMatrix ();
       glRotatef(angle*180.0/M_PI,x,y,z);
       drawAxis(0.5);
       Draw_Box (0.2);
    glPopMatrix ();
    
    geometry_msgs::Vector3 v = rollPitchYaw(q);
    char buff[256];
    sprintf(buff,"Roll: %3.2f Pitch: %3.2f Yaw: %3.2f ",v.x,v.y,v.z);
    //sprintf(buff,"Roll: %3.2f \nPitch: %3.2f \nYaw: %3.2f ",roll(q),pitch(q),yaw(q));
    std::string rpy = buff;
    
	glColor3f  ( 0.9,  0.9,  0.9); 
    drawText(10,60,rpy.c_str());

  }
  
  virtual void init(){
		restoreStateFromFile();
		glDisable(GL_LIGHTING);
		setGridIsDrawn(false);
		//help(); 
		setAnimationPeriod(0); // fast as possible
		startAnimation();
		showEntireScene();
		
		// Make world axis visible
		setAxisIsDrawn(true);
		
		// Move camera according to viewer type (on X, Y or Z axis)
		camera()->setPosition(qglviewer::Vec(2.0,2.0,2.0));
		camera()->setUpVector(qglviewer::Vec(0.0,0.0,1.0));
		camera()->lookAt(sceneCenter());
		
		/*
		// Forbid rotation
		qglviewer::WorldConstraint* constraint = new qglviewer::WorldConstraint();
		constraint->setRotationConstraintType(qglviewer::AxisPlaneConstraint::FORBIDDEN);
		camera()->frame()->setConstraint(constraint);
		*/
	}
  virtual void animate(){
		ros::spinOnce();
	}
	
  virtual QString helpString() const {
	  QString text("<h2>A n i m a t i o n</h2>");
	  text += "Use the <i>animate()</i> function to implement the animation part of your ";
	  text += "application. Once the animation is started, <i>animate()</i> and <i>draw()</i> ";
	  text += "are called in an infinite loop, at a frequency that can be fixed.<br><br>";
	  text += "Press <b>Return</b> to start/stop the animation.";
	  return text;
	}

	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
		q = msg->orientation;
		
		angle = 2.0*acos(q.w);
		double s = sqrt(1.0-q.w*q.w);
		if(s < 0.001){
			x = 1.0;
			y = 0.0;
			z = 0.0;
		}
		else {
			x = q.x/s;
			y = q.y/s;
			z = q.z/s;
		}
	}
	
	ros::Subscriber sub;
	geometry_msgs::Quaternion q;
	GLfloat  xAngle, yAngle, zAngle;
	double angle, x,y,z;

};

////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  QApplication application(argc,argv);
  
    ros::init(argc, argv, "ahrs_view");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
	
	Viewer *viewer = new Viewer(n);
	viewer->setFPSIsDisplayed(true);
	viewer->setWindowTitle("AHRS Viewer");
  
  	viewer->show();
  	
  	ROS_INFO("+---| AHRS Viewer |---------+");
  	ROS_INFO("| ESC: quit program         |");
  	ROS_INFO("| Mouse: rotate/zoom        |");
  	ROS_INFO("+---------------------------+");

  	return application.exec();
}
