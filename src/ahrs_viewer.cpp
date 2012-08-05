
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
	
	void Draw_Box (void)
	{
		glBegin (GL_QUADS);
	
		  glColor3f  ( 0.0,  0.7, 0.1);     // Front - green
		  glVertex3f (-1.0,  1.0, 1.0);
		  glVertex3f ( 1.0,  1.0, 1.0);
		  glVertex3f ( 1.0, -1.0, 1.0);
		  glVertex3f (-1.0, -1.0, 1.0);
	
		  glColor3f  ( 0.9,  1.0,  0.0);    // Back  - yellow
		  glVertex3f (-1.0,  1.0, -1.0);
		  glVertex3f ( 1.0,  1.0, -1.0);
		  glVertex3f ( 1.0, -1.0, -1.0);
		  glVertex3f (-1.0, -1.0, -1.0);
	
		  glColor3f  ( 0.2, 0.2,  1.0);     // Top - blue 
		  glVertex3f (-1.0, 1.0,  1.0);
		  glVertex3f ( 1.0, 1.0,  1.0);
		  glVertex3f ( 1.0, 1.0, -1.0);
		  glVertex3f (-1.0, 1.0, -1.0);
	
		  glColor3f  ( 0.7,  0.0,  0.1);    // Bottom - red
		  glVertex3f (-1.0, -1.0,  1.0);
		  glVertex3f ( 1.0, -1.0,  1.0);
		  glVertex3f ( 1.0, -1.0, -1.0);
		  glVertex3f (-1.0, -1.0, -1.0);
	
		glEnd();
	}


  virtual void draw(){

    glPushMatrix ();
       glScalef (0.2, 0.2, 0.2);
       glRotatef(angle*180.0/M_PI,x,y,z);
       Draw_Box ();
    glPopMatrix ();

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
