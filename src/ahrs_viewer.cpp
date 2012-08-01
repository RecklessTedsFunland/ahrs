
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

//------------ PCL Tools ---------
#include <pcl_tools/cloud_maker.hpp>
#include <pcl_tools/viewer_3d.h>


////////////////////////////////////////////////////////////////
#include <qapplication.h>
#include <QGLViewer/qglviewer.h>

/**
 * Displays a simple 3D cube to represent the pose of the sensor.
 */
class Viewer : public QGLViewer {
public:

	Viewer(): cloud(new PointCloud) { cloud_maker = NULL;}
	//virtual ~Viewer();
	
protected :

  virtual void draw(){
  	;
  }
  
  virtual void init(){
		restoreStateFromFile();
		glDisable(GL_LIGHTING);
		//glPointSize(3.0);
		setGridIsDrawn();
		//help(); 
		setAnimationPeriod(0); // fast as possible
		startAnimation();
		showEntireScene();
		//setSceneRadius(5.0);
		//setFPSIsDisplayed(true);
		
		// Make world axis visible
		setAxisIsDrawn(false);
		
		// Move camera according to viewer type (on X, Y or Z axis)
		camera()->setPosition(qglviewer::Vec(0.0,-1.0,-1.0));
		camera()->lookAt(sceneCenter());
		camera()->setUpVector(qglviewer::Vec(0.0,-1.0,0.0));
		
		//camera()->setType(Camera::ORTHOGRAPHIC);
		//camera()->showEntireScene();
		
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

};

////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  QApplication application(argc,argv);
  
    ros::init(argc, argv, "pcl_view");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ROS_INFO("Start");
	
	Viewer *viewer = new Viewer();
	viewer->setFPSIsDisplayed(true);
	viewer->setFilter(filter);
	viewer->setWindowTitle("animation");
	

    image_transport::ImageTransport transport(n);
    image_transport::CameraSubscriber depth_sub = transport.subscribeCamera("/camera/depth/image_raw", 1, 
    												&Filter::depthCb, filter);
  
  	viewer->show();

  	return application.exec();
}
