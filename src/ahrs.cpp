// see https://github.com/walchko/ahrs for more info.


// ROS ---------------------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

// AHRS ------------------
#include <ahrs/IMU.h>

// C++ ---------------------
#include <math.h>
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

// Boost -------------------
#include <boost/program_options.hpp>     // command line options
namespace po = boost::program_options;

// is there something else I can use instead?
class Vec3 {
public:
   double x, y, z;
};

class Sensor {
public:
	std::string name;
	double range;
	Vec3 bias;
	Vec3 cov;
};


//-------------------------------------------------------------------------------------
// AHRS algorithm update
// Note: [q0 q1 q2 q3] = [w x y z]
// beta = sqrt(3/4)*wb
// wb: represents the estimated mean zero gyroscope measurement error of each axis 
// gamma = sqrt(3/4)*wc
// wc: represents the estimated rate of gyroscope bias drift in each axis
//-------------------------------------------------------------------------------------
class AHRS {
public:
	AHRS(){
		reset();
		//beta = 0.1;
		beta = sqrt(3.0/4.0)*M_PI/180.0*5.0; // 5 degrees of measurement error
		useMags = true;
	}
	
	void setUseMags(bool b){
		useMags = b;
	}
	
	inline bool getUseMags(void) const {
		return useMags;
	}
	
	void setBeta(double b){
		beta = b;
	}
	
	inline double getBeta(void){
		return beta;
	}
	
	void reset(void){
		q0 = 1.0;           // w
		q1 = q2 = q3 = 0.0; // x y z
	}
	
	void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double dt) {
		double recipNorm;
		double s0, s1, s2, s3;
		double qDot1, qDot2, qDot3, qDot4;
		double hx, hy;
		double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	
		if(!useMags){
			update(gx, gy, gz, ax, ay, az, dt);
			return;
		}
		
		
		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
			ROS_ERROR("Warning: MadgwickAHRSupdate() has bad magnetometer values");
			update(gx, gy, gz, ax, ay, az, dt);
			return;
		}
	
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
	
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
	
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   
	
			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
	
			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * q0 * mx;
			_2q0my = 2.0 * q0 * my;
			_2q0mz = 2.0 * q0 * mz;
			_2q1mx = 2.0 * q1 * mx;
			_2q0 = 2.0 * q0;
			_2q1 = 2.0 * q1;
			_2q2 = 2.0 * q2;
			_2q3 = 2.0 * q3;
			_2q0q2 = 2.0 * q0 * q2;
			_2q2q3 = 2.0 * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;
	
			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0 * _2bx;
			_4bz = 2.0 * _2bz;
	
			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;
	
			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}
		else ROS_ERROR("Accels are bad");
	
		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * dt;
		q1 += qDot2 * dt;
		q2 += qDot3 * dt;
		q3 += qDot4 * dt;
	
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}
	
	//----------------------------------------------------------------------------------
	// IMU algorithm update	
	void update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
		double recipNorm;
		double s0, s1, s2, s3;
		double qDot1, qDot2, qDot3, qDot4;
		double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
	
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
	
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   
	
			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * q0;
			_2q1 = 2.0 * q1;
			_2q2 = 2.0 * q2;
			_2q3 = 2.0 * q3;
			_4q0 = 4.0 * q0;
			_4q1 = 4.0 * q1;
			_4q2 = 4.0 * q2;
			_8q1 = 8.0 * q1;
			_8q2 = 8.0 * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;
	
			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;
	
			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}
	
		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * dt;
		q1 += qDot2 * dt;
		q2 += qDot3 * dt;
		q3 += qDot4 * dt;
	
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}
	
	//inline double normQ(){ return sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3); }

	
	// calc Euler, returns rads
	inline double roll(){ return atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); }
	inline double pitch(){ return -asin(2.0*q1*q3+2.0*q0*q2); }
	inline double yaw(){ return atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0); }
	
	
	geometry_msgs::Quaternion quaterion(){
		geometry_msgs::Quaternion q;
		q.x = q1;
		q.y = q2;
		q.z = q3;
		q.w = q0;
		
		return q;
	}

protected:
	//-----------------------------------------------------------------------------
	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
	/*
	float invSqrt(float x) {
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}
	*/
	inline double invSqrt(const double x){
		return 1.0/sqrt(x);
	}
	
	double q0,q1,q2,q3;
	double beta; 
	bool useMags;
};

/**
 * ROS wrapper for the above AHRS class. It handles subscriptions and publication of 
 * of ROS messages. 
 */
class AHRS_Filter : public AHRS {

public:
	enum sensor_names {IMU_ACCELS, IMU_GYROS, IMU_MAGS};

    AHRS_Filter(ros::NodeHandle &n) {
        debug = false;
        digitalCompass = true;
        
        imu_sub = n.subscribe("imu", 100, &AHRS_Filter::callback,this);
        imu_pub = n.advertise<sensor_msgs::Imu>("imu_out", 100);
        timer_old = ros::Time::now();
        timer = timer_old;
        
        imu_msg.angular_velocity_covariance[0] = 1.0;
        imu_msg.angular_velocity_covariance[4] = 1.0;
        imu_msg.angular_velocity_covariance[8] = 1.0;
        
        imu_msg.linear_acceleration_covariance[0] = 1.0;
        imu_msg.linear_acceleration_covariance[4] = 1.0;
        imu_msg.linear_acceleration_covariance[8] = 1.0;
        
    }

    ~AHRS_Filter()
    {

    }
    
    void setDebug(bool b){
    	debug = b;
    }
    
    void setSensor(int which, Sensor& s){
    	switch(which){
    		case IMU_ACCELS:
    			accel_bias[0] = s.bias.x;
    			accel_bias[1] = s.bias.y;
    			accel_bias[2] = s.bias.z;
    			
				imu_msg.linear_acceleration_covariance[0] = s.cov.x;
				imu_msg.linear_acceleration_covariance[4] = s.cov.y;
				imu_msg.linear_acceleration_covariance[8] = s.cov.z;
    			break;
    		case IMU_GYROS:
    			gyro_bias[0] = s.bias.x;
    			gyro_bias[1] = s.bias.y;
    			gyro_bias[2] = s.bias.z;
    			
				imu_msg.angular_velocity_covariance[0] = s.cov.x;
				imu_msg.angular_velocity_covariance[4] = s.cov.y;
				imu_msg.angular_velocity_covariance[8] = s.cov.z;
    			break;
    		case IMU_MAGS:
    			mag_bias[0] = s.bias.x;
    			mag_bias[1] = s.bias.y;
    			mag_bias[2] = s.bias.z;
    			break;
    	}
    }
    
    // from "tilt compensated compass.pdf" (LSM303DLH tech note)
    // calculate heading from magnetometers and return heading in degrees.
    double heading(void){
    	double heading = 0.0;
    	
    	double norm = sqrt(mx*mx+my*my+mz*mz);
    	double mx1 = mx/norm;
    	double my1 = my/norm;
    	double mz1 = mz/norm;
    	
    	double r = roll(); // roll
    	double g = pitch(); // pitch
    	
    	double mx2 = mx1*cos(r)+mz1*sin(r);
    	double my2 = mx1*sin(g)*sin(r)+my1*cos(g)-mz1*sin(g)*sin(r);
    	double mz2 = -mx1*cos(g)*sin(r)+my1*sin(g)+mz1*cos(g)*cos(r);
    	
    	heading = atan2(my2,mx2); // dboule check this
    	
    	if(mx2 > 0.0 && my2 >= 0.0); // all good 
    	else if( mx2 < 0.0) heading = M_PI+heading;
    	else if(mx2>0.0 && my2 <=0.0) heading = 2.0*M_PI+heading;
    	else if(mx2 == 0.0 && my2 < 0.0) heading = M_PI/2.0; // 90 deg
    	else if(mx2 == 0.0 && my2 > 0.0) heading = 1.5*M_PI; // 270 deg
    	
    	//ROS_INF("heading: %f",heading);
    	
    	return 180.0/M_PI*heading;
    }
    
    void callback(const ahrs::Imu::ConstPtr& msg) {
    	ROS_INFO_ONCE("** AHRS: beta = %1.4f, Magnetometers[%s] **",
    		beta, 
    		(useMags ? "On" : "Off"));
    	
    	// why am i not using the time from the message header??
    	timer_old = timer;
        timer = ros::Time::now(); // = msg->header.stamp;
        double dt = (timer-timer_old).toSec();

		// need to correct for biases
		// somehow should handle a range change??? 
		// i.e., 250 dps -> 500 dps or g's -> m/sec^2
		double ax = -(msg->accels.x - accel_bias[0]);
		double ay = -(msg->accels.y - accel_bias[1]);
		double az = msg->accels.z - accel_bias[2];
		
		double gx = -(msg->gyros.x - gyro_bias[0]);
		double gy = -(msg->gyros.y - gyro_bias[1]);
		double gz = msg->gyros.z - gyro_bias[2];
		
		if(useMags){
			mx = -(msg->mags.x - mag_bias[0]);
			my = -(msg->mags.y - mag_bias[1]);
			mz = msg->mags.z - mag_bias[2];
			
			update(gx,gy,gz, ax,ay,az, mx,my,mz, dt);
			
			//if(digitalCompass) ROS_INFO("Heading: %3.1f deg",heading());
		}
		else {
			update(gx,gy,gz, ax,ay,az, dt);
		}
		
		// what about the message header time here too?
		imu_msg.header.stamp = ros::Time::now();
		
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        
        imu_msg.orientation = quaterion();
        imu_pub.publish(imu_msg);

		if(debug) std::cout<< imu_msg << std::endl;
    }
    
    ros::NodeHandle node;
    ros::Subscriber imu_sub;
    ros::Publisher imu_pub; //
    ros::Time timer;   // current time
    ros::Time timer_old; // last time step time
    sensor_msgs::Imu imu_msg; //
    
    double accel_bias[3];
    double gyro_bias[3];
    double mag_bias[3];
    
    double mx, my, mz;
    
    bool debug;
    bool digitalCompass;
};


////////////////// YAML File \\\\\\\\\\\\\\\\\\\\\\

void printSensor(const Sensor& sensor){
	std::cout << "----- " << sensor.name << "----------" << std::endl;
	std::cout << sensor.bias.x << " " << sensor.bias.y << " " << sensor.bias.z << std::endl;
	std::cout << sensor.cov.x << " " << sensor.cov.y << " " << sensor.cov.z << std::endl;
	std::cout << sensor.range << std::endl;
} 

void operator >> (const YAML::Node& node, Vec3& v){
	node[0] >> v.x;
	node[1] >> v.y;
	node[2] >> v.z;
}

void operator >> (const YAML::Node& node, Sensor& sensor){
	node["name"] >> sensor.name;
	node["bias"] >> sensor.bias;
	node["cov"] >> sensor.cov;
	node["range"] >> sensor.range;
}

bool loadConfigFile(AHRS_Filter& filter, std::string file){
	
    try {
		std::ifstream fin(file.c_str());
		YAML::Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		
		if(doc.size() == 0) {
			return false;
		}
		
		for(unsigned i=0;i<doc.size();++i){
			Sensor sensor;
			doc[i] >> sensor;
			
			if(!sensor.name.compare("accels")){
				filter.setSensor(AHRS_Filter::IMU_ACCELS,sensor);
				//printSensor(sensor);
			}
			if(!sensor.name.compare("gyros")){
				filter.setSensor(AHRS_Filter::IMU_GYROS,sensor);
			}
			if(!sensor.name.compare("mags")){
				filter.setSensor(AHRS_Filter::IMU_MAGS,sensor);
			}
			
		}
	} 
	catch(YAML::ParserException& e) {
		std::cout << e.what() << std::endl;
	}	
	
	return true;
}

///////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahrs");
    ros::NodeHandle n;
    
    AHRS_Filter filter(n);
    /*
    if(loadConfigFile(filter,"imu.yaml") == false){
		ROS_ERROR("Couldn't read file ... exiting"); 
		exit(0);
	}
    */
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("no-mags", "turns off use of magnetometers")
            ("debug","prints debug info to screen")
            ("beta", po::value<double>(), "sets Beta value for gradient decent")
            ("config", po::value<std::string>(), "sets configuration of IMU default sensor values")
            ;
        po::variables_map vm;       
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);   
        
        if (vm.count("help")) {
        	std::cout << "rosrun ahrs ahrs [option] \n";
        	std::cout << "    default topic in [imu] out [imu_out] \n";
            std::cout << desc << "\n";
            return 1;
        }
        
        if (vm.count("no-mags")){
        	filter.setUseMags(false);
        }
        
        if (vm.count("debug")){
        	filter.setDebug(true);
        }
        
        if (vm.count("beta")){
        	filter.setBeta( vm["beta"].as<double>() );
        }
        
        if (vm.count("config")){
        	std::string file = vm["config"].as<std::string>();
			if(loadConfigFile(filter, file) == false){
				ROS_ERROR("Couldn't read file"); 
				exit(0);
			}
        }
    }
    catch(boost::exception& e) {
        //std::cerr << "AHRS Error " /*<< e.what()*/ << "\n";
        ROS_ERROR("AHRS");
        return 1;
    }
    
    //exit(0);
    
    ros::spin();
    return 0;
}
