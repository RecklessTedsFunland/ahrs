// see https://github.com/walchko/ahrs for more info.


// ROS ---------------------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
// Soccer ------------------
#include <soccer/IMU.h>

// C++ ---------------------
#include <math.h>
#include <iostream>

// Boost -------------------
#include <boost/program_options.hpp>     // command line options
namespace po = boost::program_options;

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

	/*
	inline double roll(){ return 180.0/M_PI*atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); }
	inline double pitch(){ return -180.0/M_PI*asin(2.0*q1*q3+2.0*q0*q2); }
	inline double yaw(){ return 180.0/M_PI*atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0); }
	*/
	
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


class AHRS_Filter : public AHRS {

public:

    AHRS_Filter(ros::NodeHandle &n) {
        debug = false;
        imu_sub = n.subscribe("imu", 100, &AHRS_Filter::callback,this);
        imu_pub = n.advertise<sensor_msgs::Imu>("imu_out", 100);
        timer_old = ros::Time::now();
        timer = timer_old;
        
        //read yaml file with bias, errors, etc and fill these values below
        
        //accel.setBias(-0.021525, -0.004467, 0.031750);
        //gyro.setBias(-0.026398, 0.002946, 0.021828);
        //mag.setBias(-0.021525, -0.004467, 1.031750);
        
        //gyro.x_bias *= M_PI/180.0;
        //gyro.y_bias *= M_PI/180.0;
        //gyro.z_bias *= M_PI/180.0;
        
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
    
    void setUseFilter(bool b){
    	//accel.setUseFilter(b);
    	//gyro.setUseFilter(b);
    	//mag.setUseFilter(b);
    }
    
    void callback(const soccer::Imu::ConstPtr& msg) {
    	ROS_INFO_ONCE("** AHRS: beta = %1.4f, Magnetometers[%s] **",
    		beta, 
    		(useMags ? "On" : "Off"));
    	
    	// why am i not using the time from the message header??
    	timer_old = timer;
        timer = ros::Time::now(); // = msg->header.stamp;
        double dt = (timer-timer_old).toSec();

		double ax = msg->accels.x;
		double ay = msg->accels.y;
		double az = msg->accels.z;
		
		double gx = msg->gyros.x;
		double gy = msg->gyros.y;
		double gz = msg->gyros.z;
		
		if(useMags){
			double mx = msg->mags.x;
			double my = msg->mags.y;
			double mz = msg->mags.z;
			
			update(gx,gy,gz, ax,ay,az, mx,my,mz, dt);
		}
		else {
			update(gx,gy,gz, ax,ay,az, dt);
		}
		
		// what about the message header time here too?
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
    ros::Publisher imu_pub;
    ros::Time timer;   //general purpose timer
    ros::Time timer_old;
    sensor_msgs::Imu imu_msg;
    
    bool debug;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahrs");
    ros::NodeHandle n;
    
    AHRS_Filter filter(n);
    
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("no-mags", "turns off use of magnetometers")
            ("no-filter","turns off use of IIR data filtering")
            ("debug","prints debug info to screen")
            ("beta", po::value<double>(), "sets Beta value for gradient decent")
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
        
        if (vm.count("no-filter")){
        	filter.setUseFilter(false);
        }
        
        if (vm.count("debug")){
        	filter.setDebug(true);
        }
        
        if (vm.count("beta")){
        	filter.setBeta( vm["beta"].as<double>() );
        }
    }
    catch(boost::exception& e) {
        //std::cerr << "AHRS Error " /*<< e.what()*/ << "\n";
        ROS_ERROR("AHRS");
        return 1;
    }
    
    ros::spin();
    return 0;
}
