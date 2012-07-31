#include <ros/ros.h>
#include <soccer/IMU.h>

#include <math.h>

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
class AHRS {
public:
	AHRS(){
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
		beta = 0.1;
	}
	
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	
		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
			ROS_INFO("Warning: MadgwickAHRSupdate() has bad magnetometer values");
			update(gx, gy, gz, ax, ay, az, dt);
			return;
		}
	
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	
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
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
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
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;
	
			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
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
	
	//---------------------------------------------------------------------------------------------------
	// IMU algorithm update	
	void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   
	
			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_4q0 = 4.0f * q0;
			_4q1 = 4.0f * q1;
			_4q2 = 4.0f * q2;
			_8q1 = 8.0f * q1;
			_8q2 = 8.0f * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;
	
			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
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
	
	void print(){
		ROS_INFO_THROTTLE(0.1,"RPY: %2.2f %2.2f %2.2f",roll(),pitch(),yaw());
	}
	
	inline float normQ(){ return invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); }

	inline float roll(){ return 180.0/M_PI*atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); }
	inline float pitch(){ return -180.0/M_PI*asin(2.0*q1*q3+2.0*q0*q2); }
	inline float yaw(){ return 180.0/M_PI*atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0); }


protected:
	//---------------------------------------------------------------------------------------------------
	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
	
	float invSqrt(float x) {
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}
	
	float q0,q1,q2,q3;
	float beta;
};



//////////////////////////////////////////////////////////
#include <boost/circular_buffer.hpp>

/**
 * Implements a digital filter with B (feedforward coeff) and 
 * A (feedback coeff) produced from Matlab or Octave.
 * y[n] = Sum(bi*x[n-i]) - Sum(aj*y[n-j])
 *
 * Octave:
 * Fs = 20;              % sampling freq
 * Wp = .8; Ws = .9;     % normalized pass/stop bands [0,1]
 * Rp = 1; Rs = 40;      % ripple in pass/stop bands
 * [n,Ws] = cheb2ord(Wp,Ws,Rp,Rs);
 * [b,a] = cheby2(n,Rs,Ws);
 */
class DigitalFilter {
public:
	DigitalFilter(int s) : size(s), x_cb(s), y_cb(s){
		a = new double[size];
		b = new double[size];
	}
	
	~DigitalFilter(){
		delete[] a;
		delete[] b;
	}
	
	/**
	 * Fill coefficients
	 */
	void init(double *bb, double *aa){
		
		for(int i=0;i<size;++i){
			b[i] = bb[i];
			a[i] = aa[i];
			x_cb.push_front(0.0);
			y_cb.push_front(0.0);
		}
	}
	
	/**
	 * Difference filter
	 */
	double filter(double in){
		x_cb.push_front(in);
		
		// difference equation
		// note: subtracting -1 from y because i can't increment
		//       buffer until after i know what "out" is
		double out = b[0]*in;
		for(int i=1;i<size;++i) out +=b[i]*x_cb[i]-a[i]*y_cb[i-1];
		
		y_cb.push_front(out);
		
		return out;
	}
	
protected:
	double *a,*b;   // coeff arrays
	const int size; // size of filter = order + 1 (5th order; size = 6)
	boost::circular_buffer<double> x_cb, y_cb; // x = in; y = out
};

/**
 * Collection of 3 DigitalFilters to correct for noise and bias of a triple axis
 * sensor.
 */
class TriAxisFilter {
public:
	TriAxisFilter() : x(6), y(6), z(6) {
        double b[] = {0.52629, 2.56668, 5.07017, 5.07017, 2.56668, 0.52629};
        double a[] = {1.00000, 3.62703, 5.47120, 4.25403, 1.69702, 0.27699};
        
        x.init(b,a);
        y.init(b,a);
        z.init(b,a);
	}
	
	void setBias(double xx, double yy, double zz){
		x_bias = xx;
		y_bias = yy;
		z_bias = zz;
	}
	
	void filter(double& xx, double& yy, double& zz){
#if 1
		xx = x.filter(xx - x_bias);
		yy = y.filter(yy - y_bias);
		zz = z.filter(zz - z_bias);
#else
		xx = (xx - x_bias);
		yy = (yy - y_bias);
		zz = (zz - z_bias);
		
#endif
	}
	
protected:
	double x_bias, y_bias, z_bias;
	DigitalFilter x, y, z;
};


//////////////////////////////////////////////////////////


class Filter2 {

public:

    Filter2(ros::NodeHandle &n) : accel(), gyro(), mag(){
        debug = true;
        imu_sub = n.subscribe("imu", 10, &Filter2::callback,this);
        timer_old = ros::Time::now();
        timer = timer_old;
        
        accel.setBias(-0.021525, -0.004467, 0.031750);
        gyro.setBias(-0.026398, 0.002946, 0.021828);
        mag.setBias(-0.021525, -0.004467, 1.031750);
    }

    ~Filter2()
    {

    }
    
    void callback(const soccer::Imu::ConstPtr& msg) {
    	timer_old = timer;
        timer = ros::Time::now(); // = msg->header.stamp;
        double dt = (timer-timer_old).toSec();
#if 0        
        ahrs.update(msg->gyros.x,msg->gyros.y,msg->gyros.z,
        			msg->accels.x,msg->accels.y,msg->accels.z,
        			msg->mags.x,msg->mags.y,msg->mags.z,dt);
#else
		double ax = msg->accels.x;
		double ay = msg->accels.y;
		double az = msg->accels.z;
		accel.filter(ax,ay,az);
		
		double gx = msg->gyros.x;
		double gy = msg->gyros.y;
		double gz = msg->gyros.z;
		gyro.filter(gx,gy,gz);
		
		double mx = msg->mags.x;
		double my = msg->mags.y;
		double mz = msg->mags.z;
		mag.filter(mx,my,mz);
		
		ahrs.update(gx,gy,gz, ax,ay,az, mx,my,mz, dt);
#endif        					
        ahrs.print();
        
    }
    
    ros::NodeHandle node;
    ros::Subscriber imu_sub;
    ros::Time timer;   //general purpose timer
    ros::Time timer_old;
    bool debug;
    TriAxisFilter accel, gyro, mag;
    AHRS ahrs;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    Filter2 filter(n);
    ros::spin();
    return 0;
}
