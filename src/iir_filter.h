/*********************************\
 * Note that this doesn't work very well right now ... it is a work in 
 * progresss. :)
 * Also, the IMU has hardware filters onboard that probably do just
 * as good of a job.
 *********************************/

#ifndef __IIR_FILTER_H__
#define __IIR_FILTER_H__

#include <ros/ros.h>
#include <soccer/IMU.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include <math.h>
#include <iostream>

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
	 * Fill IIR coefficients
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
	 * Uses 2 circular buffers, on each for x and y.
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
 * The digital filters can be removed if desired, just leaving the bias corrections.
 */
class TriAxisFilter {
public:
	TriAxisFilter() : x(6), y(6), z(6) {
        double b[] = {0.52629, 2.56668, 5.07017, 5.07017, 2.56668, 0.52629};
        double a[] = {1.00000, 3.62703, 5.47120, 4.25403, 1.69702, 0.27699};
        
        x.init(b,a);
        y.init(b,a);
        z.init(b,a);
        
        useFilter = true;
	}
	
	void setBias(double xx, double yy, double zz){
		x_bias = xx;
		y_bias = yy;
		z_bias = zz;
	}
	
	void filter(double& xx, double& yy, double& zz){
		if(useFilter){
			xx = x.filter(xx - x_bias);
			yy = y.filter(yy - y_bias);
			zz = z.filter(zz - z_bias);
		}
		else {
			xx = (xx - x_bias);
			yy = (yy - y_bias);
			zz = (zz - z_bias);
		}
	}
	
	void setUseFilter(bool b){
		useFilter = b;
	}
	
	inline bool getUseFilter(void) const {
		return useFilter;
	}
	
//protected:
	double x_bias, y_bias, z_bias;
	DigitalFilter x, y, z;
	bool useFilter;
};



#endif