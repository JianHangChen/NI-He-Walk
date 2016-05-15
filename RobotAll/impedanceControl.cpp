#include "stdafx.h"
#include <impedanceControl.h>

#include <iostream>
#include <math.h>

double sampling_time = 0.005;

ImpendanceControl::ImpendanceControl( double C, double K, double KP,double KI, double bound ):
	m(1),
	c(C),
	k(K),
	kp(KP),
	ki(KI),
	bound(bound),
	x(0),
	dx(0),
	ddx(0),
	deadzone(30),
	e_sum(0),
	x_data_index(0),
	control_index(0),
	control_switch(true)
{
	x_data[0] = x;
}
double ImpendanceControl::get_x_kadd1(double Td, double T)
{	
	double f_error = (Td - T);
			
	if( fabs(f_error) > deadzone && control_switch )
	{
		control_switch = false;
		control_index = 0;
		//std::cout<<f_error<<"\n";
		if (f_error>=0)
		{
			ddx = (f_error-deadzone)/m*kp;
		} 
		else
		{
			ddx = (f_error+deadzone)/m*kp;
		}
		
		std::cout<<ddx<<"\n";
		if (ddx>180)
		{
			ddx = 180;
		} 
		if (ddx<-180)
		{
			ddx = -180;
		}
		
	}
	
	dx = dx + ddx*sampling_time;
	x = x + dx*sampling_time+ddx*sampling_time*sampling_time*0.5;
	if (x>bound)
	{
		x = bound;
	} 
	if (x<-bound)
	{
		x = -bound;
	}
	save_x_data(x);
	ddx = (  - c*dx - k*x )/m;
	control_index++;

	if (control_index>1)
	{
		control_switch = true;
	}
	return x;
}
double ImpendanceControl::get_x_kadd1()
{
	dx = dx + ddx*sampling_time;
	x = x + dx*sampling_time+ddx*sampling_time*sampling_time*0.5;
	//std::cout<<m<<"\n";
	ddx = (  - c*dx - k*x )/m;
	
	return x;
}
void ImpendanceControl::save_x_data(double e)
{
	x_data_index++;	
	x_data[x_data_index] = e;

}

void ImpendanceControl::set_ddx(double a)
{
	ddx = a;
}