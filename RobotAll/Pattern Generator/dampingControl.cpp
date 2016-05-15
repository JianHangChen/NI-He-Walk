#include "stdafx.h"
#include "dampingControl.h"

#include <iostream>
#include <math.h>

#define exp  2.71828182846

DampingControl::DampingControl( double dt, double timeConst, double dampGain,double KI, double b):
	F(pow( exp, -dt/timeConst )),
	G((timeConst/dampGain) - (timeConst/dampGain)*pow( exp, -dt/timeConst )),
	x_k(0),
	ki(KI),
	e_sum(0),
	bound(b),
	data_index(0)
{
	x_data[0] = x_k;
}
void DampingControl::save_x_data(double e)
{
	data_index++;
	x_data[data_index] = e;
}
double DampingControl::get_x_kadd1(double Td, double T)
{	
	e_sum += T-Td;
	//std::cout<<"force error = "<<T-Td<<"\n";
	double result = F*x_k + G*(T-Td)+ki*e_sum;
	if (result>=bound)
	{
		result = bound;
	}
	if (result<=-bound)
	{
		result = -bound;
	}
	x_k = result;
	save_x_data(x_k);
	return result;
}
