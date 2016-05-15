#ifndef IMPEDANCECONTROL_H
#define IMPEDANCECONTROL_H

class ImpendanceControl
{
private:
	double m;
	double c;
	double k;
	double x;
	double dx;
	double ddx;
	double kp;
	double ki;
	double e_sum;
	double bound;
	double deadzone;
	
	int control_index;
	bool control_switch;
public:	
	double x_data[50000];
	int x_data_index;
	ImpendanceControl( double C, double K, double KP,double KI, double bound );
	double get_x_kadd1(double Td, double T);
	double get_x_kadd1();
	void set_ddx(double a);
	void save_x_data(double e);
	double get_c();
	double get_k();
};


#endif
