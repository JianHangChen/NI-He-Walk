#ifndef DAMPINGCONTROL_H
#define DAMPINGCONTROL_H

#define  DAMP_CNTL_COMPLIANT 0
#define  DAMP_CNTL_NO_COMPLAINT 1

class DampingControl
{
private:
	double F;
	double G;
	double bound;
	double x_k;
	double ki;
	double e_sum;

	
	
public:	
	double x_data[50000];
	int data_index;
	DampingControl( double dt, double timeConst, double dampGain,double KI, double bound );
	double get_x_kadd1(double Td, double T);
	void save_x_data(double e);
};


#endif