#include "plane.h"
#include <cmath>
#include <iostream>
using namespace std;


cPlaneEKF::cPlaneEKF() 
{  	 

    setDim(3, 1, 3, 3, 3);  //�]�wmatrix dimention  n state = 4 (X = A X   X vector ���j�p)   m (measurement vector  ���j�p)
	/*�]�wmatrix dimention
		setDim setDim  (  
		K_UINT_32  n_,    # of state      
		K_UINT_32  nu_,   # of input 
		K_UINT_32  nw_,   # of standrd process 
		K_UINT_32  m_,    # of measure  
		K_UINT_32  nv_    # of measurement   noise 

    K_UINT_32 n;        //!< Size of the state vector.
    K_UINT_32 nu;       //!< Size of the input vector.
    K_UINT_32 nw;       //!< Size of the process noise vector.
    K_UINT_32  m;       //!< Size of the measurement vector.
    K_UINT_32 nv;       //!< Size of the measurement noise vector.


*/
	   COG_height = 0.666 ; // COG ���a���� (���mm)
	   samplingtime = 0.005;    //step time 5ms 
	   ww = sqrt( 9.8/ COG_height) ;
	   mass = 730; //�����H��q ��� kg  from adams z-force
	  

}



void cPlaneEKF::makeBaseA()
{
    A(1,1) = 1.0;
	A(1,2) = samplingtime;
	A(1,3) = 0.0;

	A(2,1) = ww*ww*samplingtime;
	A(2,2) = 1;
	A(2,3) = -ww*ww*samplingtime;
	
	A(3,1) = 0.0;
	A(3,2) = 0.0;
	A(3,3) = 1.0;


	}


void cPlaneEKF::makeA()
{
	// A(1,1) = 1.0;
	//A(1,2) = samplingtime;
	//// A(1,3) = 0.0;
	//// A(1,4) = 0.0;

	//// A(2,1) = 0.0;
	//A(2,2) = 1;
	//// A(2,3) = 0.0;
	//// A(2,4) = 0.0;

	//// A(3,1) = 0.0;
	//A(3,2) = 0;
	// A(3,3) = 1.0;
	// A(3,4) = Period;
	

}



void cPlaneEKF::makeBaseW()   //�ھ�f ��partial w ��process noise �C��state ����process noise 
{
	W(1,1) = 1.0; 
	W(1,2) = 0.0;
	W(1,3) = 0.0;
	
	W(2,1) = 0.0;
	W(2,2) = 1.0;
	W(2,3) = 0.0;
		
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(3,3) = 1.0;





}



void cPlaneEKF::makeBaseQ()  //standard  process covariance matrix �� 4X4 �y�z�U��variable����covariance 
{     
	//�ۤv��ۤv 0.001 
	Q(1,1) = 0.001;          //�Nx(4) external force �դj  ����۫Hmeasurement
	Q(1,2) = 0.001/50;
	Q(1,3) = 0.001/50;
	
	Q(2,1) = 0.001/50;
	Q(2,2) = 0.001;
	Q(2,3) = 0.001/50;
	
	Q(3,1) = 0.001/50;
	Q(3,2) = 0.001/50;
	Q(3,3) = 0.001;



}





void cPlaneEKF::makeBaseH()  ///matrix C 
{
	H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
	
	H(2,1) = 0.0;
	H(2,2) = 0.0;
	H(2,3) = 1.0;
	
	H(3,1) = ww*ww;
	H(3,2) = 0.0;
	H(3,3) = -ww*ww;
	


}







void cPlaneEKF::makeH()
{
	/*H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
	
	H(2,1) = 0.0;
	H(2,2) = 0.0;
	H(2,3) = 1.0;
	
	H(3,1) = ww*ww;
	H(3,2) = 0.0;
	H(3,3) = -ww*ww;*/
}



void cPlaneEKF::makeBaseV()
{
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(1,3) = 0.0;
	
	
	V(2,1) = 0.0;
	V(2,2) = 1.0;
    V(2,3) = 0.0;
	

	V(3,1) = 0.0;
	V(3,2) = 0.0;
    V(3,3) = 1.0;
	
}

void cPlaneEKF::makeBaseR()  //measurement noise  (�̾ڨC��sensor�|�����P) ���q�覡:�Nsensor����A��ocovariance
{

	////EXP MODE �Ѽ�
	R(1,1) = 0.0001;  // ��joint encoder �^�⪺COG ��m
	R(2,2) = 0.001223;  // �ѤO�W��X�� ZMP
	//R(3,3) = 0.00022;  //IMU �q���� COG �[�t�� 
	R(3,3) = 0.0000002;  //IMU �q���� COG �[�t��

	//process noise = 0.001
	//ADAMS MODE �Ѽ�
	//R(1,1) = 0.01;  // ��joint encoder �^�⪺COG ��m
	//R(2,2) = 0.0001;  // �ѤO�W��X�� ZMP
	//R(3,3) = 0.0001;  //IMU �q���� COG �[�t�� 
	


}


void cPlaneEKF::makeProcess() //  doing model  �y�zstate�������Y �̾�matrix A   
{
	Vector x_(x.size());
	
	x_(1) = x(1) +  samplingtime*x(2);
	x_(2) = ww*ww*samplingtime*x(1)+x(2)-ww*ww*samplingtime*x(3)+(samplingtime/mass)* x(4);
	x_(3) = x(3); 
	x.swap(x_);
}


void cPlaneEKF::makeMeasure()    //�y�zmeasurement�Pstate�������Y  �̾� matrix C  x(4) �� (1/kg)*N   1/mass*x(4)*1000 �n��1000 
{
	z(1)=x(1);
	z(2)=x(3);
	z(3)= ww*ww*x(1) - ww*ww*x(3) ;
	
}



