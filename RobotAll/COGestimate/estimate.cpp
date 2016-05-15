#include "stdafx.h"
#include "estimate.h"
#include <vector>
#include <iomanip>//����cout ��X�p���I���

	vector <double>  Cogstatelateral (50000,0);
	vector <double>  Dcogstatelateral(50000,0);
	vector <double>  zmpstatelateral (50000,0);
	

	vector <double>  Cogstatesaggital (50000,0);
	vector <double>  Dcogstatesaggital(50000,0);
	vector <double>  zmpstatesaggital (50000,0);
		
	
	double COGsaggital;
	double DCOGsaggital;
	double ZMPsaggital;
	double COGsaggital_before;
	double DCOGsaggital_before;
	double ZMPsaggital_before;
	





	cPlaneEKF filter;

    estimate ::estimate(){
 
    } ;


int estimate::compute ( double cog ,double  *zmp ,double  cogaccel,int estimatecount, int direction  ) {
	
	//direction = 1 ,lateral  direction 
	//direction = 2 ,saggital direction  

	
	const unsigned NTRY = 1200*10; //data��
	
	
	const unsigned n = 3;	//nb states
	const unsigned m = 3;	//nb measures

	double  timestep  = 0.005 ; 
	
	double  Dzmp = 0   , Dzmpbefore = 0 ; //�e�Ӯɨ誺zmp �@���L���q 
	double  DDzmp = 0  , DDzmpbefore = 0 ; //�e�Ӯɨ誺zmp �G���L���q 


	double IMU_COG  = 666/732;



	//static const double _P0[] = { 0.1 ,0.0, 0.0, 0.0, 
	//							  0.0, 0.1 , 0.0 ,0.0, 
	//							  0.0,  0.0,  0.1 ,0.0,
	//							  0.0 , 0.0  ,0.0  ,0.1
	//
	//                                   };       //initial error 

	
	
	static const double _P0[] = { 0.1 ,0.0, 0.0,  
								  0.0, 0.1 , 0.0 , 
								  0.0,  0.0,  0.1 
	                                   };       //initial error 





		
	Vector x(n);   // �}state vector 
	
	Vector state(n) ;
	
	Vector z(m);
	
	
	Matrix P0(n, n, _P0); //�إ� initial error  matrix  ���i���z��20140120
	

	//P0(0,0) = 0 ;
	
	
	
	//zmp'' �ƭȷL���B�z
	
	

	if( estimatecount < 2) {
	 Dzmp = 0 ; 
	 Dzmpbefore  = 0 ; 
	 }
	
	
	else{
			
	Dzmp  =  ( *zmp - *(zmp-2) ) / timestep  ; 

	Dzmpbefore = ( *(zmp-2) - *(zmp-4)  ) / timestep ; 

		}

	DDzmp = ( Dzmp - Dzmpbefore )/ timestep ; 

		
	
	//P0(0,0) = 0 ;
	
	filter.init(x, P0);
	    
		z(1) = cog;  
		z(2) = *zmp;  
		z(3) = cogaccel* IMU_COG ;  //�Nmeasurement �g�J IMU�w�˦�m����Ҥ��t
	   
		
		// u  control input    zmp�G���L��
		
		Vector u(1);
		
		u(1) = DDzmp ;

		
		filter.step(u, z);

		state   = filter.getX()  ;

		
		
		if (direction == 1 ) {


		Cogstatelateral.push_back  (state (1))  ;  //�Nstate�s�J  cogstate matrix  
		Dcogstatelateral.push_back (state (2))  ;
		zmpstatelateral.push_back  (state (3))  ;
			
//cout << "xp(" << ":," << estimatecount<<") = " <<setprecision(3)<< Cogstatelateral[estimatecount]<<" "<<setprecision(3)<<Dcogstatelateral[estimatecount]<<" "<<setprecision(3)<< zmpstatelateral[estimatecount]<<" "<<  setprecision(3)<< externalforcelateral[estimatecount] <<endl;
//setprecision(3)  ����cout�p���I���
		}




		else if (direction ==2){
		
		
			Cogstatesaggital.push_back  (state (1))  ;  //�Nstate�s�J  cogstate matrix  
			Dcogstatesaggital.push_back (state (2))  ;
			zmpstatesaggital.push_back  (state (3))  ;
			//cout << "xp(" << ":," << estimatecount<<") = " <<setprecision(3)<< Cogstatesaggital[estimatecount]<<" "<<setprecision(3)<<Dcogstatesaggital[estimatecount]<<" "<<setprecision(3)<< zmpstatesaggital[estimatecount]<<" "<<  setprecision(3)<< externalforcesaggital[estimatecount] <<endl;

		
				
		
		
		}



return EXIT_SUCCESS;

}







estimate :: ~ estimate(){} ;





