/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: MatOptInterface.h

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/10

Functions:
     InvSqMat() AbsDetSqMat()  SubMat() SubMatIndex() MatScalarMul()
     MatMulAB() MatMulAtB() MatMulABt() MatMiuAB() MatAddAB()

Classes: None

Description:
     ���{���D�n�Φb�إߤ@�M�x�}�B��禡�w�B�w�q�F�x�}�B�⤧�禡�A
	 �]�t�ϯx�}�B��C���ȡB�x�}�[��k���B��A
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/
#include <iostream> 
#include "math.h"

#include "windows.h" // ���Finclude timer �i�� �n��b�o��A clapack.h���e tvmet ����

// ���ǭn��
#include <f2c.h>
#include "blaswrap.h"
extern "C"
{
#include <clapack.h> 
}
// ���ǭn��

using namespace std;

void InvSqMat(double * Data, int NCol); // ��J���x�}�� M.data(), �H�� M.Cols, �ģ��Ӧ^�ǯx�}�s�̭����������СA�ĤG�Ӧ^�ǯx�}������
double AbsDetSqMat(double * Data, int NCol);  // �Q��LU decomposition �s�򭼤W�﨤�u�A�o�� det�������
void SubMat(double *MMother, int ColMother, int RowMother, double *MSon, int *CutRegion); // ���X�l�x�}
void SubMatIndex(double *MMother, int ColMother, int RowMother, double *MSon, int *ColRegion, int *RowRegion); // ���X���w��C�Ƥ��l�x�}

// basic matrix computation
void MatScalarMul(double* A, int LenA, double* Scalar, double* result); // �x�}��jScalar��
void MatScalarMul(double* A, int LenA, double Scalar, double* result); // �x�}��jScalar��
void MatMulAB(double* A, int MA, int NA, double* B, int MB, int NB, double* result); // �x�}���k A*B
void MatMulAtB(double* A, int MA, int NA, double* B, int MB, int NB, double* result); // �x�}���k A'*B
void MatMulABt(double* A, int MA, int NA, double* B, int MB, int NB, double* result); // �x�}���k A*B'
void MatMiuAB(double* A, double* B, double* result, int Len);  // �x�}��k A-B
void MatAddAB(double* A, double* B, double* result, int Len);  // �x�}�[�k A+B
void showMat(double* A,int Nrow, int Ncol );
void showVar(double x);
void Mat_weighted_mean( double* A, int MA, int NA,double WA, double* B, int MB, int NB,double WB, double* result);
void Mat_I_nxn( double* result, int n );//return I nxn (���x�})
void Mat_O_mxn(double* result,int m, int n);
void Mat_y_Ax_b( double*A, int MA, int NA, double *x, int Mx ,int Nx, double *b, double *result );//y = Ax+b  ok
void Mat_A_addcol_B( double* A, int MA, int NA, double* B, int MB, int NB, double* result );
void Mat_A_addrow_B(double* A, int MA, int NA, double* B, int MB, int NB, double* result);
void Mat_A_B_At(double* A, int MA, int NA, double* B, int MB, int NB, double* result);
void Mat_a_x_b(double* v1, double* v2, double* v3);
void showMat(double* A,int Nrow, int Ncol, char * MatName );//ok
void showVar(double x,char * VarName);//ok

void R_rpy( double r,double p, double y ,double *result); //160504 r,p,y angle to Rotation Matrix