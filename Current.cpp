/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//Current.cpp

#include "Current.h"
#include "Tool.h"
#include <math.h>

Current::Current(void)
{
	init();
}


Current::~Current(void)
{
}

//初始化
void Current::init()
{
	rho_w = 1025;

	Lpp = 162.0;
	Tm = 7.2;

	//流力系数，h/d>4.4
	double xValue[] = {-0.034, -0.032, -0.031, -0.031, -0.030, -0.028, -0.022, -0.016, -0.007, 0.003,  0.016,
		0.029, 0.039, 0.043, 0.042, 0.040, 0.035, 0.027, 0.019, 0.010, 0.004, -0.001, -0.004, -0.006,
		-0.006, -0.004, 0.000, 0.006, 0.015, 0.023, 0.031, 0.036, 0.041, 0.043, 0.045, 0.047, 0.048};

	double yValue[] = {0.00, 0.04, 0.07, 0.11, 0.18, 0.21, 0.24, 0.29, 0.33, 0.37, 0.41, 
		0.45, 0.47, 0.51, 0.54,	0.56, 0.58, 0.59, 0.60, 0.59, 0.58, 0.56, 0.55, 0.52, 
		0.48, 0.45, 0.41, 0.38, 0.35, 0.28,	0.24, 0.20, 0.16, 0.12, 0.08, 0.04, 0.00};

	double nValue[] = {0, 0.0075, 0.0175, 0.0275, 0.0350, 0.0400, 0.0450,	0.0475, 0.0525, 0.0525, 0.0500,	0.0475,
		0.0425, 0.0350,	0.0275, 0.0175, 0.0090, -0.0050, -0.0150, -0.0250, -0.0350,	-0.0475, -0.0575, -0.0650, -0.0725,
		-0.0775, -0.0800, -0.0825, -0.0800, -0.0775, -0.0725, -0.0650, -0.0600, -0.0475, -0.0350, -0.0200, -0.0100};
	for (int i = 0; i <  NUM; i ++)
	{
		cxc[i] = xValue[i];
		cyc[i] = yValue[i];
		cnc[i] = nValue[i];
	}

	Vc = 0.0;
	dirC = 0.0;
	beta = 0.0;
	psi = 0.0;

	cxcValue = 0.0;
	cycValue = 0.0;
	cncValue = 0.0;
}

//输入流速
void Current::setPara(const double speed, const double dir)
{
	Vc = speed;
	dirC = Tool::infToPi(dir * angToRad);
}

//输入船舶艏向(弧度)
void Current::setPsi(const double psiIn)
{
	psi = Tool::infToPi(psiIn);
}

//计算流力
void Current::cal()
{
	//if ((dirC-psi) < -PI)
	//{
	//	beta = PI * 2.0 - (dirC-psi);
	//} else if ((dirC-psi) > PI)
	//{
	//	beta = (dirC-psi) - PI * 2.0;
	//} else {
	//	beta = (dirC-psi);
	//}

	beta = Tool::infToPi(psi - dirC - PI);

	cxcValue = interp(cxc, NUM, fabs(beta));
	cycValue = interp(cyc, NUM, fabs(beta));
	cncValue = interp(cnc, NUM, fabs(beta));

	forceCur.xForce = 0.5*cxcValue*rho_w*pow(Vc,2.0)*Lpp*Tm;
	if (beta < -0.000001 || beta > 0.000001)
	{
		forceCur.yForce = 0.5*cycValue*rho_w*pow(Vc,2.0)*Lpp*Tm*beta/fabs(beta);
		forceCur.nMoment = 0.5*cncValue*rho_w*pow(Vc,2.0)*pow(Lpp,2.0)*Tm*beta/fabs(beta);
	} else {
		forceCur.yForce = 0.0;
		forceCur.nMoment = 0.0;
	}
	forceCur.zForce = 0.0;
	forceCur.kMoment = 0.0;
	forceCur.mMoment = 0.0;
}

//输出流力
Force6 Current::force()
{
	return forceCur;
}

//插值，计算流力系数
//输入：流力系数向量，总数，入射角
double Current::interp(const double *cc, const double num, const double bt)
{
	double cValue = 0.0;
	int index = 0;
	index = static_cast<int>(bt/(PI/(NUM-1)));
	cValue = (cc[index+1]-cc[index])*(bt/(PI/(NUM-1))-index) + cc[index];
	return cValue;
}