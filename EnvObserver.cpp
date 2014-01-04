/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-25							**
******************************************************************/

//EnvObserver.cpp

#include "EnvObserver.h"

EnvObserver::EnvObserver(void)
{
	//初始化
	init();
}

EnvObserver::~EnvObserver(void)
{
}

//初始化
void EnvObserver::init()
{
	tStep = 0.0;
	for (int i = 0; i < DOF3; i ++)
	{
		beta[i] = 0.0;
		tao[i] = 0.0;
		envTao[i] = 0.0;
		nu[i] = 0.0;
		D_nu[i] = 0.0;
		M_nu[i] = 0.0;
		K_M_nu[i] = 0.0;

		for (int j = 0; j < DOF3; j ++)
		{
			K0[i][j] = 0.0;
			M[i][j] = 0.0;
			D[i][j] = 0.0;
		}
	}
	//系统惯性矩阵(原6自由度系统惯性矩阵简化而来)
	M[0][0] = 0.0026e+10;
	M[1][1] = 0.0033e+10;
	M[2][2] = 6.5209e+10;

	//阻尼矩阵(原6自由度阻尼矩阵简化而来)
	D[0][0] = 0.0002e+8;
	D[1][1] = 0.0022e+8;
	D[2][2] = 7.1506e+8;
	
}


//设置时间间隔
void EnvObserver::setStep(const double deltaT)
{
	tStep = deltaT;
}

//设置参数
void EnvObserver::setK(const double k1, const double k2, const double k3)
{
	K0[0][0] = k1;
	K0[1][1] = k2;
	K0[2][2] = k3;
}

//输入M矩阵(3维)
void EnvObserver::setM(const double (*Mmat)[DOF3])
{
	for (int i = 0; i < DOF3; i ++)
	{
		for ( int j = 0; j < DOF3; j ++)
		{
			M[i][j] = Mmat[i][j];
		}
	}
}

//输入D矩阵(3维)
void EnvObserver::setD(const double (*Dmat)[DOF3])
{
	for (int i = 0; i < DOF3; i ++)
	{
		for ( int j = 0; j < DOF3; j ++)
		{
			D[i][j] = Dmat[i][j];
		}
	}
}

//输入推进器推力与力矩
void EnvObserver::setTao(const Force6 thrust)
{
	tao[0] = thrust.xForce;
	tao[1] = thrust.yForce;
	tao[2] = thrust.nMoment;
}

//输入船舶运动速度
void EnvObserver::setNu(const Nu nuIn)
{
	nu[0] = nuIn.u;
	nu[1] = nuIn.v;
	nu[2] = nuIn.r;
}

//输出环境力(3自由度)
Force3 EnvObserver::force()
{
	env.xForce = envTao[0];
	env.yForce = envTao[1];
	env.nMoment = envTao[2];
	return env;
}

//矩阵乘法，6x6矩阵与6x1矩阵相乘
void EnvObserver::multiMx(const double (*dataMx1)[DOF3], const double dataMx2[], double resultMx[])
{
	for (int i = 0; i < DOF3; ++ i)
	{
		resultMx[i] = 0;
		for (int j = 0; j < DOF3; ++ j)
		{
			resultMx[i] += dataMx1[i][j]*dataMx2[j];
		}
	}
}

//环境估计
void EnvObserver::cal()
{
	//计算中间变量，三维向量
	multiMx(D, nu, D_nu);
	multiMx(M, nu, M_nu);
	multiMx(K0, M_nu, K_M_nu);

	//计算中间变量
	for (int i = 0; i < DOF3; i ++)
	{
		betaTmp[i] = -(beta[i] - D_nu[i] + tao[i] + K_M_nu[i]);
	}

	//计算β的导数
	multiMx(K0, betaTmp, dBeta);

	//计算β与环境估计力
	for (int i = 0; i < DOF3; i ++)
	{
		beta[i] += dBeta[i]*tStep;
		envTao[i] = beta[i] + K_M_nu[i];
	}
}
