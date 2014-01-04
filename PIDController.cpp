/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//PIDController.h

#include "PIDController.h"
#include "Tool.h"
#include <iostream>
using namespace std;

#define Xmax 1600000
#define Ymax 1600000
#define Nmax 60000000

PIDController::PIDController(void)
	: tStep(0.05), Kp(0.001), Ki(0.0), Kd(0.0) //初值P:0.01, I:0.0003, D:0.2
{
	init();
}


PIDController::~PIDController(void)
{
	errFile.close();
	pidFile.close();
}

void PIDController::init()
{
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			M[i][j] = 0.0;
			Mp[i][j] = 0.0;
			Mi[i][j] = 0.0;
			Md[i][j] = 0.0;
		}
		etaTgt[i] = 0.0;
		etaPre[i] = 0.0;
		etaErrSum[i] = 0.0;
		etaErr[i] = 0.0;
		etaErrPre[i] = 0.0;
		etaErrErr[i] = 0.0;
	}
	M[0][0] = 5312200.0;
	M[1][1] = 8283100.0;
	M[2][2] = 3745400000.0;

	errFile.open("E:/projectProgram/data/err.txt");
	pidFile.open("E:/projectProgram/data/pid.txt");
}


void PIDController::initTStep(double val)
{
	tStep = val;
}

void PIDController::initPID(double KpVal, double KiVal, double KdVal)
{
	Kp = KpVal;
	Ki = KiVal;
	Kd = KdVal;

	for (int i = 0; i < DOF3; i ++)
	{
		Mp[i][i] = M[i][i] * Kp;
		Mi[i][i] = M[i][i] * Ki;
		Md[i][i] = M[i][i] * Kd;
	}
	pidFile << "P:\t" << Kp << endl;
	pidFile << "I:\t" << Ki << endl;
	pidFile << "D:\t" << Kd << endl;
}

void PIDController::setTarget(Eta eta)
{
	etaTarget = eta;
}

void PIDController::setEta( Eta eta )
{
	etaCurrent = eta;
	Tool::EtaToArr3(eta, etaArr, DOF3);
}

void PIDController::calculat()
{
	//eta转化为3维向量
	Tool::EtaToArr3(etaTarget, etaTgt, DOF3);

	//计算eta偏差
	for (int i = 0; i < DOF3; i ++)
	{
		etaErr[i] = etaTgt[i] - etaPre[i];
		if (DOF3-1 == i)
		{
			etaErr[i] = Tool::infToPi(etaErr[i]);
		}
		etaErrErr[i] = etaErr[i] - etaErrPre[i];
		if (DOF3-1 == i)
		{
			etaErrErr[i] = Tool::infToPi(etaErrErr[i]);
		}

		errFile << etaErr[i] << "\t";
	}

	errFile << endl;

	//计算PID结果
	Tool::multiVector(Mp, etaErr, pRst, DOF3);

	Tool::multiVector(Mi, etaErrSum, iRst, DOF3);

	Tool::multiVector(Md, etaErrErr, dRst, DOF3);

	for (int i = 0; i < DOF3; i ++)
	{
		taoArr[i] = pRst[i] + iRst[i] + dRst[i];
	}
	taoArr[0] = (taoArr[0] > Xmax) ? Xmax : taoArr[0];
	taoArr[1] = (taoArr[1] > Ymax) ? Ymax : taoArr[1];
	taoArr[2] = (taoArr[2] > Nmax) ? Nmax : taoArr[2];

	taoArr[0] = (taoArr[0] < -Xmax) ? -Xmax : taoArr[0];
	taoArr[1] = (taoArr[1] < -Ymax) ? -Ymax : taoArr[1];
	taoArr[2] = (taoArr[2] < -Nmax) ? -Nmax : taoArr[2];

	//保存上一次的eta、etaErr
	for (int i = 0; i < DOF3; i ++)
	{
		etaPre[i] = etaArr[i];
		etaErrPre[i] = etaErr[i];
		etaErrSum[i] += etaErr[i];
	}
}

//获取
Force6 PIDController::getTao()
{
	outTao.xForce = taoArr[0];
	outTao.yForce = taoArr[1];
	outTao.zForce = 0.0;
	outTao.kMoment = 0.0;
	outTao.mMoment = 0.0;
	outTao.nMoment = taoArr[2];
	

	outTao = Tool::NedToboat(outTao, etaCurrent);

	return outTao;
}