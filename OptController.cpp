/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//OptController.cpp

#include "OptController.h"
#include "Tool.h"

OptController::OptController(void) :
P(0.02), I(0.0), D(0.0)
{
	init();
}


OptController::~OptController(void)
{
}

//初始化参数
void OptController::init()
{
	sum = 0.0;

	storOptPsi.clear();
	storXForce.clear();
	storYForce.clear();

	//初始化计算环境最优艏向时使用的力与转矩
	taoX = 0.0;
	taoY = 0.0;
	taoN = 0.0;
	tagTaoY = 0.0;
	//初始化原始推力与转矩
	xForce = 0.0;
	yForce = 0.0;
	nMoment = 0.0;

	//误差等的初始化
	errY = 0.0;
	intErrY = 0.0;
	diffErrY = 0.0;
	preErrY = 0.0;

	optPsi = 0.0;
	psi = 0.0;
	signY = 1.0;
}

//设置时间间隔
void OptController::setStep(const double Intval)
{
	step = Intval;
}

//设置当前环境最优艏向
void OptController::setPsi(const double psiValue)
{
	psi = psiValue;
}

//设置当前推进器推力与转矩
void OptController::setTao(const Force6 Thrust)
{
	xForce = Thrust.xForce;
	yForce = Thrust.yForce;
	nMoment = Thrust.nMoment;
}

//设置PID参数
void OptController::setPID(const double Pval, const double Ival, const double Dval)
{
	P = Pval;
	I = Ival;
	D = Dval;
}

//计算求均值
double OptController::meanData(list<double> &valLst, const double inVal)
{
	sum = 0.0;
	valLst.push_back(inVal);
	if (COUNT < valLst.size())
	{
		valLst.pop_front();
	}

	for (int i = 0; i < valLst.size(); i ++)
	{
		sum += valLst.front();
		valLst.push_back(valLst.front());
		valLst.pop_front();
	}
	return sum/static_cast<double>(valLst.size());	 
}

//PID计算
void OptController::cal()
{
	//分别计算在一定时间内纵向与横向的平均力
	taoX = meanData(storXForce, xForce);
	taoY = meanData(storYForce, yForce);

	if (fabs(taoY) > LMTY)
	{

		//if (LMTY < taoY)
		//{
		//	signY = -1.0;
		//} else if (-LMTY > taoY)
		//{
		//	signY = 1.0;
		//}
		errY = taoY - tagTaoY;
		intErrY += errY*step;
		diffErrY = (errY - preErrY)/step;
		preErrY = errY;

		optPsi = psi + signY*(P*errY + I*intErrY + D*diffErrY);
		optPsi = meanData(storOptPsi, optPsi);
		//optPsi = Tool::infToPi(optPsi);
	}
}

//获取环境最优艏向
double OptController::OptPsi()
{
	return optPsi;
}