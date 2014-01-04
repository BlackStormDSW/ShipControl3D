/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//OptController.h

#ifndef OPTCONTROLLER_H_
#define OPTCONTROLLER_H_

#include "DataStruct.h"
#include <list>
using namespace std;

#define COUNT	20
#define LMTX	10000
#define LMTY	10000

class OptController
{
public:
	OptController(void);
	~OptController(void);

	//初始化参数
	void init();

	//设置时间间隔
	void setStep(const double Intval);

	//设置当前环境最优艏向
	void setPsi(const double preTagHead);

	//设置当前推进器推力与转矩
	void setTao(const Force6 Thrust);

	//设置PID参数
	void setPID(const double Pval, const double Ival, const double Dval);

	//计算求均值
	double meanData(list<double> &valLst, const double inVal);

	//PID计算
	void cal();

	//输出环境最优艏向
	double OptPsi();

private:
	//时间间隔
	double step;

	//存储环境最优艏向的链表
	list<double> storOptPsi;
	//存储纵向力和横向力的列表
	list<double> storXForce, storYForce;	

	//计算环境最优艏向时使用的力与转矩
	double taoX, taoY, taoN, tagTaoY;
	//原始推力与转矩
	double xForce, yForce, nMoment;

	//误差、误差的微分、误差的积分和前一个误差
	double errY, diffErrY, intErrY, preErrY;

	//最优艏向
	double optPsi, preOptPsi;

	//调整最优艏向所使用的PID系数
	double P, I, D;

	//计算均值时使用的临时变量
	double sum;

	//纵向力正负标志
	double signX;
};

#endif//OPTCONTROLLER_H_