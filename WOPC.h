/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//WOPC.h

#ifndef WOPC_H_
#define WOPC_H_

#include "DataStruct.h"
#include <stdlib.h>

using namespace std;

class WOPC
{
public:
	WOPC(void);
	~WOPC(void);

	//初始化
	void init();

	//计算虚拟圆心
	void centerControl();

	//计算环境最优艏向
	void optHeadCal();

	//环境最优艏向控制
	void wohc();

	//控制计算
	void calculat();

	//设置时间间隔
	void setStep(const double dtime);

	//设置虚拟圆的半径
	void setRadius(const double radius);

	//设置目标位置
	void setPos(const Eta etaTarget);

	//实时获取当前位置与艏向
	void setEta(const Eta etaIn);

	//实时获取当前速度与角速度
	void setNu(const Nu nuIn);

	//实时获取当前的侧推
	void setThrust(Force6 thrust);

	//输出虚拟圆心位置
	pair<double, double> getCenterPos();

	//输出实时目标位置
	pair<double, double> getRTPosDes();

	//输出实时目标艏向(稳定时便是环境最优艏向)
	double getPsiRTDes();

	//将数值从(-pi,pi)转换到(-inf,inf)
	double piToInf(double valueIn);

private:

	double xDes, yDes;			// 目标位置
	double xCenter, yCenter;	// 虚拟圆心的位置
	double x, y, psi;			// 当前位置与姿态
	double u, v, r;				// 当前速度与角速度
	double xRTDes, yRTDes, psiRTDes;	// 实时目标位置与艏向
	double psiRTDesDegree;		// 角度形式的实时目标艏向
	double rad;					// 半径
	double radRT;				// 船舶与虚拟圆心的当前距离

	double psiC, prePsiC;		// 船舶当前位置与当前圆心连线的角度 与 前一时刻角度
	double psiD;				// 船舶目标位置与当前圆心连线的角度
	double psiCD;				// psiC与psiD之差
	double dpsiC;				// psiC的微分

	double kGain, hGain, mGain, pGain;	// 计算环境最优时的增益
	double mAdd;						// 计算环境最优时的附加量
	double upLmt, lowLmt;				// 计算环境最优时的上下限
	double ep;							// 计算环境最优时的中间变量

	double tStep;				// 步长
	bool usedFlag;				// 是否运行过的标志

	double xForce;				// 推进器输出的纵向力
	double yForce;				// 推进器输出的横向力

	int state;					//角度的象限
	double current_angle, previous_angle;
	double accumulate;	//
};

#endif//WOPC_H_
