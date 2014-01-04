/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//ShipControl.h

#ifndef SHIPMODEL_H_
#define SHIPMODEL_H_

#include "DataStruct.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <list>
using namespace std;

class ShipModel
{
public:
	ShipModel(void);
	~ShipModel(void);

	//初始化
	void init();

	//初始化船舶参数
	void setData(Data *data_);

	//设置时间间隔
	void setStep(const double step);

	//输入力
	void setForce(Force6 force);

	//输入船舶的初始位置姿态
	void setInitEta(Eta etaIn);

	//输出位置姿态
	Eta getEta();

	//输出速度角速度
	Nu getNu();

	//计算M矩阵
	void calM();

	//Cross-flow drag and surge resistance
	Force6 crosFlowDrag(Nu nu);

	//积分求Y和N Drag时所用函数
	double func(const Nu &nu, const double &x, bool flag=true);

	//积分,累加
	double integrt(Nu &nu, double max, double min, double delta, bool flag=true);
	//Hoerner方程
	double Hoerner(double BValue,double TValue);
	//两个数交换
	void swap(double &a, double &b);
	
	//对状态方程求解
	double solStateSpaceFunc(const double (*Ar)[visDampNum], const double Br[], const double Cr[], const double Dr, const double var, double xTemp[]);

	//计算粘滞阻尼矩阵
	Force6 viscousDamp(const Nu &nu);

	//roll damping
	double rollDamp(const double &varIn);
	
	//船舶控制运行
	void cal();

	//六自由度北东坐标系下，线速度、角速度向位置、姿态转换
	//eta0:输入的北东坐标；nu：输入的船体坐标；
	//intev：时间间隔；输出为转换后的北东坐标
	void nuToEta(Eta &eta0, Nu nu, double intev);

private:
	//船舶参数
	Data *data;

	double M[DOF6][DOF6], datMinv[DOF6][DOF6];
	double MinvTao[DOF6];

	//旋转矩阵
	double RotMx[DOF3][DOF3], TransMx[DOF3][DOF3];

	//时间间隔
	double tStep;

	//流体记忆效应
	Force6 mu;
	//拖拽力
	Force6 drag;

	double muArray[DOF6], dragArray[DOF6], sprStifArray[DOF6], dampArray[DOF6];

	//风浪流环境力与推进器合力
	Force6 taoForce;
	double taoArray[DOF6];

	//所有力的合力
	Force6 totalForce;
	double totForceArray[DOF6];

	//船舶位置姿态
	Eta eta;
	double etaArray[DOF6];

	//船舶速度角速度
	Nu nu;
	double nuArray[DOF6];
};

#endif//SHIPMODEL_H_