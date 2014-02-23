/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//NMPController.h

#ifndef NMPCONTROLLER_H_
#define NMPCONTROLLER_H_

#include "DataStruct.h"
#include <fstream>

#define MAXDIM		12
#define MAXSURGE	2500000
#define MAXSWAY		600000
#define MAXYAW		14000000

class NMPCcontroller
{
public:
	NMPCcontroller(void);
	~NMPCcontroller(void);

	//参数初始化
	void init();

	//计算M矩阵
	void calM();

	//控制器计算
	void cal();

	//设置预测周期
	void setT(const double period);

	//设置权值
	void setWeight(const double lmd1, const double lmd2, const double lmd3);

	//速度输入
	void setNu(const Nu nu);

	//位置输入
	void setEta(const Eta eta);

	//输入目标位置姿态
	void setTarget(const Eta eta);
	
	//输入外界干扰力
	void setEnv(const Force3 forcEnv);

	//控制力输出
	Force6 Force();

	//高斯消元法求矩阵的逆
	bool inv(double (*A)[DOF3], double (*B)[DOF3]);

private:
	double M[MAXDIM][MAXDIM], A[DOF3][MAXDIM], B[DOF3][DOF3],
		D[DOF3][MAXDIM], F[MAXDIM], U[DOF3];
	double C[DOF3][DOF3], invC[DOF3][DOF3];

	double M3[DOF3][DOF3];

	double q0[MAXDIM], q1[MAXDIM][DOF3], q1T[DOF3][MAXDIM];

	//目标输出值
	double yd[MAXDIM];
	
	double m[DOF3][DOF3], d[DOF3][DOF3];
	double a[DOF3][DOF3], b[DOF3];

	//位置与速度变量
	double x, y, psi, u, v, r;

	//预测时间长度T
	double T;

	//性能指标函数中的权值
	double lambda[DOF3];

	//推进器约束
	double lmtForce[DOF3];

	//力与力矩
	Force6 force;

	//外界环境力
	Force3 env;

	ofstream outFile;
};

#endif//NMPCONTROLLER_H_