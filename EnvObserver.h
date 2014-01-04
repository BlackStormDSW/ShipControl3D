/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-25							**
******************************************************************/

//EnvObserver.h

#ifndef ENVOBSERVER_H_
#define ENVOBSERVER_H_

#include "DataStruct.h"

class EnvObserver
{
public:
	EnvObserver(void);
	~EnvObserver(void);

	//初始化
	void init();

	//设置时间间隔
	void setStep(const double deltaT);

	//设置参数
	void setK(const double k1, const double k2, const double k3);

	//输入M矩阵(3维)
	void setM(const double (*Mmat)[DOF3]);

	//输入D矩阵(3维)
	void setD(const double (*Dmat)[DOF3]);

	//输入推进器推力与力矩
	void setTao(const Force6 thrust);

	//输入船舶运动速度
	void setNu(const Nu nuIn);

	//输出环境力(3自由度)
	Force3 force();

	//矩阵乘法，6x6矩阵与6x1矩阵相乘
	void multiMx(const double (*dataMx1)[DOF3], const double dataMx2[], double resultMx[]);

	//环境估计
	void cal();

private:
	//M、D矩阵
	double M[DOF3][DOF3], D[DOF3][DOF3];
	//参数K0
	double K0[DOF3][DOF3];
	//中间变量β、其导数与中间变量
	double beta[DOF3], dBeta[DOF3], betaTmp[DOF3];
	//环境估计力、力矩
	Force3 env;
	//推进器推力、力矩与环境估计力、力矩
	double tao[DOF3], envTao[DOF3];
	//速度项
	double nu[DOF3];
	//计算过程中间变量
	double D_nu[DOF3], M_nu[DOF3], K_M_nu[DOF3];
	//时间间隔
	double tStep;
};

#endif//ENVOBSERVER_H_
