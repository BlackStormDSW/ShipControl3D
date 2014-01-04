/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//Current.h

#ifndef CURRENT_H_
#define CURRENT_H_

#include "DataStruct.h"

#define NUM		37

class Current
{
public:
	Current(void);
	~Current(void);

	//初始化
	void init();

	//输入流速和入射角(°)
	void setPara(const double speed, const double dir);

	//输入船舶艏向(弧度)
	void setPsi(const double psiIn);
	
	//计算流力
	void cal();

	//输出流力
	Force6 force();

	//插值，计算流力系数
	//输入：流力系数向量，总数，入射角
	double interp(const double *cc, const double num, const double bt);

private:
	//流力系数向量
	double cxc[NUM], cyc[NUM], cnc[NUM];
	//流力系数
	double cxcValue, cycValue, cncValue;
	//垂线间长
	double Lpp;
	//平均吃水
	double Tm;
	//北东坐标下的流速、流向(弧度)
	double Vc, dirC;
	//流的入射角(弧度)
	double beta;
	//船舶艏向
	double psi;
	//流力
	Force6 forceCur;
	//海水密度
	double rho_w;
};

#endif//CURRENT_H_