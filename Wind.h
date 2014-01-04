/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//Wind.h

#ifndef WIND_H_
#define WIND_H_

#include "DataStruct.h"

#define rowPara 19
#define colPara 7

class Wind
{
public:
	Wind(void);
	~Wind(void);

	//初始化参数
	void init();

	//设置风速风向
	void setPara(double speedTrue, double angTrue);

	//设置船舶艏向,弧度
	void setHead(const double head);

	//设置船舶速度
	void setNu(const Nu nuShip);

	//获取风对船舶的作用
	Force6 getWindTao();

	//计算风力
	void cal();

	//差值函数 遭遇角为角度
	void interp(const double valInit[rowPara][colPara], double result[colPara], double ang);

private:
	double L_oa;  			//船全长 (m)
	double B0;	            //船宽 (m)
	double H_s;             //船舶在水线以上部分侧面中心到水线距离 (m)
	double A_s;             //水线以上船舶侧投影面积 (m^2)
	double A_f;             //水线以上船舶正投影面积 (m^2)--横剖面投影
	double A_ss;            //（船）上层建筑的侧投影面积 (m^2)
	double wind_c;	        //除吃水线和船桅、通气设备等细长体外的船模型的侧投影周长 (m)
	double wind_e;			//从船首到侧投影面积的矩心的距离 (m)
	double wind_M;          // 正投影面内可见的船桅或中柱的不同组数；不包括紧贴船桥前部的中柱
	double rho_a;           //20 C 的空气密度 kg/m^3
	double V_T;             //绝对风速
	double ang_T;			//绝对风向
	double angEnc;			//遭遇角
	double psi;				//艏向角
	double V_R;				//相对风速

	Nu nu;					//船舶速度

	Force6 tao;				//风对船舶的作用

	//xyn方向风压力系数矩阵
	double AA[rowPara][colPara], BB[rowPara][colPara], CC[rowPara][colPara];
	double A[rowPara], B[rowPara], C[rowPara];
	double Cx, Cy, Cn;
};


#endif//WIND_H_