/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-31							**
******************************************************************/

//Tool.h

#ifndef TOOL_H_
#define TOOL_H_

#include "DataStruct.h"

class Tool
{
public:
	Tool(void);
	~Tool(void);

	//Force6转换为数组
	static void Force6ToArray(const Force6 &force, double tao[]);

	//数组转换为Force6
	static void ArrayToForce6(const double taoArray[], Force6 &tao);

	//Eta转换为数组
	static void Eta6ToArray(const Eta &eta, double etaArray[]);

	//数组转换为Eta
	static void ArrayToEta(const double etaArray[], Eta &eta);

	//Nu转换为数组
	static void Nu6ToArray(const Nu &nu, double nuArray[]);

	//数组转换为Nu
	static void ArrayToNu(const double nuArray[], Nu &nu);

	//将数值转换到-PI~PI
	static double infToPi(double inValue);

	//初始化Nu
	static void initNu(Nu &nu);

	//初始化Force6
	static void initForce6(Force6 &force);

	//设置Eta
	static Eta setEta(const double x, const double y, const double psi);

	//初始化Eta
	static void initEta(Eta &eta);

	//初始化目标eta
	static void initEtaTarget(Eta &eta, double x, double y, double psi);

	//高斯消元法求矩阵的逆
	static bool inv(double (*A)[DOF6], double (*B)[DOF6]);

	//矩阵乘法，6x6矩阵与6x1矩阵相乘
	static void multiMx(const double (*dataMx1)[DOF6], const double dataMx2[], double resultMx[]);

	//矩阵加法，6x6矩阵与6x6矩阵相加
	static void plusMx( const double (*dataMx1)[DOF6], const double (*dataMx2)[DOF6], double (*resultMx)[DOF6]);

	//矩阵加法，3x3矩阵与3x1向量相乘
	static void multiVector(double A[][DOF3], double B[], double C[], int sz);

	//六自由度的力转换为三维数组
	static void Force6ToArr3(Force6 force, double arr[], int sz);

	//六自由度的速度角速度转换为三维数组
	static void NuToArr3(Nu nu0, double arr[], int sz);

	//六自由度位置姿态转换为三维数组
	static void EtaToArr3(Eta eta0, double arr[], int sz);

	//三维数组转换为六自由度位置姿态
	static void Arr3ToEta(double arr[], Eta &eta0, int sz);

	//三维数组相减
	static void subArr3(double arr1[], double arr2[], double arr[], int sz);

	//三维数组相加
	static void addArr3(double arr1[], double arr2[], double arr[], int sz);

	//旋转矩阵
	static void rotMat(double psi, double inValue[], double outValue[], int sz);
	static void transRot(double psi, double inValue[], double outValue[], int sz);

	//力由北东坐标系转换为船体坐标系
	static Force6 NedToboat(const Force6 &force, const Eta &eta);
};

#endif//TOOL_H_