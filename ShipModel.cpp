/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//ShipModel.cpp

#include "ShipModel.h"
#include "Tool.h"
#include <mat.h>
#include <iostream>
#include <QDebug>
#include <stdlib.h>
using namespace std;

ShipModel::ShipModel(void)
{
	//初始化
	init();	
}


ShipModel::~ShipModel(void)
{
	closeFiles();
}

//初始化
void ShipModel::init()
{	
	tStep = 0.0;

	CY_2D = 0.0;

	for (int i = 0; i < DOF6; i ++)
	{
		MinvTao[i] = 0.0;

		muArray[i] = 0.0;
		dragArray[i] = 0.0;
		sprStifArray[i] = 0.0;
		dampArray[i] = 0.0;

		taoArray[i] = 0.0;
		totForceArray[i] = 0.0;
		etaArray[i] = 0.0;
		nuArray[i] = 0.0;

		for (int j = 0; j < DOF6; j ++)
		{
			M[i][j]			= 0.0;
			datMinv[i][j]	= 0.0;
		}
	}

	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			RotMx[i][j]		= 0.0;
			TransMx[i][j]	= 0.0;
		}
	}

	data = new Data;

	//初始化船舶速度
	Tool::initNu(nu);
	Tool::initForce6(taoForce);

	openFiles();
}


//打开文件
void ShipModel::openFiles()
{
	dragFile.open("E:/projectProgram/data/drag.txt");
	muFile.open("E:/projectProgram/data/mu.txt");
	dampFile.open("E:/projectProgram/data/damp.txt");
	sprStifFile.open("E:/projectProgram/data/sprStif.txt");
}

//关闭文件
void ShipModel::closeFiles()
{
	dragFile.close();
	muFile.close();
	dampFile.close();
	sprStifFile.close();
}

//初始化船舶参数
void ShipModel::setData(Data *data_)
{
	data = data_;
}

//设置时间间隔
void ShipModel::setStep(const double step)
{
	tStep = step;
}

//输入力
void ShipModel::setForce(Force6 force)
{
	taoForce = force;
}

//输入船舶的初始位置姿态
void ShipModel::setInitEta(Eta etaIn)
{
	eta = etaIn;
}

//输出位置姿态
Eta ShipModel::getEta()
{
	return eta;
}

//输出速度角速度
Nu ShipModel::getNu()
{
	return nu;
}

//计算M矩阵
void ShipModel::calM()
{
	//计算惯性矩阵M
	Tool::plusMx(data->dataVesABC.Ainf, data->dataVes.MRB, M);
	//计算惯性矩阵M的逆
	Tool::inv(M, datMinv);

	CY_2D = Hoerner(data->dataVes.main.B, data->dataVes.main.T);
}

//Cross-flow drag and surge resistance
//给出的公式中，系数为正值，但simulink程序中系数是负值
Force6 ShipModel::crosFlowDrag(Nu nu)
{
	Force6 force;
	double rho_w, Lpp, B, T, Ax, Ay, CX;
	double xDrag = 0.0, yDrag = 0.0, nDrag = 0.0;
	rho_w = data->dataVes.main.rho;
	Lpp = data->dataVes.main.Lpp;
	B = data->dataVes.main.B;
	T = data->dataVes.main.T;
	Ax = 0.9*T*B;
	Ay = 0.9*T*Lpp;
	CX = 1;
	xDrag = -0.5*rho_w*CX*Ax*nu.u*fabs(nu.u);
	yDrag = -0.5*rho_w*Ay/Lpp*CY_2D*integrt(nu, 0.5*Lpp, -0.5*Lpp, 0.01, true);
	nDrag = -0.5*rho_w*Ay/Lpp*CY_2D*integrt(nu, 0.5*Lpp, -0.5*Lpp, 0.01, false);

	force.xForce = xDrag;
	force.yForce = yDrag;
	force.nMoment = nDrag;
	force.zForce = 0;
	force.kMoment = 0;
	force.mMoment = 0;
	return force;
}
//积分求Y和N Drag时所用函数;flag为真时，结果为求yDrag所用，否则，结果为求nDrag所用
double ShipModel::func(const Nu &nu, const double &x, bool flag)
{
	if (flag)
	{
		return (nu.v + x*nu.r)*fabs(nu.v + x*nu.r);
	} else {
		return x*(nu.v + x*nu.r)*fabs(nu.v + x*nu.r);
	}
}

//积分,累加
double ShipModel::integrt(Nu &nu, double max, double min, double delta, bool flag)
{
	double funPre = 0.0, funPost = 0.0;
	double result = 0.0;
	for (double elem = min; elem < max; elem += delta)
	{
		funPre = func(nu, elem, flag);
		funPost = func(nu, elem+delta, flag);
		result += funPre*delta + (funPost-funPre)*delta/2.0;
	}
	return result;
}

//Hoerner方程
double ShipModel::Hoerner(double BValue,double TValue)
{
	double x[] = {0.0108623, 0.176606, 0.353025, 0.451863, 0.472838, 0.492877,
		0.493252, 0.558473, 0.646401,	0.833589, 0.988002, 1.30807, 1.63918, 
		1.85998, 2.31288, 2.59998, 3.00877, 3.45075, 3.7379, 4.00309};
	double y[] = {1.96608, 1.96573, 1.89756, 1.78718, 1.58374, 1.21082, 1.27862, 
		1.08356, 0.998631, 0.87959, 0.828415, 0.759941, 0.691442, 0.657076, 
		0.630693, 0.596186, 0.586846, 0.585909, 0.559877, 0.559315};
	
	double hData = BValue/(2*TValue);

	double result = 0.0;
	if (hData>x[0]&&hData<x[HoerNum-1])
	{
		int index = 0;
		while (!(hData>=x[index]&&hData<=x[index+1]))
		{
			index ++;
		}
		result = (y[index+1]-y[index])/(x[index+1]-x[index])*(hData-x[index])+y[index];
	} else if (hData<x[0]) {
		result = (y[1]-y[0])/(x[1]-x[0])*(hData-x[0])+y[0];
	} else {
		result = (y[HoerNum-1]-y[HoerNum-2])/(x[HoerNum-1]-x[HoerNum-2])*(hData-x[HoerNum-2])+y[HoerNum-2];
	}
	return result;
}

//对状态方程求解
double ShipModel::solStateSpaceFunc(const double (*Ar)[visDampNum], const double Br[], const double Cr[], const double Dr, const double var, double xTemp[])
{
	double x[visDampNum] = {0.0}, y = 0.0;

	for (int i = 0; i < visDampNum; ++ i)
	{
		for (int j = 0; j < visDampNum; ++ j)
		{
			x[i] += Ar[i][j]*xTemp[j];
		}
		x[i] = xTemp[i] + (x[i] + Br[i]*var)*tStep;
	}

	for (int i = 0; i < visDampNum; ++ i)
	{
		y += Cr[i]*x[i];
		xTemp[i] = x[i];
	}
	y += Dr*var;
	return y;
}

//计算粘滞阻尼矩阵
Force6 ShipModel::viscousDamp(const Nu &nu)
{
	Force6 muResult;
	double result[DOF6] = {0};
	static double xArray[DOF6][DOF6][visDampNum] = {0.0};
	double varIn[DOF6] = {nu.u, nu.v, nu.w, nu.p, nu.q, nu.r};

	for (int i = 0; i < DOF6; ++ i)
	{
		for (int j = 0; j < DOF6/2; ++ j)
		{

			if (3 == i && 1 == j)
			{
				result[i] += rollDamp(varIn[j*2+1]);
			} else {
				if (0 == i%2)
				{
					result[i] += solStateSpaceFunc(data->dataVesABC.Ar[i][j*2], data->dataVesABC.Br[i][j*2], data->dataVesABC.Cr[i][j*2], data->dataVesABC.Dr[i][j*2], varIn[j*2], xArray[i][j*2]);
				} else {
					result[i] += solStateSpaceFunc(data->dataVesABC.Ar[i][j*2+1], data->dataVesABC.Br[i][j*2+1], data->dataVesABC.Cr[i][j*2+1], data->dataVesABC.Dr[i][j*2+1], varIn[j*2+1], xArray[i][j*2+1]);
				}
			}
		}
	}
	Tool::ArrayToForce6(result, muResult);

	return muResult;
}

//roll damping
double ShipModel::rollDamp(const double &varIn)
{
	double kVisc = 0.0, kAdd = 0.0;
	double BValue[visDampNum], x[visDampNum] = {0.0};
	static double xTemp[visDampNum] = {0.0};
	kAdd = varIn*(data->dataVesABC.B44_inf[0]+data->dataVesABC.D44[0]);
	for (int i = 0; i < visDampNum; ++ i)
	{
		BValue[i] = varIn*data->dataVesABC.B44[0][i];
		for (int j = 0; j < visDampNum; ++ j)
		{
			x[i] += data->dataVesABC.A44[0][i][j] * xTemp[j];
		}
		x[i] += BValue[i];
		xTemp[i] = x[i]*tStep;
	}

	for (int i = 0; i < visDampNum; ++ i)
	{
		kVisc += xTemp[i] * data->dataVesABC.C44[0][i];
	}
	kVisc += kAdd;
	return kVisc;
}

//船舶控制运行
void ShipModel::cal()
{
	Tool::Force6ToArray(taoForce, taoArray);
	//计算合力
	for (int i = 0; i < DOF6; ++ i)
	{
		totForceArray[i] = taoArray[i] + dragArray[i] - sprStifArray[i] - muArray[i] - dampArray[i];
	}

	Tool::multiMx(datMinv, totForceArray, MinvTao);

	//计算船舶速度
	for (int i = 0; i < DOF6; ++ i)
	{
		nuArray[i] += MinvTao[i]*tStep;
	}

	Tool::ArrayToNu(nuArray, nu);

	//计算Spring stiffness, damping
	Tool::multiMx(data->dataVesABC.Binf, nuArray, dampArray);

	//h-frame potential / viscous damping
	mu = viscousDamp(nu);

	//Cross-flow drag and surge resistance
	drag = crosFlowDrag(nu);

	//计算船舶位置和姿态
	nuToEta(eta, nu, tStep);
	
	//六自由度的力和力矩转化为数组
	Tool::Eta6ToArray(eta, etaArray);
	Tool::Force6ToArray(mu, muArray);
	Tool::Force6ToArray(drag, dragArray);

	//计算Spring stiffness
	Tool::multiMx(data->dataVesABC.G, etaArray, sprStifArray);

	dragFile << dragArray[0] << "\t" << dragArray[1] << "\t" << dragArray[2] << "\t"
		<< dragArray[3] << "\t" << dragArray[4] << "\t" << dragArray[5] << endl;

	muFile << muArray[0] << "\t" << muArray[1] << "\t" << muArray[2] << "\t"
		<< muArray[3] << "\t" << muArray[4] << "\t" << muArray[5] << endl;

	dampFile << dampArray[0] << "\t" << dampArray[1] << "\t" << dampArray[2] << "\t"
		<< dampArray[3] << "\t" << dampArray[4] << "\t" << dampArray[5] << endl;

	sprStifFile << sprStifArray[0] << "\t" << sprStifArray[1] << "\t" << sprStifArray[2] << "\t"
		<< sprStifArray[3] << "\t" << sprStifArray[4] << "\t" << sprStifArray[5] << endl;

}

//六自由度下，有北东坐标系向船体坐标系旋转
void ShipModel::nuToEta(Eta &eta, Nu nu, double intev)
{
	double phi = eta.phi;
	double theta = eta.theta;
	double psi = eta.psi;
	RotMx[0][0] = cos(psi)*cos(theta);
	RotMx[0][1] = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	RotMx[0][2] = sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta);
	RotMx[1][0] = sin(psi)*cos(theta);
	RotMx[1][1] = cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi);
	RotMx[1][2] = -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi);
	RotMx[2][0] = -sin(theta);
	RotMx[2][1] = cos(theta)*sin(phi);
	RotMx[2][2] = cos(theta)*cos(phi);

	TransMx[0][0] = 1;
	TransMx[0][1] = sin(phi)*tan(theta);
	TransMx[0][2] = cos(phi)*tan(theta);
	TransMx[1][0] = 0;
	TransMx[1][1] = cos(phi);
	TransMx[1][2] = -sin(phi);
	TransMx[2][0] = 0;
	TransMx[2][1] = sin(phi)/cos(theta);
	TransMx[2][2] = cos(phi)/cos(theta);

	eta.n += (RotMx[0][0]*nu.u+RotMx[0][1]*nu.v+RotMx[0][2]*nu.w)*intev;
	eta.e += (RotMx[1][0]*nu.u+RotMx[1][1]*nu.v+RotMx[1][2]*nu.w)*intev;
	eta.d += (RotMx[2][0]*nu.u+RotMx[2][1]*nu.v+RotMx[2][2]*nu.w)*intev;
	eta.phi   += (TransMx[0][0]*nu.p+TransMx[0][1]*nu.q+TransMx[0][2]*nu.r)*intev;
	eta.theta += (TransMx[1][0]*nu.p+TransMx[1][1]*nu.q+TransMx[1][2]*nu.r)*intev;
	eta.psi   += (TransMx[2][0]*nu.p+TransMx[2][1]*nu.q+TransMx[2][2]*nu.r)*intev;
}
