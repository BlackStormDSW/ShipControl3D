/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//filter.cpp
//参考《Handbook of Marine Craft Hydrodynamics and Motion Control》
//中 《Nonlinear Passive Observer Dsigns》 11.4一节

#include "filter.h"
#include "Tool.h"
#include <math.h>

Filter::Filter(void) :
tStep(0.05)
{
	init();
}


Filter::~Filter(void)
{
}

//初始化各参数
void Filter::init()
{
	for (int i = 0; i < DOF3; i ++)
	{
		bias[i] = 0.0;
		taoArr[i] = 0.0;
		Du[i] = 0.0;
		nuArr[i] = 0.0;
		etaArr[i] = 0.0;
		etaArrOut[i] = 0.0;
		etaw[i] = 0.0;
		xiw[i] = 0.0;
		Ku[i] = 0.0;
		wou[i] = 0.0;

		etaResArr[i] = 0.0;

		for (int j = 0; j < DOF3; j ++)
		{
			K11[i][j] = 0.0;
			K12[i][j] = 0.0;
			Kw[i][j] = 0.0;
			KG[i][j] = 0.0;
			Ktb[i][j] = 0.0;
			K3[i][j] = 0.0;
			K4[i][j] = 0.0;
			w_o[i][j] = 0.0;
			w_c[i][j] = 0.0;
			T_b[i][j] = 0.0;
			lambda[i][j] = 0.0;
			M[i][j] = 0.0;
			invM[i][j] = 0.0;
			D[i][j] = 0.0;

			if (i == j)
			{
				w_o[i][j] = 0.8976;
				w_c[i][j] = 1.0;
				T_b[i][j] = 100.0;
				lambda[i][j] = 0.1;
				K12[i][j] = 2.0*w_o[i][j]*(1-lambda[i][j]);
				K11[i][j] = -2.0*(1-lambda[i][j])*(w_c[i][j]-w_o[i][j]);
				KG[i][j] = 2.0*lambda[i][j]*w_o[i][j];
				Kw[i][j] = pow(w_o[i][j], 2.0);
				Ktb[i][j] = 1.0/T_b[i][j];
			}
		}
	}

	w_c[2][2] = 5.0;

	K3[0][0] = pow(40.0, 4.0)*0.1;
	K3[1][1] = pow(40.0, 4.0)*0.1;
	K3[2][2] = pow(40.0, 4.0)*0.1*0.1;

	K4[0][0] = pow(40.0, 4.0);
	K4[1][1] = pow(40.0, 4.0);
	K4[2][2] = pow(40.0, 4.0)*0.1;

	//系统惯性矩阵(原6自由度系统惯性矩阵简化而来)
	M[0][0] = 0.0026e+10;
	M[1][1] = 0.0033e+10;
	M[2][2] = 6.5209e+10;

	//阻尼矩阵(原6自由度阻尼矩阵简化而来)
	D[0][0] = 0.0002e+8;
	D[1][1] = 0.0022e+8;
	D[1][2] = -0.0177e+8;
	D[2][1] = -0.0177e+8;
	D[2][2] = 7.1506e+8;

	invM[0][0] = 1.0/M[0][0];
	invM[1][1] = 1.0/M[1][1];
	invM[2][2] = 1.0/M[2][2];

	//初始化eta输出值
	etaResult.n = 0.0;
	etaResult.e = 0.0;
	etaResult.d = 0.0;
	etaResult.phi = 0.0;
	etaResult.theta = 0.0;
	etaResult.psi = 0.0;

}

void Filter::setEta(const Eta eta0)
{
	eta.n = eta0.n;
	eta.e = eta0.e;
	eta.d = eta0.d;
	eta.phi = eta0.phi;
	eta.theta = eta0.theta;
	eta.psi = eta0.psi;
}

void Filter::setTao(const Force6 tao0)
{
	tao.xForce = tao0.xForce;
	tao.yForce = tao0.yForce;
	tao.zForce = tao0.zForce;
	tao.kMoment = tao0.kMoment;
	tao.mMoment = tao0.mMoment;
	tao.nMoment = tao0.nMoment;
}

void Filter::setStep(const double step)
{
	tStep = step;
}

//计算
Eta Filter::cal()
{
	double psi0 = eta.psi;
	static double etaArr[DOF3] = {0.0, 0.0, 0.0};
	double subArr[DOF3] = {0.0, 0.0, 0.0};
	double tao0[DOF3] = {0.0, 0.0, 0.0};
	double tao1[DOF3] = {0.0, 0.0, 0.0};
	double tao2[DOF3] = {0.0, 0.0, 0.0};
	double tao3[DOF3] = {0.0, 0.0, 0.0};
	static double invTb[DOF3] = {0.0, 0.0, 0.0};
	double K3u[DOF3] = {0.0, 0.0, 0.0};
	double K4u[DOF3] = {0.0, 0.0, 0.0};
	double K11u[DOF3] = {0.0, 0.0, 0.0};
	double K12u[DOF3] = {0.0, 0.0, 0.0};
	double psiWF[DOF3] = {0.0, 0.0, 0.0};
	double nuArr1[DOF3] = {0.0, 0.0, 0.0};
	double xiww[DOF3] = {0.0, 0.0, 0.0};
	//计算eta差，并将psi转换到-pi~pi内
	Tool::EtaToArr3(eta, etaArr, DOF3);

	Tool::addArr3(etaResArr, etaw, xiww, DOF3);
	Tool::subArr3(etaArr, xiww, subArr, DOF3);
	subArr[2] = Tool::infToPi(subArr[2]);

	//计算bias
	Tool::multiVector(K3, subArr, K3u, DOF3);
	for (int i = 0; i < DOF3; i ++)
	{
		bias[i] += (K3u[i]-invTb[i]) * tStep;
	}
	Tool::multiVector(Ktb, bias, invTb, DOF3);

	//计算附加tao 1
	Tool::Force6ToArr3(tao, taoArr, DOF3);
	Tool::transRot(psi0, bias, tao1, DOF3);
	Tool::addArr3(taoArr, tao1, tao0, DOF3);

	//计算附加tao 2
	Tool::multiVector(K4, subArr, K4u, DOF3);
	Tool::transRot(psi0, K4u, tao2, DOF3);

	//计算nu
	Tool::addArr3(tao0, tao2, tao0, DOF3);
	Tool::subArr3(tao0, Du, tao0, DOF3);
	Tool::multiVector(invM, tao0, tao3, DOF3);
	for (int i = 0; i < DOF3; i ++)
	{
		nuArr[i] += tStep * tao3[i];
	}
	Tool::multiVector(D, nuArr, Du, DOF3);

	//变换nu
	Tool::rotMat(psi0, nuArr, nuArr1, DOF3);
	//计算eta结果
	Tool::multiVector(w_c, subArr, K2u, DOF3);
	Tool::addArr3(nuArr1, K2u, nuArr1, DOF3);

	for (int i = 0; i < DOF3; i ++)
	{
		etaResArr[i] += nuArr1[i] * tStep;
	}
	//etaResArr[2] = Tool::infToPi(etaResArr[2]);

	//计算eta_w
	Tool::multiVector(K11, subArr, K11u, DOF3);
	Tool::multiVector(K12, subArr, K12u, DOF3);
	Tool::subArr3(K12u, Ku, K12u, DOF3);
	Tool::subArr3(K12u, wou, K12u, DOF3);

	for (int i = 0; i < DOF3; i ++)
	{
		etaw[i] += tStep * K12u[i];
	}

	Tool::addArr3(K11u, etaw, psiWF, DOF3);

	for (int i = 0; i < DOF3; i ++)
	{
		xiw[i] += tStep * psiWF[i];
	}
	Tool::multiVector(KG, etaw, Ku, DOF3);
	Tool::multiVector(Kw, xiw, wou, DOF3);

	Tool::Arr3ToEta(etaResArr, etaResult, DOF3);
	return etaResult;
}

Eta Filter::getEta()
{
	return eta;
}