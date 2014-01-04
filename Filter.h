/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//Filter.h
//²Î¿¼fossenµÄ¡¶DP_ForceRAO.mdl¡·ÖÐPassive DP wave filter

#pragma once

#include "DataStruct.h"

class Filter
{
public:
	Filter(void);
	~Filter(void);

	void init();

	void setEta(const Eta eta0);
	void setTao(const Force6 tao0);
	void setStep(const double step);
	Eta cal();
	Eta getEta();
	
private:
	Eta eta, etaResult;
	Force6 tao;
	double tStep;
	double Kw[DOF3][DOF3], KG[DOF3][DOF3], K3[DOF3][DOF3], K4[DOF3][DOF3],
		K11[DOF3][DOF3], K12[DOF3][DOF3], Ktb[DOF3][DOF3];
	double lambda[DOF3][DOF3], w_o[DOF3][DOF3], w_c[DOF3][DOF3], T_b[DOF3][DOF3];
	double M[DOF3][DOF3], D[DOF3][DOF3], invM[DOF3][DOF3];
	double bias[DOF3], taoArr[DOF3], Du[DOF3], nuArr[DOF3], K2u[DOF3],
		etaArr[DOF3], etaArrOut[DOF3], etaw[DOF3], xiw[DOF3];
	double Ku[DOF3], wou[DOF3];
	double etaResArr[DOF3];
};

