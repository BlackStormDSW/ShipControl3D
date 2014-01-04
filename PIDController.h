/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//PIDController.h

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "DataStruct.h"
#include <fstream>

class PIDController
{
public:
	PIDController(void);
	~PIDController(void);

	void init();
	void initTStep(double val);
	void initPID(double KpVal = 0.15, double KiVal = 0.0, double KdVal = 0); //初值P:0.01, I:0.0003, D:0.2
	void setTarget(Eta eta);
	void setEta(Eta eta);

	void calculat();
	//输出力
	Force6 getTao();
private:
	Force6 outTao;
	double taoArr[DOF3];
	double etaPre[DOF3], etaErrPre[DOF3], etaTgt[DOF3], etaErr[DOF3], etaArr[DOF3];
	double etaErrErr[DOF3], etaErrSum[DOF3];
	double tStep;
	double Kp, Ki, Kd;
	Eta etaTarget, etaCurrent;
	double M[DOF3][DOF3], Mp[DOF3][DOF3], Mi[DOF3][DOF3], Md[DOF3][DOF3];
	double pRst[DOF3], iRst[DOF3], dRst[DOF3];

	ofstream errFile, pidFile;
};


#endif//PIDCONTROLLER_H_