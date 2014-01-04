/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//ShipParameter.h

#ifndef SHIPPARAMETER_H_
#define SHIPPARAMETER_H_

#include <mat.h>
#include "DataStruct.h"

class ShipParameter
{
public:
	ShipParameter(void);
	~ShipParameter(void);

	//读取mat文件
	void readMat(string fileName);

	//输出数据
	Data getData();

private:
	Data data;
	int rowCount, columnCount;
	const mwSize *dimAmpRAO, *dimPhaseRAO, *dimAmpDrift, *dimPhaseDrift, *dimOmegaRAO, *dimOmegaDrift;
	double *initMRB, *velocities;
	char *vesselName;
	double *Lpp, *T, *B, *rho, *m, *nabla, *GM_L, *GM_T, *C_B, *Lwl, *S, *CG, *CB;
	//double **dataA;
	//forceRAO与driftdir中各数据指针
	double *dataAmpRAO, *dataPhaseRAO, *dataAmpDrift, *dataPhaseDrift, *dataOmegaRAO, *dataOmegaDrift;
	//输入和输出mat文件句柄
	MATFile *pmatFileIn;
	//forceRAO与driftdir中胞元数组
	mxArray *ampRAO[DOF6], *phase[DOF6], *ampDrift[DOF6];
	//输入的mat文件中包含的矩阵或结构体
	mxArray *vessel, *vesselABC, *M, *D, *CD, *MRB, *main,*forceRAO, *driftfrc, *veloc;
	const mxArray *pmxName;
	mxArray *ampForceRAO, *phaseForceRAO, *wForceRAO;
	mxArray *ampDriftfrc, *phaseDriftfrc, *wDriftfrc;

	mxArray *AinfABC, *BinfABC, *ArABC, *BrABC, *CrABC, *DrABC;
	mxArray *B44_infABC, *A44ABC, *B44ABC, *C44ABC, *D44ABC, *MAABC, *GABC, *MinvABC;
	mxArray *r_gABC, *AABC, *BABC, *freqsABC; 
	double *dataAinf, *dataBinf, *dataAr, *dataBr, *dataCr, *dataDr;
	double *dataB44_inf, *dataA44, *dataB44, *dataC44, *dataD44, *dataMA, *dataG, *dataMinv;
	double *datar_g, *dataA, *dataB, *dataFreqs;

	double *dataM, *dataD, *dataCD;
};


#endif//SHIPPARAMETER_H_