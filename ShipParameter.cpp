/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//ShipParameter.h

#include "ShipParameter.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>

ShipParameter::ShipParameter(void) :
rowCount(0), columnCount(0)
{
	memset(&data, 0, sizeof(Data));
	dimAmpRAO = new mwSize;
	pmatFileIn = NULL;
	vessel = NULL;
	vesselABC = NULL;
	M = NULL;
	D = NULL;
	CD = NULL;
}

ShipParameter::~ShipParameter(void)
{
	//delete dimAmpRAO;
}

//读取mat文件
void ShipParameter::readMat(string fileName)
{
	//以只读形式打开mat文件
	pmatFileIn = matOpen(fileName.c_str(), "r");
	if (NULL == pmatFileIn)
	{
		cout << "error open the file " << fileName << endl;
		exit(EXIT_FAILURE);
	}
	//将mat文件中的vessel、vesselABC结构体分别赋给vessel、vesselABC
	vessel = matGetVariable(pmatFileIn, "vessel");
	vesselABC = matGetVariable(pmatFileIn, "vesselABC");
	M = matGetVariable(pmatFileIn, "M");
	D = matGetVariable(pmatFileIn, "D");
	CD = matGetVariable(pmatFileIn, "CD_DATA");

	//将vessel结构体中的矩阵赋给相应变量
	main = mxGetField(vessel, 0, "main");
	MRB = mxGetField(vessel, 0, "MRB");
	veloc = mxGetField(vessel, 0, "velocities");
	forceRAO = mxGetField(vessel, 0, "forceRAO");
	driftfrc = mxGetField(vessel, 0, "driftfrc");
	ampForceRAO = mxGetField(forceRAO, 0, "amp");
	phaseForceRAO = mxGetField(forceRAO, 0, "phase");
	wForceRAO = mxGetField(forceRAO, 0, "w");
	ampDriftfrc = mxGetField(driftfrc, 0, "amp");
	phaseDriftfrc = mxGetField(driftfrc, 0, "phase");
	wDriftfrc = mxGetField(driftfrc, 0, "w");

	//将vesselABC结构体中的各矩阵赋给相应变量
	AinfABC = mxGetField(vesselABC, 0, "Ainf");
	BinfABC = mxGetField(vesselABC, 0, "Binf");
	ArABC = mxGetField(vesselABC, 0, "Ar");
	BrABC = mxGetField(vesselABC, 0, "Br");
	CrABC = mxGetField(vesselABC, 0, "Cr");
	DrABC = mxGetField(vesselABC, 0, "Dr");
	B44_infABC = mxGetField(vesselABC, 0, "B44_inf");
	A44ABC = mxGetField(vesselABC, 0, "A44");
	B44ABC = mxGetField(vesselABC, 0, "B44");
	C44ABC = mxGetField(vesselABC, 0, "C44");
	D44ABC = mxGetField(vesselABC, 0, "D44");
	MAABC = mxGetField(vesselABC, 0, "MA");
	GABC = mxGetField(vesselABC, 0, "G");
	MinvABC = mxGetField(vesselABC, 0, "Minv");
	r_gABC = mxGetField(vesselABC, 0, "r_g");
	AABC = mxGetField(vesselABC, 0, "A");
	BABC = mxGetField(vesselABC, 0, "B");
	freqsABC = mxGetField(vesselABC, 0, "freqs");

	/*读取mat文件中的main并赋给程序结构体中*/
	//main结构体中的各数据赋给程序结构体的main中各元素
	pmxName = mxGetField(main, 0, "name");
	int strSize = (int)mxGetNumberOfElements(pmxName)+1;
	vesselName = (char *)mxCalloc(strSize, sizeof(char));
	mxGetString(pmxName, vesselName,(mwSize)strSize);
	Lpp = (double *)mxGetData(mxGetField(main, 0, "Lpp"));
	T = (double *)mxGetData(mxGetField(main, 0, "T"));
	B = (double *)mxGetData(mxGetField(main, 0, "B"));
	rho = (double *)mxGetData(mxGetField(main, 0, "rho"));
	m = (double *)mxGetData(mxGetField(main, 0, "m"));
	nabla = (double *)mxGetData(mxGetField(main, 0, "nabla"));
	GM_L = (double *)mxGetData(mxGetField(main, 0, "GM_L"));
	GM_T = (double *)mxGetData(mxGetField(main, 0, "GM_T"));
	C_B = (double *)mxGetData(mxGetField(main, 0, "C_B"));
	Lwl = (double *)mxGetData(mxGetField(main, 0, "Lwl"));
	S = (double *)mxGetData(mxGetField(main, 0, "S"));
	CG = (double *)mxGetData(mxGetField(main, 0, "CG"));
	CB = (double *)mxGetData(mxGetField(main, 0, "CB"));

	//data.main.shipName = vesselName;
	data.dataVes.main.Lpp = *Lpp;
	data.dataVes.main.T = *T;
	data.dataVes.main.B = *B;
	data.dataVes.main.rho = *rho;
	data.dataVes.main.m = *m;
	data.dataVes.main.nabla = *nabla;
	data.dataVes.main.GM_L = *GM_L;
	data.dataVes.main.GM_T =*GM_T;
	data.dataVes.main.C_B = *C_B;
	data.dataVes.main.Lwl = *Lwl;
	data.dataVes.main.S = *S;
	data.dataVes.main.CG.x = CG[0];
	data.dataVes.main.CG.y = CG[1];
	data.dataVes.main.CG.z = CG[2];
	data.dataVes.main.CB.x = CB[0];
	data.dataVes.main.CB.y = CB[1];
	data.dataVes.main.CB.z = CB[2];

	velocities = (double *)mxGetData(veloc);
	for (int velNo = 0; velNo < velTotal; ++ velNo)
	{
		data.dataVes.vel[velNo] = velocities[velNo];
	}

	/************************************************************************/
	/*          将vessel中的数据赋给data结构体中各元素                           */
	/************************************************************************/
	/*读取mat文件中的MRB并赋给程序结构体中*/
	//MRB矩阵中的数据赋给initMRB
	initMRB = (double *)mxGetData(MRB);
	//MRB矩阵的行与列
	rowCount = (int)mxGetM(MRB);
	columnCount = (int)mxGetM(MRB);
	if (DOF6 == rowCount && DOF6 == columnCount)
	{
		//对MRB变量赋值，并显示
		for (int row = 0; row < rowCount; ++ row)
		{
			for (int column = 0; column < columnCount; ++ column)
			{
				data.dataVes.MRB[row][column] = initMRB[columnCount*row + column];
			}
		}
	} else {
		cout << "mat 文件中的 MRB不是6阶方阵， 请检查后再次运行程序！" << endl;
		exit(EXIT_FAILURE);
	}

	/*读取mat文件中的forceRAO并赋给程序结构体中*/
	//forceRAO中amp矩阵中的数据赋给amp
	for (int i = 0; i < DOF6; ++ i)
	{
		//将amp、phase中每个胞元单独拆出来
		ampRAO[i] = mxGetCell(ampForceRAO, i);
		phase[i] = mxGetCell(phaseForceRAO, i);
		ampDrift[i] = mxGetCell(ampDriftfrc, i);

		//ampRAO三层下标，phase、ampDrift与ampRAO一致
		dimAmpRAO = mxGetDimensions(ampRAO[i]);
		//将amp[i]首元素指针赋给dataAmpRAO
		dataAmpRAO = (double *)mxGetPr(ampRAO[i]);
		dataPhaseRAO = (double *)mxGetPr(phase[i]);
		dataAmpDrift = (double *)mxGetPr(ampDrift[i]);
		//将mat文件中amp各元素数据赋给结构体中amp相应元素
		for (int page = 0; page < velTotal/*dimAmpRAO[2]*/; ++ page)
		{
			for (int row = 0; row < dimAmpRAO[0]; ++ row)
			{
				for (int column = 0; column < dimAmpRAO[1]; ++ column)
				{
					data.dataVes.forceRAO.amp[i][page][row][column] =
						dataAmpRAO[page*dimAmpRAO[0]*dimAmpRAO[1]+column*dimAmpRAO[0]+row];
					data.dataVes.forceRAO.phase[i][page][row][column] =
						dataPhaseRAO[page*dimAmpRAO[0]*dimAmpRAO[1]+column*dimAmpRAO[0]+row];
					data.dataVes.driftfrc.amp[i][page][row][column] =
						dataAmpDrift[page*dimAmpRAO[0]*dimAmpRAO[1]+column*dimAmpRAO[0]+row];
				}
			}
		}
	}
	//将mat文件中Omega相应数据提取出来
	dimOmegaRAO = mxGetDimensions(wForceRAO);
	dataOmegaRAO = (double *)mxGetPr(wForceRAO);
	dataOmegaDrift = (double *)mxGetPr(wDriftfrc);
	for (int i = 0; i < dimOmegaRAO[1]; ++ i)
	{
		data.dataVes.forceRAO.w[i] = dataOmegaRAO[i];
		data.dataVes.driftfrc.w[i] = dataOmegaDrift[i];
	}

	/************************************************************************/
	/*              将vesselABC中的数据赋给dataVessel结构体中各元素              */
	/************************************************************************/
	dataAinf = (double *)mxGetData(AinfABC);
	dataBinf = (double *)mxGetData(BinfABC);
	dataAr = (double *)mxGetData(ArABC);
	dataBr = (double *)mxGetData(BrABC);
	dataCr = (double *)mxGetData(CrABC);
	dataDr = (double *)mxGetData(DrABC);
	dataMA = (double *)mxGetData(MAABC);
	dataG = (double *)mxGetData(GABC);
	dataMinv = (double *)mxGetData(MinvABC);
	for (int i = 0; i < DOF6; ++ i)
	{
		for (int j = 0; j < DOF6; ++ j)
		{
			data.dataVesABC.Ainf[i][j] = dataAinf[i*DOF6+j];
			data.dataVesABC.Binf[i][j] = dataBinf[i*DOF6+j];
			data.dataVesABC.MA[i][j] = dataMA[i*DOF6+j];
			data.dataVesABC.G[i][j] = dataG[i*DOF6+j];
			data.dataVesABC.Minv[i][j] = dataMinv[i*DOF6+j];
			for (int k = 0; k < velTotal; ++ k)
			{
				for (int m = 0; m < velTotal; ++ m)
				{
					//注意矩阵中行和列的顺序
					data.dataVesABC.Ar[i][j][k][m] = dataAr[(((i*DOF6)+j)*velTotal+m)*velTotal+k];
				}
				data.dataVesABC.Br[i][j][k] = dataBr[((i*DOF6)+j)*velTotal+k];
				data.dataVesABC.Cr[i][j][k] = dataCr[((i*DOF6)+j)*velTotal+k];
			}
			data.dataVesABC.Dr[i][j] = dataDr[(i*DOF6)+j];
		}
	}
	dataB44_inf = (double *)mxGetData(B44_infABC);
	dataA44 = (double *)mxGetData(A44ABC);
	dataB44 = (double *)mxGetData(B44ABC);
	dataC44 = (double *)mxGetData(C44ABC);
	dataD44 = (double *)mxGetData(D44ABC);
	for (int i = 0; i < velTotal; ++ i)
	{
		data.dataVesABC.B44_inf[i] = dataB44_inf[i];
		data.dataVesABC.D44[i] = dataD44[i];
		for (int j = 0; j < velTotal; ++ j)
		{
			data.dataVesABC.B44[i][j] = dataB44[i*velTotal+j];
			data.dataVesABC.C44[i][j] = dataC44[i*velTotal+j];
			for (int k = 0; k < velTotal; ++ k)
			{
				data.dataVesABC.A44[i][j][k] = dataA44[(i*velTotal+j)*velTotal+k];
			}
		}
	}

	datar_g = (double *)mxGetData(r_gABC);
	data.dataVesABC.r_g.x = datar_g[0];
	data.dataVesABC.r_g.y = datar_g[1];
	data.dataVesABC.r_g.z = datar_g[2];
	dataA = (double *)mxGetData(AABC);
	dataB = (double *)mxGetData(BABC);
	dataFreqs = (double *)mxGetData(freqsABC);
	for (int i = 0; i < freqNum; ++ i)
	{
		data.dataVesABC.freqs[i] = dataFreqs[i];
		for (int j = 0; j < DOF6; ++ j)
		{
			for (int k = 0; k < DOF6; ++ k)
			{
				data.dataVesABC.A[i][j][k] = dataA[(i*DOF6+j)*DOF6+k];
				data.dataVesABC.B[i][j][k] = dataB[(i*DOF6+j)*DOF6+k];

			}
		}
	}

	dataM = (double *)mxGetData(M);
	dataD = (double *)mxGetData(D);
	dataCD = (double *)mxGetData(CD);
	for (int i = 0; i < DOF6/2; ++ i)
	{
		for (int j = 0; j < DOF6/2; ++ j)
		{
			data.Mmx[i][j] = dataM[i*DOF6/2+j];
			data.Dmx[i][j] = dataD[i*DOF6/2+j];
		}
	}
	for (int i = 0; i < 2; ++ i)
	{
		for (int j = 0; j < HoerNum; ++ j)
		{
			data.CDdata[i][j] = dataCD[i*HoerNum+j];
		}
	}

	//释放临时数据
	mxFree(dataAmpRAO);
	mxFree(dataPhaseRAO);
	mxFree(dataAmpDrift);
	mxFree(dataOmegaRAO);
	mxFree(dataOmegaDrift);

	//关闭打开的mat文件
	matClose(pmatFileIn);
}

//输出数据
Data ShipParameter::getData()
{
	return data;
}