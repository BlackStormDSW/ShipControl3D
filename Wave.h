/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//Wave.h
//参考 matlab 中函数 Wave_init.m

#ifndef WAVE_H_
#define WAVE_H_

#include "DataStruct.h"

class Wave
{
public:
	Wave();
	Wave(double dir, double Hs);

	~Wave();

	//初始化参数
	void init();

	//设置Data
	void setData(Data *dt);

	//设置浪高和浪向
	void setPara(double Hs, double ang);

	//计算
	void calWave();

	//计算阶乘
	double fact(int n);

	//波浪谱(ITTC双参数普)
	double waveSpec(double Hs, double T0, double omega);

	//设置w向量
	void setWVec(double w[], int size);

	//获取波浪幅值
	double *getZeta();
	//获取波浪方向
	double *getPsi();
	//获取波浪频率
	double *getOmega();
	//获取波浪数
	double *getWaveNum();
	//获取波浪随机相位
	double *getPhase();
	//获取结果向量的维数
	int getDim();

	//获取艏向值的在艏向列表中的值(2*pi分为36份)
	void headValue(double head, double &value1, double &value2);

	//获取Psi值的索引(从0开始，共36个索引)
	int getIndexPsi(double ps);
	
	//获取w值的索引(从0开始，共36个索引)
	int getIndexW(double omega);

	//计算位置相位
	double *posPhase(const Eta &eta, double time);


	//计算依据艏向的一阶二阶波浪力
	void calLoadForHead(double WF[], double DF[], const Eta &eta, double head, double time);

	//计算一阶二阶波浪力
	void calLoad(double WaveF[], double WaveD[]);

	//计算波浪力
	void cal(const Eta &eta, double time_);

	//获取一阶二阶波浪力
	void getLoad(Force6 &waveF, Force6 &waveD);
	
private:
	double fCut;	//截断频率 Frequencies cutoff factor
	double dirCut;	//截断方向 Direction cutoff factor
	double nFreq;	//频率数   Number of frequencies in grid
	double nDir;	//方向数   Number of directions in grid
	double psiMean;	//平均浪向(rad)	Mean wave direction
	double hs;		//有义波高(m)		Significant wave height Hs
	double omegaPeak;	//峰值频率(rad/s) Peak frequency omega_0
	int	   spread;	//波浪传播因子	Wave spreading factor
	double engLim;	//wave coomponent energy limit
	double depth;	//平均水深(m)		Average water depth

	double omegaMax;	//
	double deltaOmega;	//频率间隔
	double deltaPsi;	//艏向间隔
	double psiStart;	//起始艏向
	double psiMax;		//最大艏向
	double engMean;		//能量均值
	double engComp;		//
	double K;			//系数
	double Spsi;		//
	double *spec;		//波浪谱
	//double *omegaVec;	//记录omega
	double *omVec;		//
	double **psiVec;	//记录psi
	double Stot;		//Spectrum value;
	double nWaves;		//波浪总数
	double *wVec;
	int    wSize;
	double wt;			//权值

	int *idxPsi, *idxW;
	double *posPhs;

	double WaveF1[DOF6], WaveD1[DOF6], WaveF2[DOF6], WaveD2[DOF6],
		 WaveF[DOF6], WaveD[DOF6];

	Data *data;

	//输出结果
	double *Zeta;		//
	double *Omega;
	double *Phase;
	double *WaveNum;		//波浪数
	double *Psi;

	int kId;
};

#endif//WAVE_H_