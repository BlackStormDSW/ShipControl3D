/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-31							**
******************************************************************/

//ShipControl.h

#ifndef SHIPCONTROL_H
#define SHIPCONTROL_H

#include "DataStruct.h"
#include "ShipModel.h"
#include "PIDController.h"
#include "NMPCcontroller.h"
#include "OptController.h"
#include "EnvObserver.h"
#include "Filter.h"
#include "WOPC.h"
#include "Wind.h"
#include "Wave.h"
#include "Current.h"
#include <fstream>
#include <QThread>

class ShipControl : public QThread
{
    Q_OBJECT
public:
	ShipControl(void);
	~ShipControl(void);

	//初始化
	void init();

	//设置船舶参数
	void setData(Data *data_);
	
	//打开文件
	void openFiles();

	//关闭文件
	void closeFiles();

	//设置参数
	void setParameter();

	//输出位置姿态
	Eta getEta();
	
	//开始进行船舶控制
	void startRun();

	//停止进行船舶控制
	void stopRun();

	//控制计算
	void cal();

protected:
    //船舶控制运行
    void run();
		
	//重载<<操作符
	friend ostream& operator << (ostream &os, const Eta &eta);

private:
	Data *data;

	DataSetStruct *dataSet;

	ShipModel model;
	
	double tStep, time;	//整个仿真计算过程步长
	ofstream etaFile, outFltFile, nuFile;
	ofstream centerFile, optHeadFile, targetFile;
	ofstream wave1File, wave2File, windFile, curFile;
	ofstream taoFile, thrustFile;
	ofstream paraFile;

	//船舶位置姿态，目标位置姿态， 滤波后的位置姿态
	Eta eta, etaTarget, etaFlt;
	
	//船舶速度角速度
	Nu nu;

	//风干扰
	Wind wind;

	//波浪干扰
	Wave wave;

	//海流干扰
	Current cur;

	//推进器推力
	Force6 thrust;

	//风的作用力
	Force6 windForce;

	//波浪力
	Force6 wave1Force, wave2Force;

	//流的作用力
	Force6 curForce;

	double thrustArray[DOF6], wave1Array[DOF6], wave2Array[DOF6],
		windArray[DOF6], curArray[DOF6];

	//推进器与环境的合力
	Force6 tao;
	double taoArray[DOF6];

	//PID控制器
	PIDController pid;

	//NMPC控制器
	NMPCcontroller nmpc;

	//环境最优艏向控制器
	OptController optPsiCtrl;

	//WOPC
	WOPC wopc;

	//环境观测器
	EnvObserver envObs;

	//创建滤波器
	Filter filter;

	//控制器计算的频率计数器
	int ctrlCount;
	//控制器计算周期(是0.05s的ctrlCyc倍)
	int ctrlCyc;
	
	//环境估计力
	Force3 envEst;

	//最优艏向
	double optPsi;

	//仿真总时间
	double maxTime;

	//运行标志
	bool runEnable;
};


#endif//SHIPCONTROL_H
