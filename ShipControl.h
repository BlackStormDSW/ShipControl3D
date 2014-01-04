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

	//用户接口
	void userInterFace();

	//设置参数
    void setParameter();
		
	//重载<<操作符
	friend ostream& operator << (ostream &os, const Eta &eta);

protected:
    //船舶控制运行
    void run();

private:
	Data *data;

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

	//设定风速，风向(°)
	double SpeedWind, DirWind;

	//设定浪高，浪向(°)
	double HeightWave, DirWave;

	//设定流速，流向(°)
	double SpeedCurrent, DirCurrent;

	//初始位置与艏向
	double xOrigin, yOrigin, psiOrigin;

	//目标位置与艏向
	double xTarget, yTarget, psiTarget;

	//环境最优动力定位半径
	double radius;

	//控制器等的各种参数
	//PID参数
	double kp, ki, kd;
	//NMPC的预测周期
	double Tpre;
	//NMPC的权值
	double w1, w2, w3;
	//ZPC-W的三个参数
	double kpZ, kiZ, kdZ;
	//环境观测器的三个参数
	double k1Env, k2Env, k3Env;

	//动力定位任务类型：1.常规动力定位；2.环境最优动力定位
	int dpFlag;
	//动力定位控制方法：1.PID控制；2.非线性模型预测控制
	int ctlFlag;
	//环境最优动力定位控制策略类型：
	//1.WOPC与ZPC-W结合后的环境最优动力定位控制策略
	//2.WOPC借用环境估计的环境最优动力定位控制策略
	//3.ZPC-W环境最优动力定位控制策略
	int wopcFlag;

	//用户设置过程中使用的临时变量
	char xOriginStr[20], yOriginStr[20], PsiOriginStr[20];
	char *xOriginStrEnd, *yOriginStrEnd, *PsiOriginStrEnd;

	char xTargStr[20], yTargStr[20], PsiTargStr[20];
	char *xTargStrEnd, *yTargStrEnd, *PsiTargStrEnd;

	char SpeedWindStr[20], DirWindStr[20], HeightWaveStr[20], DirWaveStr[20], SpeedCurrentStr[20], DirCurrentStr[20];
	char *SpeedWindStrEnd, *DirWindStrEnd, *HeightWaveStrEnd, *DirWaveStrEnd, *SpeedCurrentStrEnd, *DirCurrentStrEnd;

	char dpFlagStr[20], ctlFlagStr[20], wopcFlagStr[20];

	char kpStr[20], kiStr[20], kdStr[20];
	char *kpStrEnd, *kiStrEnd, *kdStrEnd;

	char kpZStr[20], kiZStr[20], kdZStr[20];
	char *kpZStrEnd, *kiZStrEnd, *kdZStrEnd;

	char tStr[20], w1Str[20], w2Str[20], w3Str[20];
	char *tStrEnd, *w1StrEnd, *w2StrEnd, *w3StrEnd;

	double valueStr;
};


#endif//SHIPCONTROL_H
