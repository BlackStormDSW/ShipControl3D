/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-31							**
******************************************************************/

//ShipControl.cpp

#include "ShipControl.h"
#include "Tool.h"
#include <mat.h>
#include <stdlib.h>
#include <QTextStream>
#include <QDebug>

ShipControl::ShipControl(void) :
tStep(0.05), time(0)
{
	//初始化
	init();	
}


ShipControl::~ShipControl(void)
{
	//关闭文件
	closeFiles();

	//delete data;
}

//初始化
void ShipControl::init()
{
	//打开文件
	openFiles();

	//初始化程序计算的暂停状态
	pauseState = false;
	//初始化环境最优艏向
	optPsi = 0.0;
	
	//设定风速，风向
	dataSet.windSpeed	= 10.0;
	dataSet.windDir	= 90.0;

	//设定浪高，浪向
	dataSet.waveHeight	= 2.0;
	dataSet.waveDir	= 90.0;

	//设定流速，流向
	dataSet.curSpeed	= 0.5;
	dataSet.curDir		= 90.0;

	//初始化动力定位控制类型：
	//1.常规动力定位；
	//2.ZPC-W环境最优动力定位控制策略
	//3.WOPC与ZPC-W结合后的环境最优动力定位控制策略
	//4.WOPC借用环境估计的环境最优动力定位控制策略
	dataSet.dpMode = NORMAL_DP;
	//初始化动力定位控制方法：1.PID控制；2.非线性模型预测控制
	dataSet.ctrlType = NMPC_CTRL;

	//初始化PID参数	
	dataSet.kp = 0.15;
	dataSet.ki = 0.0;
	dataSet.kd = 10.0;

	//初始化NMPC的预测周期
	dataSet.tNMPC = 9.0;
	//初始化NMPC的三个权值
	dataSet.w1NMPC = 0.9;
	dataSet.w2NMPC = 0.0005;
	dataSet.w3NMPC = 0.0;

	//初始位置与艏向
	dataSet.nOrigin	= 0.0;
	dataSet.eOrigin	= 0.0;
	dataSet.psiOrigin	= 0.0;

	//目标位置与艏向
	dataSet.nTarget	= 100.0;
	dataSet.eTarget	= 100.0;
	dataSet.psiTarget	= 30.0;

	//环境最优动力定位半径
	dataSet.radius = 60.0;

	//环境最优艏向的三个参数
	dataSet.kpWOHC = 5e-6;
	dataSet.kiWOHC = 0.0;
	dataSet.kdWOHC = 0.0;

	//环境估计的三个参数
	dataSet.k1 = 0.8;
	dataSet.k2 = 2.0;
	dataSet.k3 = 2.0;

	for (int i = 0; i < DOF6; i ++)
	{
		thrustArray[i] = 0.0;
		wave1Array[i] = 0.0;
		wave2Array[i] = 0.0;
		windArray[i] = 0.0;
		curArray[i] = 0.0;
		taoArray[i] = 0.0;
	}

	//初始化data
	data = new Data;

	//设置控制器进行控制的步长
	ctrlCount = 0;
	ctrlCyc = 6;

	//虚拟圆心
	xCenter = 0.0;
	yCenter = 0.0;

	//初始化船舶速度
	Tool::initNu(nu);

	//初始化船舶的位置与姿态
	eta		= Tool::setEta(dataSet.nOrigin, dataSet.eOrigin, dataSet.psiOrigin);
	etaFlt	= Tool::setEta(dataSet.nOrigin, dataSet.eOrigin, dataSet.psiOrigin);

	//初始化作用力
	Tool::initForce6(thrust);
	Tool::initForce6(thrustZPCW);

	//初始化风作用力
	Tool::initForce6(windForce);

	//初始化一阶、二阶波浪力
	Tool::initForce6(wave1Force);
	Tool::initForce6(wave2Force);

	//初始化流作用力
	Tool::initForce6(curForce);
}

//设置船舶参数
void ShipControl::setData(Data *data_)
{
	data = data_;
}

//打开文件
void ShipControl::openFiles()
{
	paraFile.open("E:/projectProgram/data/parameters.txt");
	etaFile.open("E:/projectProgram/data/eta.txt");
	nuFile.open("E:/projectProgram/data/nu.txt");
	etaFltFile.open("E:/projectProgram/data/etaFilter.txt");
	centerFile.open("E:/projectProgram/data/center.txt");
	optHeadFile.open("E:/projectProgram/data/optHead.txt");
	targetFile.open("E:/projectProgram/data/target.txt");
	wave1File.open("E:/projectProgram/data/wave1.txt");
	wave2File.open("E:/projectProgram/data/wave2.txt");
	windFile.open("E:/projectProgram/data/wind.txt");
	curFile.open("E:/projectProgram/data/current.txt");
	taoFile.open("E:/projectProgram/data/tao.txt");
	thrustFile.open("E:/projectProgram/data/thrust.txt");
	envObsFile.open("E:/projectProgram/data/envObs.txt");
}

//关闭文件
void ShipControl::closeFiles()
{
	paraFile.close();
	etaFile.close();
	nuFile.close();
	etaFltFile.close();
	centerFile.close();
	optHeadFile.close();
	targetFile.close();
	wave1File.close();
	wave2File.close();
	windFile.close();
	curFile.close();
	taoFile.close();
	thrustFile.close();
	envObsFile.close();
}

//设置参数
void ShipControl::setParameter()
{
	//初始化环境最优艏向控制器的参数
	optPsiCtrl.setStep(tStep*ctrlCyc);
	optPsiCtrl.setPID(dataSet.kpWOHC, dataSet.kiWOHC, dataSet.kdWOHC);

	//初始化环境观测器
	envObs.setStep(tStep);
	envObs.setK(dataSet.k1, dataSet.k2, dataSet.k3);

	//设置船舶模型的参数
	model.setStep(tStep);
	model.setInitEta(eta);
	//设置船舶的参数
	model.setData(data);
	model.calM();

	//初始化环境的信息
	wind.setPara(dataSet.windSpeed, dataSet.windDir);
	
	wave.setData(data);
	wave.setPara(dataSet.waveHeight, dataSet.waveDir);
	wave.calWave();

	cur.setPara(dataSet.curSpeed, dataSet.curDir);

	//初始化PID
	pid.initPID(dataSet.kp, dataSet.ki, dataSet.kd);

	//设置NMPC中的周期与权值
	nmpc.setT(dataSet.tNMPC);
	nmpc.setWeight(dataSet.w1NMPC, dataSet.w2NMPC, dataSet.w3NMPC);
	nmpc.calM();


	filter.setStep(tStep);

	//初始化船舶的目标位置与姿态
	Tool::initEtaTarget(etaTarget, dataSet.nTarget, dataSet.eTarget, dataSet.psiTarget*angToRad);

	wopc.setStep(tStep*ctrlCyc);
	//设置环境最优动力定位的半径
	wopc.setRadius(dataSet.radius);

	//设置环境最优动力定位的目标位置
	wopc.setPos(etaTarget);

	//将初始化的数据发送到设置对话框
	emit sendDataSet(dataSet);
}

//输出位置姿态
Eta ShipControl::getEta()
{
	return eta;
}

//输出目标位置姿态
Eta ShipControl::getTarget()
{
	return etaTarget;
}

//开始进行船舶控制
void ShipControl::startRun()
{
	pauseState = false;
}

//暂停计算
void ShipControl::pauseRun()
{
	pauseState = true;
}

//停止计算
void ShipControl::ResetRun()
{
	//pauseState = false;

	//初始化船舶的位置与姿态
	eta		= Tool::setEta(dataSet.nOrigin, dataSet.eOrigin, dataSet.psiOrigin);
	etaFlt	= Tool::setEta(dataSet.nOrigin, dataSet.eOrigin, dataSet.psiOrigin);
	Tool::initForce6(thrust);
	model.setInitEta(eta);
}

//控制计算
void ShipControl::cal()
{
	//循环进行模型计算
	while (1)
	{
		//得到环境估计力
		envObs.setNu(nu);
		envObs.setTao(thrust);
		envObs.cal();		
		envEst = envObs.force();
		envObsFile << time << "\t" << envEst.xForce << "\t" << envEst.yForce << "\t" << envEst.nMoment << endl;

		//设置环境最优动力定位的当前位置艏向与速度角速度
		wopc.setEta(eta);
		wopc.setNu(nu);

		//风实时输入信息
		wind.setHead(eta.psi);
		wind.setNu(nu);
		//计算风的作用力
		wind.cal();

		windForce = wind.getWindTao();
		Tool::Force6ToArray(windForce, windArray);

		//波浪力计算
		wave.cal(eta, time);
		wave.getLoad(wave1Force, wave2Force);

		//Tool::initForce6(wave1Force);

		Tool::Force6ToArray(wave1Force, wave1Array);
		Tool::Force6ToArray(wave2Force, wave2Array);

		//流力计算
		cur.setPsi(eta.psi);
		cur.cal();
		curForce = cur.force();
		Tool::Force6ToArray(curForce, curArray);

		thrustZPCW.xForce = -(wave2Array[0] + windArray[0] + curArray[0]);
		thrustZPCW.yForce = -(wave2Array[1] + windArray[1] + curArray[1]);
		thrustZPCW.nMoment = -(wave2Array[5] + windArray[5] + curArray[5]);

		envEst.xForce = -thrustZPCW.xForce;
		envEst.yForce = -thrustZPCW.yForce;
		envEst.nMoment = -thrustZPCW.nMoment;

		qDebug() << "Thrust : \t" << thrustZPCW.xForce << "\t" << thrustZPCW.yForce << "\t" << thrustZPCW.nMoment << endl;

		if (0 == ctrlCount)
		{
			//动力定位控制
			switch (dataSet.ctrlType)
			{
				//PID控制
			case PID_CTRL:
				pid.setTarget(etaTarget);
				pid.setEta(etaFlt);
				pid.calculat();
				thrust = pid.getTao();
				break;
				//NMPC控制
			case NMPC_CTRL:
				nmpc.setTarget(etaTarget);
				nmpc.setEnv(envEst);
				nmpc.setEta(etaFlt);
				nmpc.setNu(nu);
				nmpc.cal();
				thrust = nmpc.Force();
				break;
			default:
				break;
			}

			//环境最优艏向控制
				switch (dataSet.dpMode)
				{
				case WOPC_DP:
					//wopc
					wopc.setThrust(thrustZPCW);
					wopc.calculat();
					xCenter = wopc.getCenterPos().first;
					yCenter = wopc.getCenterPos().second;
					etaTarget.n = wopc.getRTPosDes().first;
					etaTarget.e = wopc.getRTPosDes().second;
					etaTarget.psi = wopc.getPsiRTDes();
					optPsi = wopc.getPsiRTDes();
					break;
				case ZPCW_DP:
					//zpc-w
					optPsiCtrl.setPsi(eta.psi);
					optPsiCtrl.setTao(thrustZPCW);
					optPsiCtrl.cal();
					optPsi = optPsiCtrl.OptPsi();
					qDebug() << "zpcw optHead:\t" << optPsi*180/3.14 << "\t" << etaTarget.psi*180/3.14 << endl;
					etaTarget.psi = optPsi;
					break;
				case OPT_DP:
					//optPsi =  0.1*(-atan2(envEst.yForce, envEst.xForce) - 0.5*PI) + eta.psi;
					optPsi =  0.1*atan2(thrustZPCW.yForce, thrustZPCW.xForce) + eta.psi;
					
					//while (PI < optPsi)
					//{
					//	optPsi -= 2*PI;
					//} 
					//while (-PI > optPsi)
					//{
					//	optPsi += 2*PI;
					//}

					etaTarget.psi = optPsi;

					break;
				default:
					break;
				}

		}
		ctrlCount ++;
		if (ctrlCyc <= ctrlCount)
		{
			ctrlCount = 0;
		}

		targetFile << time << "\t" << etaTarget.n << "\t"
			<< etaTarget.e << "\t" << etaTarget.psi << endl;

		//环境最优动力定位保存数据
		if (WOPC_DP == dataSet.dpMode || 
			ZPCW_DP == dataSet.dpMode || 
			OPT_DP == dataSet.dpMode)
		{
			optHeadFile << time << "\t" << optPsi << "\n";
			if (WOPC_DP == dataSet.dpMode)
			{
				centerFile << time << "\t" << xCenter
					<< "\t" << yCenter << "\n";
			}
		}

		//推力结构体转换为数组
		Tool::Force6ToArray(thrust, thrustArray);

		//在文件中分别记录各种力
		taoFile << time << "\t";
		thrustFile << time << "\t";
		windFile << time << "\t";
		wave1File << time << "\t";
		wave2File << time << "\t";
		curFile << time << "\t";
		for (int i = 0; i < DOF6; ++ i)
		{
			//计算合力
			taoArray[i] = thrustArray[i] + wave1Array[i] + wave2Array[i] + windArray[i] + curArray[i];

			taoFile << taoArray[i] << "\t";
			thrustFile << thrustArray[i] << "\t";
			windFile << windArray[i] << "\t";
			wave1File << wave1Array[i] << "\t";
			wave2File << wave2Array[i] << "\t";
			curFile << curArray[i] << "\t";
		}
		taoFile << "\n";
		thrustFile << "\n";
		windFile << "\n";
		wave1File << "\n";
		wave2File << "\n";
		curFile << "\n";

		Tool::ArrayToForce6(taoArray, tao);

		//程序暂停运行
		while (pauseState) 
		{
			msleep(100);
		}

		//合外力作用到船舶模型上
		model.setForce(tao);

		//解算船舶模型
		model.cal();

		//计算后得到船舶的位置姿态以及速度角速度
		eta = model.getEta();
		nu = model.getNu();		

		//滤波
		filter.setTao(thrust);
		filter.setEta(eta);
		etaFlt = filter.cal();

		etaFile << time << "\t" << eta << "\n";

		nuFile << time << "\t" << nu.u << "\t" << nu.v << "\t" << nu.w 
			<< "\t" << nu.p << "\t" << nu.q << "\t" << nu.r << "\n";

		etaFltFile << time << "\t" << etaFlt << "\n";

		time += tStep;	
	}
}

//从设置对话框接收数据
void ShipControl::receivDataSet(DataSetStruct dataReceive)
{
	dataSet = dataReceive;
	setParameter();
}

//船舶控制运行
void ShipControl::run()
{	
	time = 0.0;
	//船舶开始进行控制计算
	cal();
}

//重载<<操作符
ostream& operator << (ostream &os, const Eta &eta)
{
	os << eta.n << "\t" << eta.e << "\t" << eta.d << "\t"
		<< eta.phi << "\t" << eta.theta << "\t" << eta.psi;
	return os;
}
