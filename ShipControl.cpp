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
	//允许程序进行计算
	runEnable = true;

	//打开文件
	openFiles();

	//仿真总时间
	maxTime = 1000.0;

	//设定风速，风向
	SpeedWind	= 20.0;
	DirWind		= 90.0;

	//设定浪高，浪向
	HeightWave	= 5.0;
	DirWave		= 150.0;

	//设定流速，流向
	SpeedCurrent	= 2.0;
	DirCurrent		= 120.0;

	//初始位置与艏向
	xOrigin		= 0.0;
	yOrigin		= 0.0;
	psiOrigin	= 0.0;

	//目标位置与艏向
	xTarget		= 100.0;
	yTarget		= 100.0;
	psiTarget	= 130.0;

	//初始化环境最优艏向
	optPsi = 0.0;

	//初始化PID参数
	kp = 0.15;
	ki = 0.0;
	kd = 0.0;

	//初始化NMPC的预测周期
	Tpre = 9.0;
	//初始化NMPC的三个权值
	w1 = 0.9;
	w2 = 0.0005;
	w3 = 0.0;

	//ZPC-W的三个参数
	kpZ = 1e-8;
	kiZ = 0.0;
	kdZ = 0.0;

	//环境估计的三个参数
	k1Env = 0.8;
	k2Env = 2.0;
	k3Env = 2.0;


	for (int i = 0; i < DOF6; i ++)
	{
		thrustArray[i] = 0.0;
		wave1Array[i] = 0.0;
		wave2Array[i] = 0.0;
		windArray[i] = 0.0;
		curArray[i] = 0.0;
		taoArray[i] = 0.0;
	}

	data = new Data;

	srand(int(time));

	ctrlCount = 0;
	ctrlCyc = 6;

	//环境最优动力定位半径
	radius = 60.0;

	//初始化船舶速度
	Tool::initNu(nu);

	//初始化船舶的位置与姿态
	eta = Tool::setEta(xOrigin, yOrigin, psiOrigin);
	etaFlt = Tool::setEta(xOrigin, yOrigin, psiOrigin);

	//初始化作用力
	Tool::initForce6(thrust);

	//初始化风作用力
	Tool::initForce6(windForce);

	//初始化一阶、二阶波浪力
	Tool::initForce6(wave1Force);
	Tool::initForce6(wave2Force);

	//初始化流作用力
	Tool::initForce6(curForce);

	//初始化动力定位任务类型：1.常规动力定位；2.环境最优动力定位
	dpFlag = 1;
	//初始化动力定位控制方法：1.PID控制；2.非线性模型预测控制
	ctlFlag = 1;
	//初始化环境最优动力定位控制策略类型：
	//1.WOPC与ZPC-W结合后的环境最优动力定位控制策略
	//2.WOPC借用环境估计的环境最优动力定位控制策略
	//3.ZPC-W环境最优动力定位控制策略
	wopcFlag = 1;

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
	outFltFile.open("E:/projectProgram/data/etaFilter.txt");
	centerFile.open("E:/projectProgram/data/center.txt");
	optHeadFile.open("E:/projectProgram/data/optHead.txt");
	targetFile.open("E:/projectProgram/data/target.txt");
	wave1File.open("E:/projectProgram/data/wave1.txt");
	wave2File.open("E:/projectProgram/data/wave2.txt");
	windFile.open("E:/projectProgram/data/wind.txt");
	curFile.open("E:/projectProgram/data/current.txt");
	taoFile.open("E:/projectProgram/data/tao.txt");
	thrustFile.open("E:/projectProgram/data/thrust.txt");
}

//关闭文件
void ShipControl::closeFiles()
{
	paraFile.close();
	etaFile.close();
	nuFile.close();
	outFltFile.close();
	centerFile.close();
	optHeadFile.close();
	targetFile.close();
	wave1File.close();
	wave2File.close();
	windFile.close();
	curFile.close();
	taoFile.close();
	thrustFile.close();
}

//设置参数
void ShipControl::setParameter()
{

	//初始化环境最优艏向控制器的参数
	optPsiCtrl.setStep(tStep*ctrlCyc);
	optPsiCtrl.setPID(kpZ, kiZ, kdZ);

	//初始化环境观测器
	envObs.setStep(tStep);
	envObs.setK(k1Env, k2Env, k3Env);

	//设置船舶模型的参数
	model.setStep(tStep);
	model.setInitEta(eta);
	//设置船舶的参数
	model.setData(data);
	model.calM();

	//初始化环境的信息
	wind.setPara(SpeedWind, DirWind);
	
	wave.setData(data);
	wave.setPara(HeightWave, DirWave);
	wave.calWave();

	cur.setPara(SpeedCurrent, DirCurrent);

	//初始化PID
	pid.initPID(kp, ki, kd);

	//设置NMPC中的周期与权值
	nmpc.setT(Tpre);
	nmpc.setWeight(w1, w2, w3);
	nmpc.calM();


	filter.setStep(tStep);

	//初始化船舶的目标位置与姿态
	Tool::initEtaTarget(etaTarget, xTarget, yTarget, psiTarget*angToRad);

	wopc.setStep(tStep*ctrlCyc);
	//设置环境最优动力定位的半径
	wopc.setRadius(radius);

	//设置环境最优动力定位的目标位置
	wopc.setPos(etaTarget);
}

//输出位置姿态
Eta ShipControl::getEta()
{
	return eta;
}

//开始进行船舶控制
void ShipControl::startRun()
{

}

//停止计算
void ShipControl::stopRun()
{
	runEnable = false;
}

//控制计算
void ShipControl::cal()
{

}

//船舶控制运行
void ShipControl::run()
{	
	runEnable = true;

	time = 0.0;
	//循环进行模型计算
	while (runEnable)
	{
		//得到环境估计力
		envObs.setNu(nu);
		envObs.setTao(thrust);
		envObs.cal();		
		envEst = envObs.force();

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

		Tool::Force6ToArray(wave1Force, wave1Array);
		Tool::Force6ToArray(wave2Force, wave2Array);

		//流力计算
		cur.setPsi(eta.psi);
		cur.cal();
		curForce = cur.force();
		Tool::Force6ToArray(curForce, curArray);

		if (0 == ctrlCount)
		{
			//动力定位控制
			switch (ctlFlag)
			{
				//PID控制
			case 1:
				pid.setTarget(etaTarget);
				pid.setEta(eta);
				pid.calculat();
				thrust = pid.getTao();
				break;
				//NMPC控制
			case 2:
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
			if (2 == dpFlag)
			{
				switch (wopcFlag)
				{
				case 1:
					//wopc
					wopc.setThrust(thrust);
					wopc.calculat();
					etaTarget.n = wopc.getRTPosDes().first;
					etaTarget.e = wopc.getRTPosDes().second;
					etaTarget.psi = wopc.getPsiRTDes();
					optPsi = wopc.getPsiRTDes();
					break;
				case 2:
					break;
				case 3:
					//zpc-w
					optPsiCtrl.setPsi(optPsi);
					optPsiCtrl.setTao(thrust);
					optPsiCtrl.cal();
					optPsi = optPsiCtrl.OptPsi();
					break;
				default:
					break;
				}
			}

		}
		ctrlCount ++;
		if (ctrlCyc <= ctrlCount)
		{
			ctrlCount = 0;
		}

		//环境最优动力定位保存数据
		if (2 == dpFlag)
		{
            optHeadFile << time << "\t" << optPsi << "\n";
			if (1 == wopcFlag || 2 == wopcFlag)
			{
				centerFile << time << "\t" << etaTarget.n
                    << "\t" << etaTarget.e << "\n";
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

        outFltFile << time << "\t" << etaFlt << "\n";

		time += tStep;	
	}

}

//重载<<操作符
ostream& operator << (ostream &os, const Eta &eta)
{
	os << eta.n << "\t" << eta.e << "\t" << eta.d << "\t"
		<< eta.phi << "\t" << eta.theta << "\t" << eta.psi;
	return os;
}

