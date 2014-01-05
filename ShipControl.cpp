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

	//用户进行初始化
    userInterFace();

	//初始化环境的信息
	wind.setPara(SpeedWind, DirWind);
	cur.setPara(SpeedCurrent, DirCurrent);
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

	//仿真总时间
	maxTime = 1000.0;

	//设定风速，风向
	SpeedWind	= 10.0;
	DirWind		= 90.0;

	//设定浪高，浪向
	HeightWave	= 2.0;
	DirWave		= 150.0;

	//设定流速，流向
	SpeedCurrent	= 1.0;
	DirCurrent		= 120.0;

	//初始位置与艏向
	xOrigin		= 0.0;
	yOrigin		= 0.0;
	psiOrigin	= 0.0;

	//目标位置与艏向
	xTarget		= 100.0;
	yTarget		= 100.0;
	psiTarget	= 30.0;

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

//用户接口
void ShipControl::userInterFace()
{
//    QTextStream out(stdout);

//    out << "※※动力定位控制仿真※※\n\n\n";

//    out << "★请设置环境参数：\n";

//    out << "\t风速(默认为" << SpeedWind << "m/s) = ";

//	gets_s(SpeedWindStr, 20);
//	valueStr = strtod(SpeedWindStr, &SpeedWindStrEnd);
//	SpeedWind = (SpeedWindStrEnd == SpeedWindStr) ? SpeedWind : valueStr;

//    out << "\t风向(默认为" << DirWind << "°) = ";

//	gets_s(DirWindStr, 20);
//	valueStr = strtod(DirWindStr, &DirWindStrEnd);
//	DirWind = (DirWindStrEnd == DirWindStr) ? DirWind : valueStr;

//    out << "\t浪高(默认为" << HeightWave << "m) = ";

//	gets_s(HeightWaveStr, 20);
//	valueStr = strtod(HeightWaveStr, &HeightWaveStrEnd);
//	HeightWave = (HeightWaveStrEnd == HeightWaveStr) ? HeightWave : valueStr;

//    out << "\t浪向(默认为" << DirWave << "°) = ";

//	gets_s(DirWaveStr, 20);
//	valueStr = strtod(DirWaveStr, &DirWaveStrEnd);
//	DirWave = (DirWaveStrEnd == DirWaveStr) ? DirWave : valueStr;

//    out << "\t流速(默认为" << SpeedCurrent << "m/s) = ";

//	gets_s(SpeedCurrentStr, 20);
//	valueStr = strtod(SpeedCurrentStr, &SpeedCurrentStrEnd);
//	SpeedCurrent = (SpeedCurrentStrEnd == SpeedCurrentStr) ? SpeedCurrent : valueStr;

//    out << "\t流向(默认为" << DirCurrent << "°) = ";

//	gets_s(DirCurrentStr, 20);
//	valueStr = strtod(DirCurrentStr, &DirCurrentStrEnd);
//	DirCurrent = (DirCurrentStrEnd == DirCurrentStr) ? DirCurrent : valueStr;

//    out << "\n★请设置船舶在动力定位过程中的初始位置艏向与目标位置艏向：" << "\n";

//    out << "\t初始北位置(默认为" << xOrigin << "m) = ";

//	gets_s(xOriginStr, 20);
//	valueStr = strtod(xOriginStr, &xOriginStrEnd);
//	xOrigin = (xOriginStrEnd == xOriginStr) ? xOrigin : valueStr;

//    out << "\t初始东位置(默认为" << yOrigin << "m) = ";

//	gets_s(yOriginStr, 20);
//	valueStr = strtod(yOriginStr, &yOriginStrEnd);
//	yOrigin = (yOriginStrEnd == yOriginStr) ? yOrigin : valueStr;

//    out << "\t初始艏向(默认为" << psiOrigin << "°) = ";

//	gets_s(PsiOriginStr, 20);
//	valueStr = strtod(PsiOriginStr, &PsiOriginStrEnd);
//	psiOrigin = (PsiOriginStrEnd == PsiOriginStr) ? psiOrigin : valueStr;

//    out << "\n";

//    out << "\t目标北位置(默认为" << xTarget << "m) = ";

//	gets_s(xTargStr, 20);
//	valueStr = strtod(xTargStr, &xTargStrEnd);
//	xTarget = (xTargStrEnd == xTargStr) ? xTarget : valueStr;

//    out << "\t目标东位置(默认为" << yTarget << "m) = ";

//	gets_s(yTargStr, 20);
//	valueStr = strtod(yTargStr, &yTargStrEnd);
//	yTarget = (yTargStrEnd == yTargStr) ? yTarget : valueStr;

//    out << "\t目标艏向(默认为" << psiTarget << "°) = ";

//	gets_s(PsiTargStr, 20);
//	valueStr = strtod(PsiTargStr, &PsiTargStrEnd);
//	psiTarget = (PsiTargStrEnd == PsiTargStr) ? psiTarget : valueStr;

//    out << "\n★请选择动力定位任务类型(1.常规动力定位；2.环境最优动力定位)：" << "\n";

//	do
//	{
//        out << "\t默认任务类型为 1.常规动力定位。\n\t请输入任务类型代码： ";

//		cin.get(dpFlagStr, 20);
//		cin.get();

//		dpFlag = ( 0 != atoi(dpFlagStr))? atoi(dpFlagStr) : 1;

//		if (1 != dpFlag && 2 != dpFlag)
//		{
//            out << "\t¤¤输入有误，请确认后再次输入！" << "\n";
//		}
//	} while (1 != dpFlag && 2 != dpFlag);

//    out << "\n★请选择动力定位控制器类型(1.PID控制器；2.非线性模型预测控制器)：" << "\n";

//	do
//	{
//        out << "\t默认控制器类型为 1.PID控制器。\n\t请输入控制器类型代码： ";

//		cin.get(ctlFlagStr, 20);
//		cin.get();

//		ctlFlag = ( 0 != atoi(ctlFlagStr))? atoi(ctlFlagStr) : 1;

//		if (1 != ctlFlag && 2 != ctlFlag)
//		{
//            out << "\t¤¤输入有误，请确认后再次输入！" << "\n";
//		}
//	} while (1 != ctlFlag && 2 != ctlFlag);

//	//设置DP控制器参数
//	switch (ctlFlag)
//	{
//	case 1:
//        out << "\n★请输入PID控制器的参数" << "\n";

//        out << "\tKp(默认为" << kp << ") = ";

//		gets_s(kpStr, 20);
//		valueStr = strtod(kpStr, &kpStrEnd);
//		kp = (kpStrEnd == kpStr) ? kp : valueStr;

//        out << "\n\tKi(默认为" << ki << ") = ";

//		gets_s(kiStr, 20);
//		valueStr = strtod(kiStr, &kiStrEnd);
//		ki = (kiStrEnd == kiStr) ? ki : valueStr;

//        out << "n\tKd(默认为" << kd << ") = ";

//		gets_s(kdStr, 20);
//		valueStr = strtod(kdStr, &kdStrEnd);
//		kd = (kdStrEnd == kdStr) ? kd : valueStr;

//		break;
//	case 2:
//        out << "\n★请输入NMPC控制器的参数" << "\n";

//        out << "\t预测周期T(默认为" << Tpre << ") = ";

//		gets_s(tStr, 20);
//		valueStr = strtod(tStr, &tStrEnd);
//		Tpre = (tStrEnd == tStr) ? Tpre : valueStr;

//        out << "\t权值w1(默认为" << w1 << ") = ";

//		gets_s(w1Str, 20);
//		valueStr = strtod(w1Str, &w1StrEnd);
//		w1 = (w1StrEnd == w1Str) ? w1 : valueStr;

//        out << "\t权值w2(默认为" << w2 << ") = ";

//		gets_s(w2Str, 20);
//		valueStr = strtod(w2Str, &w2StrEnd);
//		w2 = (w2StrEnd == w2Str) ? w2 : valueStr;

//        out << "\t权值w3(默认为" << w3 << ") = ";

//		gets_s(w3Str, 20);
//		valueStr = strtod(w3Str, &w3StrEnd);
//		w3 = (w3StrEnd == w3Str) ? w3 : valueStr;

//		break;
//	default:
//		break;
//	}

//	//设置环境最优艏向控制的参数
//	if (2 == dpFlag)
//	{
//    out << "\n★请选择环境最优动力定位控制策略\n"
//		<< "\t(1.WOPC与ZPC-W结合后的环境最优动力定位控制策略;\n"
//		<< "\t2.WOPC借用环境估计的环境最优动力定位控制策略;\n"
//        << "\t3.ZPC-W环境最优动力定位控制策略)：" << "\n";

//	do
//	{
//        out << "\t默认控制策略为 1.WOPC与ZPC-W结合后的环境最优动力定位控制策略。\n"
//			<< "\t请输入控制策略代码： ";

//		cin.get(wopcFlagStr, 20);
//		cin.get();

//		wopcFlag = ( 0 != atoi(wopcFlagStr))? atoi(wopcFlagStr) : 1;

//		if (1 != wopcFlag && 2 != wopcFlag && 3 != wopcFlag)
//		{
//            out << "\t¤¤输入有误，请确认后再次输入！" << "\n";
//		}
//	} while (1 != wopcFlag && 2 != wopcFlag && 3 != wopcFlag);

//		switch (wopcFlag)
//		{
//		case 1:
//			break;
//		case 2:
//			break;
//		case 3:
//            out << "\n★请输入ZPC-W环境最优艏向控制器的参数" << "\n";

//            out << "\tKpZ(默认为" << kpZ << ") = ";

//			gets_s(kpZStr, 20);
//			valueStr = strtod(kpZStr, &kpZStrEnd);
//			kpZ = (kpZStrEnd == kpZStr) ? kpZ : valueStr;

//            out << "\tKiZ(默认为" << kiZ << ") = ";

//			gets_s(kiZStr, 20);
//			valueStr = strtod(kiZStr, &kiZStrEnd);
//			kiZ = (kiZStrEnd == kiZStr) ? kiZ : valueStr;

//            out << "\tKdZ(默认为" << kdZ << ") = ";

//			gets_s(kdZStr, 20);
//			valueStr = strtod(kdZStr, &kdZStrEnd);
//			kdZ = (kdZStrEnd == kdZStr) ? kdZ : valueStr;

//			break;
//		default:
//			break;
//		}
//	}


//    out << "\n正在进行计算，请稍等..." << "\n";

//	paraFile << "※※船舶动力定位过程中各参数信息※※\n";
//	paraFile << "船舶初始位置与艏向为： (" << xOrigin << "m, " << yOrigin << "m, " << psiOrigin << "°).\n";
//	paraFile << "船舶目标位置与艏向为： (" << xTarget << "m, " << yTarget << "m, " << psiTarget << "°).\n";

//	paraFile << "★环境参数★\n";
//	paraFile << "风速为： " << SpeedWind << "m/s, " << "风向为： " << DirWind << "°.\n";
//	paraFile << "浪高为： " << HeightWave << "m, " << "浪向为： " << DirWave << "°.\n";
//	paraFile << "流速为： " << SpeedCurrent << "m/s, " << "流向为： " << DirCurrent << "°.\n";

//	paraFile << "★动力定位任务类型★\n";
//	switch (dpFlag)
//	{
//	case 1:
//		paraFile << "常规动力定位\n";
//		break;
//	case 2:
//		paraFile << "环境最优动力定位\n";
//		break;
//	default:
//		break;
//	}

//    paraFile << "★动力定位控制器类型★" << "\n";
//	switch (dpFlag)
//	{
//	case 1:
//        paraFile << "\tPID控制器" << "\n" << "\n";
//		paraFile << "★PID控制器参数★\n";
//        paraFile << "\tKp = " << kp << "\n";
//        paraFile << "\tKi = " << ki << "\n";
//        paraFile << "\tKd = " << kd << "\n";
//        paraFile << "\n";
//		break;
//	case 2:
//        paraFile << "\t非线性模型预测控制器" << "\n" << "\n";
//		paraFile << "★非线性模型预测控制器参数★\n";
//        paraFile << "\tT = " << Tpre << "\n";
//        paraFile << "\tw1 = " << w1 << "\n";
//        paraFile << "\tw2 = " << w2 << "\n";
//        paraFile << "\tw3 = " << w3 << "\n";
//        paraFile << "\n";
//		break;
//	default:
//		break;
//	}
	
//	if (2 == dpFlag)
//	{
//		switch (wopcFlag)
//		{
//		case 1:
//            paraFile << "★ZPC-W环境最优艏向控制器的参数★" << "\n";
//            paraFile << "\tKpZ = " << kpZ << "\n";
//            paraFile << "\tKiZ = " << kiZ << "\n";
//            paraFile << "\tKdZ = " << kdZ << "\n";
//            paraFile << "\n";
//			break;
//		case 2:
//			break;
//		case 3:
//			break;
//		default:
//			break;
//		}

//	}
//    targetFile << xTarget << "\t" << yTarget << "\n";

	setParameter();
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

//船舶控制运行
void ShipControl::run()
{	
	//设置船舶的参数
	model.setData(data);
	model.calM();

	//初始化波浪的信息
	wave.setData(data);
	wave.setPara(HeightWave, DirWave);
	wave.calWave();

	//循环进行模型计算
	for (time = 0.0; time <= maxTime; time += tStep)
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
	}

}

//重载<<操作符
ostream& operator << (ostream &os, const Eta &eta)
{
	os << eta.n << "\t" << eta.e << "\t" << eta.d << "\t"
		<< eta.phi << "\t" << eta.theta << "\t" << eta.psi;
	return os;
}

