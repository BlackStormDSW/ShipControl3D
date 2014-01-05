/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//DataStruct.h

#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

#include <string>
using namespace std;

#define DOF3 3
#define DOF6 6
#define wNum 36
#define velTotal 5
#define visDampNum 5
#define PI	3.141592653589793
#define E	2.718281828459046

//速度的米每秒转换到节
#define M2KNOT 0.5144

//角度转弧度系数
#define angToRad (PI/180)
//弧度转角度系数
#define radToAng (180/PI)
//重力加速度
#define gravity 9.81
//vesselABC中freqs数
#define freqNum 1146
//Hoerner数据的个数
#define  HoerNum 20

#pragma pack(1)

//力、转矩结构体
struct Force3
{
	double xForce;		//纵向力
	double yForce;		//横向力
	double nMoment;		//转矩
};

struct Force6
{
	double xForce;		//纵向力
	double yForce;		//横向力
	double zForce;		//垂向力
	double kMoment;		//转矩
	double mMoment;		//转矩
	double nMoment;		//转矩
};

//位置和姿态结构体
struct Eta
{
	double n;			//北位置
	double e;			//东位置
	double d;			//
	double phi;			//横摇角 弧度
	double theta;		//纵摇角 弧度
	double psi;			//艏向角 弧度
};

//线速度和角速度结构体
struct Nu
{
	double u;			//纵向速度
	double v;			//横向速度
	double w;			//升沉速度
	double p;			//横摇角速度
	double q;			//纵摇角速度
	double r;			//艏摇角速度
};

struct Coord
{
	double x;
	double y;
	double z;
};

struct VesselMain
{
    std::string shipName;
	double Lpp;
	double T;
	double B;
	double rho;
	double m;
	double nabla;
	double GM_L;
	double GM_T;
	double C_B;
	double Lwl;
	double S;
	Coord CG;
	Coord CB;
};

struct ForceRAO
{
	double amp[DOF6][velTotal][wNum][wNum];
	double phase[DOF6][velTotal][wNum][wNum];
	double w[wNum];
};

struct Driftfrc
{
	double amp[DOF6][velTotal][wNum][wNum];
	double w[wNum];
};

//vessel内的数据
struct VesselData
{
	VesselMain main;
	double MRB[DOF6][DOF6];
	ForceRAO forceRAO;
	Driftfrc driftfrc;
	double vel[velTotal];
};

//vesselABC内的数据
struct VesselABCData
{
	double Ainf[DOF6][DOF6];
	double Binf[DOF6][DOF6];
	double Ar[DOF6][DOF6][velTotal][velTotal];
	double Br[DOF6][DOF6][velTotal];
	double Cr[DOF6][DOF6][velTotal];
	double Dr[DOF6][DOF6];
	double B44_inf[velTotal];
	double A44[velTotal][velTotal][velTotal];
	double B44[velTotal][velTotal];
	double C44[velTotal][velTotal];
	double D44[velTotal];
	double MA[DOF6][DOF6];
	double G[DOF6][DOF6];
	double Minv[DOF6][DOF6];
	Coord r_g;
	double A[freqNum][DOF6][DOF6];
	double B[freqNum][DOF6][DOF6];
	double freqs[freqNum];

};

struct Data
{
	VesselData dataVes;
	VesselABCData dataVesABC;
	double Mmx[DOF6/2][DOF6/2];
	double Dmx[DOF6/2][DOF6/2];
	double CDdata[2][HoerNum];
};

struct xStatOut
{
	double x1;
	double x2;
	double x3;
	double x4;
	double x5;
};

#pragma pack()

#endif//DATASTRUCT_H_
