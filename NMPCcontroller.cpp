/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2013-12-17							**
******************************************************************/

//NMPCcontroller.cpp

#include "NMPCcontroller.h"
#include <math.h>
#include <iostream>
using namespace std;

NMPCcontroller::NMPCcontroller(void)
{
	init();
}

NMPCcontroller::~NMPCcontroller(void)
{
	outFile.close();
}

//参数初始化
void NMPCcontroller::init()
{
	//初始化预测时间长度T与权值
	T = 9.0;

	lambda[0] = 20.0;		//>=0
	lambda[1] = 20.0;			//>0
	lambda[2] = 0.00000001;			//>=0

	//初始化推进器推力与力矩的限值
	lmtForce[0] = MAXSURGE;
	lmtForce[1] = MAXSWAY;
	lmtForce[2] = MAXYAW;

	//初始化推进器的推力与力矩
	force.xForce = 0.0;
	force.yForce = 0.0;
	force.zForce = 0.0;
	force.kMoment = 0.0;
	force.mMoment = 0.0;
	force.nMoment = 0.0;

	//初始化船舶系统惯性矩阵、阻尼矩阵等
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			m[i][j] = 0.0;
			d[i][j] = 0.0;
			a[i][j] = 0.0;

			B[i][j] = 0.0;
			C[i][j] = 0.0;

			M3[i][j] = 0.0;
		}
		b[i] = 0.0;
		U[i] = 0.0;
	}

	//系统惯性矩阵(原6自由度系统惯性矩阵简化而来)
	//m[0][0] = 0.0026e+10;
	//m[1][1] = 0.0033e+10;
	//m[2][2] = 6.5209e+10;

	m[0][0] = 5312200.0*0.5;
	m[1][1] = 828310.0*2.0;
	m[2][2] = 37454000000.0;

	//阻尼矩阵(原6自由度阻尼矩阵简化而来)
	d[0][0] = 0.0002e+8;
	d[1][1] = 0.0022e+8;
	d[2][2] = 7.1506e+8;

	//计算过程中的中间系数a
	a[0][0] = -d[0][0]/m[0][0];
	a[1][1] = -(m[2][2]*d[1][1]-m[1][2]*d[2][1])/(m[1][1]*m[2][2]-m[1][2]*m[2][1]);
	a[1][2] = -(m[2][2]*d[1][2]-m[1][2]*d[2][2])/(m[1][1]*m[2][2]-m[1][2]*m[2][1]);
	a[2][1] = -(m[1][1]*d[2][1]-m[2][1]*d[1][1])/(m[1][1]*m[2][2]-m[1][2]*m[2][1]);
	a[2][2] = -(m[1][1]*d[2][2]-m[2][1]*d[1][2])/(m[1][1]*m[2][2]-m[1][2]*m[2][1]);

	//计算过程中的中间系数b
	b[0] = 1.0/m[0][0];
	b[1] = 1.0/m[1][1];
	b[2] = 1.0/m[2][2];
	
	//初始化计算过程中的矩阵变量
	for (int i = 0; i < MAXDIM; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			M[i][j] = 0.0;
		}
		for (int j = 0; j < DOF3; j ++)
		{
			q1[i][j] = 0.0;
			q1T[j][i] = 0.0;
		}
		F[i] = 0.0;
		q0[i] = 0.0;
		yd[i] = 0.0;
	}

	outFile.open("E:/projectProgram/data/nmpcTest.txt");
}

//计算M矩阵
void NMPCcontroller::calM()
{
	//计算M3和M矩阵
	for (int i = 0; i < DOF3; i ++)
	{
		M3[i][i] = lambda[2]*T;
	}

	M[0][0] = lambda[0] + lambda[1]*T;
	M[0][1] = lambda[0]*T + lambda[1]*pow(T,2.0)/2.0;
	M[0][2] = lambda[0]*pow(T,2.0)/2.0 + lambda[1]*pow(T,3.0)/6.0;
	M[0][3] = lambda[0]*pow(T,3.0)/6.0 + lambda[1]*pow(T,4.0)/24.0;

	M[1][0] = M[0][1];
	M[1][1] = lambda[0]*pow(T,2.0) + lambda[1]*pow(T,3.0)/3.0;
	M[1][2] = lambda[0]*pow(T,3.0)/2.0 + lambda[1]*pow(T,4.0)/8.0;
	M[1][3] = lambda[0]*pow(T,4.0)/6.0 + lambda[1]*pow(T,5.0)/30.0;

	M[2][0] = M[0][2];
	M[2][1] = M[1][2];
	M[2][2] = lambda[0]*pow(T,4.0)/4.0 + lambda[1]*pow(T,5.0)/20.0;
	M[2][3] = lambda[0]*pow(T,5.0)/12.0 + lambda[1]*pow(T,6.0)/72.0;

	M[3][0] = M[0][3];
	M[3][1] = M[1][3];
	M[3][2] = M[2][3];
	M[3][3] = lambda[0]*pow(T,6)/36.0 + lambda[1]*pow(T,7.0)/252.0;

	//计算矩阵变量中的各元素
	for (int i = 0; i < MAXDIM/3; i ++)
	{
		for (int j = 0; j < MAXDIM/3; j ++)
		{
			M[i+4][j+4] = M[i][j];
			M[i+8][j+8] = M[i][j];
		}
	}
}

//控制器计算
void NMPCcontroller::cal()
{
	//每次计算都需要初始化的矩阵变量
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			A[i][j] = 0.0;
			D[i][j] = 0.0;
		}
	}
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			B[j][i] = 0.0;
		}
		U[i] = 0.0;
	}

	//计算q0矩阵12x1
	q0[0]=x;
	q0[1]=u*cos(psi)-v*sin(psi);
	q0[2]=-u*r*sin(psi)-v*r*cos(psi)+a[0][0]*u*cos(psi)-a[1][1]*v*sin(psi)-a[1][2]*r*sin(psi);
	q0[3]=(-u*r*cos(psi)+v*r*sin(psi)-a[0][0]*u*sin(psi)-a[1][1]*v*cos(psi)-a[1][2]*r*cos(psi))*r
		+(-r*sin(psi)+a[0][0]*cos(psi))*(a[0][0]*u)
		+(-r*cos(psi)-a[1][1]*sin(psi))*(a[1][1]*v+a[1][2]*r)
		+(-u*sin(psi)-v*cos(psi)-a[1][2]*sin(psi))*(a[2][1]*v+a[2][2]*r);

	q0[4]=y;
	q0[5]=u*sin(psi)+v*cos(psi);
	q0[6]=u*r*cos(psi)-v*r*sin(psi)+a[0][0]*u*sin(psi)+a[1][1]*v*cos(psi)+a[1][2]*r*cos(psi);
	q0[7]=(-u*r*sin(psi)-v*r*cos(psi)+a[0][0]*u*cos(psi)-a[1][1]*v*sin(psi)-a[1][2]*r*sin(psi))*(r)
		+(r*cos(psi)+a[0][0]*sin(psi))*(a[0][0]*u)
		+(-r*sin(psi)+a[1][1]*cos(psi))*(a[1][1]*v+a[1][2]*r)
		+(u*cos(psi)-v*sin(psi)+a[1][2]*cos(psi))*(a[2][1]*v+a[2][2]*r);

	q0[8]=psi;
	q0[9]=r;
	q0[10]=a[2][1]*v+a[2][2]*r;
	q0[11]=a[2][1]*a[1][1]*v+a[2][1]*a[1][2]*r+a[2][2]*a[2][1]*v+a[2][2]*a[2][2]*r;

	//计算q1矩阵12x3
	q1[2][0]=b[0]*cos(psi);
	q1[2][1]=-b[1]*sin(psi);
	q1[2][2]=0;

	q1[3][0]=b[0]*(-2*r*sin(psi)+a[0][0]*cos(psi));
	q1[3][1]=b[1]*(-2*r*cos(psi)-a[1][1]*sin(psi));
	q1[3][2]=b[2]*(-u*sin(psi)-v*cos(psi)-a[1][2]*sin(psi));


	q1[6][0]=b[0]*sin(psi);
	q1[6][1]=b[1]*cos(psi);
	q1[6][2]=0;

	q1[7][0]=b[0]*(2*r*cos(psi)+a[0][0]*sin(psi));
	q1[7][1]=b[1]*(-2*r*sin(psi)+a[1][1]*cos(psi));
	q1[7][2]=b[2]*(u*cos(psi)-v*sin(psi)+a[1][2]*cos(psi));

	q1[10][0]=0;
	q1[10][1]=0;
	q1[10][2]=b[2];

	q1[11][0]=0;
	q1[11][1]=a[2][1]*b[1];
	q1[11][2]=a[2][2]*b[2];
	
	//计算q1T矩阵12x3
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			q1T[i][j] = q1[j][i];
		}
	}

	//计算A=q1T×M矩阵3x12
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			for (int p = 0; p < MAXDIM; p ++)
			{
				A[i][j] += q1T[i][p] * M[p][j];
			}
		}
	}

	//计算B=q1T×M×q1=A×q1矩阵3x3
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			for (int p = 0; p < MAXDIM; p ++)
			{
				B[i][j] += A[i][p] * q1[p][j];
			}
		}
	}

	//计算C=B+M3矩阵3x3
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < DOF3; j ++)
		{
			C[i][j] = B[i][j] + M3[i][j];
		}
	}

	//计算矩阵C的逆
	inv(C, invC);
	
	//计算D=invC×A矩阵3x12
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			for (int p = 0; p < DOF3; p ++)
			{
				D[i][j] += invC[i][p] * A[p][j];
			}
			outFile << D[i][j] << "\t";
		}
	}
	outFile << endl;

	//计算F=q0-yd矩阵12x1
	for (int i = 0; i < MAXDIM; i ++)
	{
		F[i] = q0[i] - yd[i];
	}

	//计算U= - DxF 矩阵3x1
	for (int i = 0; i < DOF3; i ++)
	{
		for (int j = 0; j < MAXDIM; j ++)
		{
			U[i] += D[i][j] * F[j];
		}
	}

	force.xForce = -U[0] - env.xForce;
	force.yForce = -U[1] - env.yForce;
	force.nMoment = -U[2] - env.nMoment;

	//限制纵向力
	if (MAXSURGE < force.xForce)
	{
		force.xForce = MAXSURGE;
	} else if (-MAXSURGE >force.xForce)
	{
		force.xForce = -MAXSURGE;
	}

	//限制横向力
	if (MAXSWAY < force.yForce)
	{
		force.yForce = MAXSWAY;
	} else if (-MAXSWAY >force.yForce)
	{
		force.yForce = -MAXSWAY;
	}

	//限制转艏力矩
	if (MAXYAW < force.nMoment)
	{
		force.nMoment = MAXYAW;
	} else if (-MAXYAW >force.nMoment)
	{
		force.nMoment = -MAXYAW;
	}

}

//设置预测周期
void NMPCcontroller::setT(const double period)
{
	T = period;
}

//设置权值
void NMPCcontroller::setWeight(const double lmd1, const double lmd2, const double lmd3)
{
	lambda[0] = lmd1;
	lambda[1] = lmd2;
	lambda[2] = lmd3;
}

//速度输入
void NMPCcontroller::setNu( const Nu nu )
{
	u = nu.u;
	v = nu.v;
	r = nu.r;
}

//位置输入
void NMPCcontroller::setEta( const Eta eta )
{
	x = eta.n;
	y = eta.e;
	psi = eta.psi;
}

//输入目标位置姿态
void NMPCcontroller::setTarget( const Eta eta )
{
	yd[0] = eta.n;
	yd[4] = eta.e;
	yd[8] = eta.psi;
}

//输入外界干扰力
void NMPCcontroller::setEnv(const Force3 forcEnv)
{
	env.xForce = forcEnv.xForce;
	env.yForce = forcEnv.yForce;
	env.nMoment = forcEnv.nMoment;
}

//控制力输出
Force6 NMPCcontroller::Force()
{
	return force;
}

//----------------------------------------------
//功能：采用部分主元的高斯消去法求3x3方阵A的逆矩阵B
//入口参数：输入方阵A，输出方阵B,方阵阶数n
//返回值：true or false
//----------------------------------------------
bool NMPCcontroller::inv(double (*A)[DOF3], double (*B)[DOF3])
{
	int i, j, k;
	double max, temp;
	double t[DOF3][DOF3];		//临时矩阵

	//将A矩阵存放在临时矩阵t[][]中
	for (i = 0; i < DOF3; i++)
	{
		for (j = 0; j < DOF3; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//初始化B矩阵为单位阵
	for (i = 0; i < DOF3; i++)
	{
		for (j = 0; j < DOF3; j++)
		{
			B[i][j] = (i == j)? (double) 1:0;
		}
	}
	for (i = 0; i < DOF3; i++)
	{
		//寻找主元
		max= t[i][i];
		k= i;
		for (j = i+1; j < DOF3; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max= t[j][i];
				k=j;
			}
		}

		//如果主元所在行不足i行，进行行交换
		if (k != i)
		{
			for (j = 0;j < DOF3;j++)
			{
				temp= t[i][j];
				t[i][j] = t[k][j];
				t[k][j] =temp;
				//B伴随交换
				temp= B[i][j];
				B[i][j] = B[k][j];
				B[k][j] =temp;
			}
		}
		//判断主元是否为0， 若是， 则矩阵A不是满秩矩阵，不存在逆矩阵
		if (t[i][i] == 0)
		{
			cout <<"There is no inverse matrix!";
			return false;
		}
		//消去A的第i列中除去i行以外的各行元素
		temp= t[i][i];
		for (j = 0; j < DOF3; j++)
		{
			t[i][j] = t[i][j] / temp; //主主对角线上的元素变为1
			B[i][j] = B[i][j] / temp; //伴随计算
		}
		for (j = 0; j < DOF3; j++) //第0行->第DOF3-1行
		{
			if (j != i)			//不是第i行
			{
				temp= t[j][i];
				for (k = 0; k < DOF3; k ++)	//第j行元素-i行元素*第j列i行元素
				{
					t[j][k] = t[j][k]- t[i][k]*temp;
					B[j][k] = B[j][k]- B[i][k]*temp;
				}
			}
		}
	}
	return true;
}
