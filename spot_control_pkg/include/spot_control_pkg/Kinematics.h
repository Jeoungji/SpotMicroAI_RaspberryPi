#pragma once

#ifndef _KINEMATICS
#define _KINEMATICS

#include "Matrix.h"
#include <cmath>
#define pi 3.141592653589793238462643383
#define sHp 1
#define cHp 0

class Kinematic {
private:
	double l0;
	double l1;
	double l2;
	double l3;
	double l4;
	double l5;

	double L;
	double W;
	double H;

	double l3_4_simple_angle;
	double l3_4_simple_distance;

	float thetas[4][3];
	Matrix mat;

	double func1[4][4] = { {cHp, 0, sHp, L / 2}, {0,1,0,0},{-sHp,0,cHp,W / 2}, {0,0,0,1} };
	double func2[4][4] = { {cHp, 0, sHp, L / 2}, {0,1,0,0},{-sHp,0,cHp,- W / 2}, {0,0,0,1} };
	double func3[4][4] = { {cHp, 0, sHp, - L / 2}, {0,1,0,0},{-sHp,0,cHp,W / 2}, {0,0,0,1} };
	double func4[4][4] = { {cHp, 0, sHp, - L / 2}, {0,1,0,0},{-sHp,0,cHp,- W / 2}, {0,0,0,1} };
	double Ix[4][4] = { {-1,0,0,0},{0,1,0,0}, {0,0,1,0}, {0,0,0,1} };
public:
	Kinematic();
	void bodyIK(double returns[4][4][4], double angles[3], double center[3]);
	void legIK(double returns[3], double point[4]);
	//void calcLegPoints(float* returns, float* angles);
	void calcIK(double returns[4][3], double Lp[4][4], double angles[3], double center[3]);
	void legFK(double returns[4], int leg, double joint[4]);
	void bodyFK(double returns[4][4], double joint[4][4], double angles[3], double center[3]);
	//void testlegIK(float* returns, float* point);
public:
	//void setL_m(int num) { L_m = L_m + num; }
	//void setM_m(int num) { M_m = M_m + num; }
	//int getL() { return L; }
	//int getM() { return M; }
	//void addL(int num) { setL(getL() + num); }
	//void addM(int num) { setM(getM() + num); }
};

#endif