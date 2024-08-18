#include "Kinematics.h"
#include <iostream>

Kinematic::Kinematic()
	// : l1(55), l2(13), l3(110), l4(120),
	// L(185), W(80), H(180)
	// : l1(59.7), l2(10), l3(107), l4(115),
	// L(185), W(80), H(180)
	: l0(9), l1(59.7), l2(10), l3(106.99), l4(14.75), l5(115), 
	L(91.65*2), W(39*2), H(180)
{
	l3_4_simple_distance = sqrt(pow(l3,2) + pow(l4,2));
	l3_4_simple_angle = atan2(l4,l3);
	for (int i = 0; i < 12; i++)
		thetas[i / 3][i % 3] = 0;
}

void Kinematic::bodyIK(double returns[4][4][4], double angles[3], double center[3]) {

	if (center[0] > 110) center[0] = 110;
	if (center[0] < -270) center[0] = -270;
	if (center[1] < -60) center[1] = -60;
	if (center[1] > 95) center[1] = 95;
	if (center[2] < -110) center[2] = -110;
	if (center[2] > 110) center[2] = 110;

	if (angles[0] > 0.84) angles[0] = 0.84;
	if (angles[0] < -0.84) angles[0] = -0.84;
	if (angles[1] > 1.10) angles[1] = 1.10;
	if (angles[1] < -1.10) angles[1] = -1.10;
	if (angles[2] > 0.84) angles[2] = 0.84;
	if (angles[2] < -0.68) angles[2] = -0.68;

	double Rx[4][4] = { {1,0,0,0},{0,cos(angles[0]),-sin(angles[0]),0},{0,sin(angles[0]),cos(angles[0]),0},{0,0,0,1} };
	double Ry[4][4] = { {cos(angles[1]), 0, sin(angles[1]), 0}, {0,1,0,0}, {-sin(angles[1]), 0, cos(angles[1]), 0}, {0,0,0,1} };
	double Rz[4][4] = { {cos(angles[2]), -sin(angles[2]), 0, 0}, {sin(angles[2]), cos(angles[2]),0,0}, {0,0,1,0}, {0,0,0,1} };
	double T[4][4] = { {0,0,0,center[0]}, {0,0,0,center[1]}, {0,0,0,center[2]}, {0,0,0,0} };

	double Rxyz[4][4] = { 0, };
	double Ryz[4][4] = { 0, };

	mat.dot(Ryz, Ry, Rz);
	mat.dot(Rxyz, Rx, Ryz);

	double Tm[4][4] = { 0, };
	mat.add(Tm, Rxyz, T);

	mat.dot(returns[0], Tm, func1);
	mat.dot(returns[1], Tm, func2);
	mat.dot(returns[2], Tm, func3);
	mat.dot(returns[3], Tm, func4);
}

void Kinematic::legIK(double returns[3], double point[4]) /* 3 float, 3 float */

{
	double x = point[0]; // -
	double y = point[1] - l0; // |
	double z = point[2]; // /

	double F = pow(x,2) + pow(y,2) - pow(l1,2);
	
	if (F < 0) {
		F = l1;
	}
	else F = sqrt(F);

	double G = F - l2;
	double H = sqrt(pow(G,2) + pow(z,2));
	returns[0] = - atan2(y, x) - atan2(F, -l1);

	double D = (pow(H, 2) - pow(l3_4_simple_distance, 2) - pow(l5, 2)) / (2 * l3_4_simple_distance * l5);

	if (D > 1) {
		returns[2] = 0;
	} else if (D < -1) {
		returns[2] = pi;
	}
	else returns[2] = acos(D) + l3_4_simple_angle;

	returns[1] = atan2(z, G) - atan2(l5 * sin(returns[2]), l3_4_simple_distance + l5 * cos(returns[2])) - l3_4_simple_angle;
}

void Kinematic::calcIK(double returns[4][3], double Lp[4][4], double angles[3], double center[3]) {
	
	double T[4][4][4] = { 0, };
	bodyIK(T, angles, center);
	

	{
		double invT[4][4] = { 0, };
		mat.inv(invT, T[0]);
		double dotT[4] = { 0, };
		Lp[0][3] = 1;
		mat.dot(dotT, invT, Lp[0]);
		legIK(returns[0], dotT);
	}
	{
		double invT[4][4] = { 0, };
		mat.inv(invT, T[1]);
		double dotT[4] = { 0, };
		double dotIT[4] = { 0, };
		Lp[1][3] = 1;
		mat.dot(dotT, invT, Lp[1]);
		mat.dot(dotIT, Ix, dotT);
		legIK(returns[1], dotIT);
	}
	{
		double invT[4][4] = { 0, };
		mat.inv(invT, T[2]);
		double dotT[4] = { 0, };
		Lp[2][3] = 1;
		mat.dot(dotT, invT, Lp[2]);
		legIK(returns[2], dotT);
	}
	{
		double invT[4][4] = { 0, };
		mat.inv(invT, T[3]);
		double dotT[4] = { 0, };
		double dotIT[4] = { 0, };
		Lp[3][3] = 1;
		mat.dot(dotT, invT, Lp[3]);
		mat.dot(dotIT, Ix, dotT);
		legIK(returns[3], dotIT);
	}
}

void Kinematic::legFK(double returns[4], int leg, double joint[4])  /* 3 float, 3 float */
{	
	double dir0 = 1;
	double dir1 = 1;
	joint[1] = joint[1] + l3_4_simple_angle;
	joint[2] = joint[2] - l3_4_simple_angle;
	returns[3] = 1;
	switch (leg) {
		case 1:
			dir1= -1;
		break;
		case 2:
			dir0 = -1;
		break;
		case 3:
			dir0 = -1;
			dir1 = -1;
		break;
	}
	returns[0] = l3_4_simple_distance*sin(joint[1]) + l5*sin(joint[1]+joint[2]) + dir0* L/2;
	double zp = l3_4_simple_distance*cos(joint[1]) + l5*cos(joint[1]+joint[2]) + l2;
	double lp = sqrt(pow(zp,2) + pow(l1,2));
	double alpha = -dir1*atan2(zp, l1);
	returns[2] = dir1*(lp * cos(alpha - joint[0]) + W/2);
	returns[1] = dir1*lp * sin(alpha - joint[0]) + l0;
}

void Kinematic::bodyFK(double returns[4][4], double joint[4][4], double angles[3], double center[3])
{

	double angle[3] = {angles[0], angles[2], angles[1]};
	double T_wb[4][4] = {{cos(angle[2])*cos(angle[1]),  cos(angle[2])*sin(angle[1])*sin(angle[0])-sin(angle[2])*cos(angle[1]), 	cos(angle[2])*sin(angle[1])*cos(angle[0])+sin(angle[2])*sin(angle[1]), 	center[0]}, 
						{sin(angle[2])*cos(angle[1]),   sin(angle[2])*sin(angle[1])*sin(angle[0])+cos(angle[2])*cos(angle[1]), 	sin(angle[2])*sin(angle[1])*cos(angle[0])-cos(angle[2])*sin(angle[1]), 	center[1]},
						{-sin(angle[1]),    			cos(angle[1])*sin(angle[0]), 											cos(angle[1])*cos(angle[0]), 											center[2]},
						{0,								0,																		0,																		1}};
	double lp[4][4];
	for (int i = 0; i <4; i++) {
		legFK(lp[i], i, joint[i]);
		mat.dot(returns[i], T_wb, lp[i]);
	}
}

//void Kinematic::testlegIK(float* returns, float* point);