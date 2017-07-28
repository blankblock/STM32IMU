
#include "kalman.h"
#include "matrix.h"
#include "config.h"


void kalmanInit(KalmanFilterTypedef *filter)
{
	filter->Q_angle = Q_ANGLE_DEFAULT;
	filter->Q_bias = Q_GYRO_BIAS;
	filter->R_measure = R_ANGLE;
	filter->P[0][0] = 0;
	filter->P[0][1] = 0;
	filter->P[1][0] = 0;
	filter->P[1][1] = 0;
	filter->K[0] = 0;
	filter->K[1] = 0;
	filter->S = 0;
	filter->y = 0;
	filter->bias = 0;
}

float kalmanGetAngle(KalmanFilterTypedef *filter, float newAngle, float newRate, float dt)
{
	//step 1
	filter->rate = newRate - filter->bias;
	filter->angle += dt * filter->rate;
	//step 2: Update estimation error covariance - Project the error covariance ahead
	filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
	filter->P[0][1] -= dt * filter->P[1][1];
	filter->P[1][0] -= dt * filter->P[1][1];
	filter->P[1][1] += filter->Q_bias * dt;
	//Step 3
	filter->y = newAngle - filter->angle;
	//Step 4: Calculate Kalman gain
	filter->S = filter->P[0][0] + filter->R_measure;
	
	//Step 5: Kk = Pk|k-1 * H^T * S^-1_k
	filter->K[0] = filter->P[0][0]/filter->S;
	filter->K[1] = filter->P[1][0]/filter->S;
	
	//Step 6
	filter->angle+=filter->K[0] * filter->y;
	filter->bias +=filter->K[1] * filter->y;
	
	//Step 7
	filter->P[0][0] -= filter->K[0] * filter->P[0][0];
	filter->P[0][1] -= filter->K[0] * filter->P[0][1];
	filter->P[1][0] -= filter->K[1] * filter->P[0][0];
	filter->P[1][1] -= filter->K[1] * filter->P[0][1];
	
	return filter->angle;
}

void kalmanSetAngle(KalmanFilterTypedef *filter, float newAngle)
{
	filter->angle = newAngle;
}