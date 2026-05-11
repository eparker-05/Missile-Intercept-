#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "State_Vectors.h"

struct KalmanFilter {
    double x[4];      // [x, y, vx, vy]
    double P[4][4];   // uncertainty
    double Q[4][4];   // process noise
    double R[2][2];   // measurement noise
};

void initialize_filter(KalmanFilter& kf, const state& initial_target);
void predict(KalmanFilter& kf, double dt);
void update(KalmanFilter& kf, double measured_x, double measured_y);
state get_estimated_state(const KalmanFilter& kf);

#endif
