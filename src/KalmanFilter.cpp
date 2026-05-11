#include "KalmanFilter.h"

void initialize_filter(KalmanFilter& kf, const state& initial_target)
{
    kf.x[0] = initial_target.x;
    kf.x[1] = initial_target.y;
    kf.x[2] = initial_target.vx;
    kf.x[3] = initial_target.vy;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf.P[i][j] = 0.0;
            kf.Q[i][j] = 0.0;
        }
    }

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            kf.R[i][j] = 0.0;
        }
    }

    kf.P[0][0] = 100.0;
    kf.P[1][1] = 100.0;
    kf.P[2][2] = 25.0;
    kf.P[3][3] = 25.0;

    kf.Q[0][0] = 0.1;
    kf.Q[1][1] = 0.1;
    kf.Q[2][2] = 1.0;
    kf.Q[3][3] = 1.0;

    kf.R[0][0] = 25.0;
    kf.R[1][1] = 25.0;
}

void predict(KalmanFilter& kf, double dt)
{
    kf.x[0] = kf.x[0] + kf.x[2] * dt;
    kf.x[1] = kf.x[1] + kf.x[3] * dt;

    double P_new[4][4];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_new[i][j] = kf.P[i][j];
        }
    }

    P_new[0][0] += dt * (kf.P[2][0] + kf.P[0][2]) + dt * dt * kf.P[2][2];
    P_new[0][1] += dt * kf.P[2][1] + dt * kf.P[0][3] + dt * dt * kf.P[2][3];
    P_new[1][0] += dt * kf.P[3][0] + dt * kf.P[1][2] + dt * dt * kf.P[3][2];
    P_new[1][1] += dt * (kf.P[3][1] + kf.P[1][3]) + dt * dt * kf.P[3][3];

    P_new[0][2] += dt * kf.P[2][2];
    P_new[1][3] += dt * kf.P[3][3];
    P_new[2][0] += dt * kf.P[2][2];
    P_new[3][1] += dt * kf.P[3][3];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf.P[i][j] = P_new[i][j] + kf.Q[i][j];
        }
    }
}

void update(KalmanFilter& kf, double measured_x, double measured_y)
{
    double y0 = measured_x - kf.x[0];
    double y1 = measured_y - kf.x[1];

    double S00 = kf.P[0][0] + kf.R[0][0];
    double S01 = kf.P[0][1];
    double S10 = kf.P[1][0];
    double S11 = kf.P[1][1] + kf.R[1][1];

    double detS = S00 * S11 - S01 * S10;

    double invS00 = S11 / detS;
    double invS01 = -S01 / detS;
    double invS10 = -S10 / detS;
    double invS11 = S00 / detS;

    double K[4][2];

    for (int i = 0; i < 4; i++) {
        K[i][0] = kf.P[i][0] * invS00 + kf.P[i][1] * invS10;
        K[i][1] = kf.P[i][0] * invS01 + kf.P[i][1] * invS11;
    }

    for (int i = 0; i < 4; i++) {
        kf.x[i] = kf.x[i] + K[i][0] * y0 + K[i][1] * y1;
    }

    double P_old[4][4];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_old[i][j] = kf.P[i][j];
        }
    }

    for (int j = 0; j < 4; j++) {
        kf.P[0][j] = P_old[0][j] - K[0][0] * P_old[0][j] - K[0][1] * P_old[1][j];
        kf.P[1][j] = P_old[1][j] - K[1][0] * P_old[0][j] - K[1][1] * P_old[1][j];
        kf.P[2][j] = P_old[2][j] - K[2][0] * P_old[0][j] - K[2][1] * P_old[1][j];
        kf.P[3][j] = P_old[3][j] - K[3][0] * P_old[0][j] - K[3][1] * P_old[1][j];
    }
}

state get_estimated_state(const KalmanFilter& kf)
{
    state estimate;

    estimate.x = kf.x[0];
    estimate.y = kf.x[1];
    estimate.vx = kf.x[2];
    estimate.vy = kf.x[3];
    estimate.ax = 0.0;
    estimate.ay = 0.0;

    return estimate;
}
