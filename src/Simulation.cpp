#include <iostream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>

#include "State_Vectors.h"
#include "Dynamics.h"
#include "Guidance.h"
#include "KalmanFilter.h"

using namespace std;

int main()
{
//---------Creating two vehicles----------
    state interceptor;
    state target;
//--------Initializing Values-----------
    double dt = 0.01; // s
    double N = 4.5;
    double max_ac = 300.0; // m/s^2
    int total_hits = 0;
    int total_misses = 0;

    KalmanFilter kf;

    default_random_engine generator;
    normal_distribution<double> noise(0.0, 4.0);
    uniform_real_distribution<double> dist(0.0, 1.0);
//----------Starting CSV----------
    ofstream outFile("simulation_output.csv");
    ofstream mcFile("monte_carlo_results.csv");


    outFile << "time,"
            << "true_distance,estimated_distance,"
            << "lambda,lambda_dot,"
            << "ix,iy,"
            << "true_tx,true_ty,"
            << "measured_tx,measured_ty,"
            << "estimated_tx,estimated_ty,"
            << "Vc,ac,acx,acy\n";

     mcFile << "run,target_x0,target_y0,target_vx0,target_ay,min_true_distance,hit\n";

    outFile << fixed << setprecision(6);
    mcFile << fixed << setprecision(6); 

    for(int run = 1; run <= 500; run ++){
    double min_true_distance = 1e9; // m
    //reset interceptor
    interceptor.x = 0.0; // m
    interceptor.y = 0.0; // m
    interceptor.vx = 300.0; // m/s
    interceptor.vy = 0.0; // m/s
    interceptor.ax = 0.0; // m/s^2
    interceptor.ay = 0.0; // m/s^2
    //randomize target
    double rx = dist(generator);
    double ry = dist(generator);
    double rand_vx = dist(generator);
    double rand_ay = dist(generator);
    double rand_vy = dist(generator);
    target.x = 1000.0 + (2000.0 - 1000.0) * rx; // m
    target.y =  300.0 + (1000.0 - 300.0) * ry; // m
    target.vx = 50.0 + (150.0 - 50.0) * rand_vx; // m/s
    target.vy = -40.0 + 80.0 * rand_vy; // m/s
    target.ax = 0.0; // m/s^2
    target.ay = -30.0 + (60.0) * rand_ay; // m/s^2

    double target_x0 = target.x;
    double target_y0 = target.y;
    double target_vx0 = target.vx;
    double target_ay0 = target.ay;

    double lambda_prev = compute_lambda(interceptor, target);
     //reset kalman filter
    initialize_filter(kf, target);
    bool hit = false;
    //run monte carlo
//----------Loop to update state--------------
    for (double t = 0.0; t < 20.0; t += dt) {

        double measured_x = target.x + noise(generator);
        double measured_y = target.y + noise(generator);

        predict(kf, dt);
        update(kf, measured_x, measured_y);

        state estimated_target = get_estimated_state(kf);

        double lambda = compute_lambda(interceptor, estimated_target);
        double lambda_dot = compute_lambda_dot(lambda, lambda_prev, dt);
        lambda_prev = lambda;

        double Vc = compute_closing_velocity(interceptor, estimated_target);

        double ac = compute_PN_acceleration(N, Vc, lambda_dot);
        ac = limit_acceleration(ac, max_ac);

        apply_lateral_acceleration(interceptor, ac);

        double acx = interceptor.ax;
        double acy = interceptor.ay;
//-------Function to update both vehicles---------
        update_state(interceptor, dt);
        update_state(target, dt);

        double true_distance = compute_distance(interceptor, target);
        double estimated_distance = compute_distance(interceptor, estimated_target);

        outFile << t << ","
                << true_distance << ","
                << estimated_distance << ","
                << lambda << ","
                << lambda_dot << ","
                << interceptor.x << ","
                << interceptor.y << ","
                << target.x << ","
                << target.y << ","
                << measured_x << ","
                << measured_y << ","
                << estimated_target.x << ","
                << estimated_target.y << ","
                << Vc << ","
                << ac << ","
                << acx << ","
                << acy << "\n";

        
        if (true_distance < min_true_distance) {
    min_true_distance = true_distance;
    }
//--------Checking if interceptor is in certain range of target----------
    if (true_distance < 5.0) {
        hit = true;
            break;
        }
    }
    mcFile << run << ","
               << target_x0 << ","
               << target_y0 << ","
               << target_vx0 << ","
               << target_ay0 << ","
               << min_true_distance << ","
               << hit << "\n";

//-------Updating hit counter---------
    if(hit) {
    total_hits++;
}
    else {
    total_misses++;
}
}

cout << "\n===== Monte Carlo Results =====" << endl;
cout << "Total Hits: " << total_hits << endl;
cout << "Total Misses: " << total_misses << endl;
cout << "Hit Percentage: "
     << (100.0 * total_hits / (total_hits + total_misses))
     << "%" << endl;

    outFile.close();
    mcFile.close();

    return 0;
}
