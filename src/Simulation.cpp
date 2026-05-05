#include <iostream>
#include <fstream>
#include <iomanip>
#include "State_Vectors.h"
#include "Dynamics.h"
#include "Guidance.h"

using namespace std;

int main()
{
    state interceptor;
    state target;

    double dt = 0.01;
    double N = 4.0;
    double max_ac = 100.0;

    interceptor.x = 0.0;
    interceptor.y = 0.0;
    interceptor.vx = 300.0;
    interceptor.vy = 0.0;
    interceptor.ax = 0.0;
    interceptor.ay = 0.0;

    target.x = 1000.0;
    target.y = 500.0;
    target.vx = 100.0;
    target.vy = 0.0;
    target.ax = 0.0;
    target.ay = 10.0;

    ofstream outFile("simulation_output.csv");
    outFile << "time,distance,lambda,lambda_dot,ix,iy,tx,ty,Vc,ac,iax,iay\n";
    outFile << fixed << setprecision(6);

    double lambda_prev = compute_lambda(interceptor, target);

    for (double t = 0.0; t < 20.0; t += dt) {
        double distance = compute_distance(interceptor, target);
        double lambda = compute_lambda(interceptor, target);
        double lambda_dot = compute_lambda_dot(lambda, lambda_prev, dt);
        lambda_prev = lambda;

        double Vc = compute_closing_velocity(interceptor, target);
        double ac = compute_PN_acceleration(N, Vc, lambda_dot);
        ac = limit_acceleration(ac, max_ac);

        apply_lateral_acceleration(interceptor, ac);

        update_state(interceptor, dt);
        update_state(target, dt);

        cout << t << " " << distance << endl;

        outFile << t << ","
                << distance << ","
                << lambda << ","
                << lambda_dot << ","
                << interceptor.x << ","
                << interceptor.y << ","
                << target.x << ","
                << target.y << ","
                << Vc << ","
                << ac << ","
                << interceptor.ax << ","
                << interceptor.ay << "\n";

        if (distance < 1.0) {
            break;
        }
    }

    outFile.close();

    return 0;
}
