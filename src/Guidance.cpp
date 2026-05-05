#include <cmath>
#include "Guidance.h"

double compute_distance(const state& interceptor, const state& target)
{
    double rx = target.x - interceptor.x;
    double ry = target.y - interceptor.y;

    return sqrt(rx * rx + ry * ry);
}

double compute_lambda(const state& interceptor, const state& target)
{
    double rx = target.x - interceptor.x;
    double ry = target.y - interceptor.y;

    return atan2(ry, rx);
}

double compute_closing_velocity(const state& interceptor, const state& target)
{
    double rx = target.x - interceptor.x;
    double ry = target.y - interceptor.y;

    double rvx = target.vx - interceptor.vx;
    double rvy = target.vy - interceptor.vy;

    double distance = sqrt(rx * rx + ry * ry);

    return - (rx * rvx + ry * rvy) / distance;
}

double compute_lambda_dot(double lambda, double lambda_prev, double dt)
{
    return (lambda - lambda_prev) / dt;
}

double compute_PN_acceleration(double N, double Vc, double lambda_dot)
{
    return N * Vc * lambda_dot;
}

double limit_acceleration(double ac, double max_ac)
{
    if (ac > max_ac) {
        ac = max_ac;
    }

    if (ac < -max_ac) {
        ac = -max_ac;
    }

    return ac;
}

void apply_lateral_acceleration(state& interceptor, double ac)
{
    double magnitude = sqrt(interceptor.vx * interceptor.vx + interceptor.vy * interceptor.vy);

    double vx_hat = interceptor.vx / magnitude;
    double vy_hat = interceptor.vy / magnitude;

    double acx = ac * -vy_hat;
    double acy = ac * vx_hat;

    interceptor.ax = acx;
    interceptor.ay = acy;
}
