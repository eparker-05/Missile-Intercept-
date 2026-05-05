#ifndef Guidance_h
#define Guidance_h

#include "State_Vectors.h"

double compute_distance(const state& interceptor, const state& target);
double compute_lambda(const state& interceptor, const state& target);
double compute_closing_velocity(const state& interceptor, const state& target);
double compute_lambda_dot(double lambda, double lambda_prev, double dt);
double compute_PN_acceleration(double N, double Vc, double lambda_dot);
double limit_acceleration(double ac, double max_ac);
void apply_lateral_acceleration(state& interceptor, double ac);

#endif
