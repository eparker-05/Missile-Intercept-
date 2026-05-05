#include "Dynamics.h"

void update_state(state& obj, double dt)
{
    obj.vx = obj.vx + obj.ax * dt;
    obj.vy = obj.vy + obj.ay * dt;
    obj.x = obj.x + obj.vx * dt;
    obj.y = obj.y + obj.vy * dt;
}
