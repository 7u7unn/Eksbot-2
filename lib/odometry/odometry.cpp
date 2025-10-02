#include "odometry.h"

Odometry::Odometry(float w_radi, float base_width, float gearbox_r, float gearbox_l)
{
    this->w_radi = w_radi;
    this->base_width = base_width;
    this->gearbox_r = gearbox_r;
    this->gearbox_l = gearbox_l;

    x = 0.0;
    y = 0.0;
    theta = 0.0;
    prevLeftTicks = 0;
    prevRightTicks = 0;
}