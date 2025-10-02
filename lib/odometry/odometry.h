#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry{
    public:
        Odometry(
            float w_radi, 
            float base_width, 
            float gearbox_r, 
            float gearbox_l
        );
        void update(long right_ticks, long left_ticks);
        void getX();
        float getY();
        float getTheta();

    private:
        float wheelRadius;    // wheel radius (m)
        float baseWidth;      // distance between wheels (m)
        int ticksPerRev;      // encoder resolution

        float x, y, theta;    // current pose (m, m, rad)
        long prevLeftTicks;   // previous encoder reading
        long prevRightTicks;
};

#endif

