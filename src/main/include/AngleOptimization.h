#pragma once
#include "math.h"
#include <cmath>
/**
 * contains methods to find the most 
 * efficient way to turn from current given
 * angle to another given angle
 * */
class AngleOptimizer
{
    private:
        double option[4];
        double direction;
        double shortest;
    public:
        /**
         * returns the difference between the setpoint angle and the current angle
         * when facing forward or backward is allowed
        */
        double getShortestAngle(double current, double setpoint) {
            option[0] = setpoint - current;
            option[1] = option[0] + (std::signbit(option[0]) ? 360 : -360);
            option[2] = option[0] + (std::signbit(option[0]) ? 180 : -180);
            option[3] = option[2] + (std::signbit(option[2]) ? 360 : -360);
          
            // choose the lowest angle option
            shortest = option[0];
            for (int i = 1; i<4; i++) {
                if(std::abs(option[i]) <= std::abs(shortest)) {shortest = option[i];}
            }
            direction = (shortest == option[0] || shortest == option[1]) ? 1 : -1;
            return shortest;
        }

        double getDirection() {
            return direction;
        }
};

double difference;

/**
 * returns the difference between the setpoint angle and the current angle
 * when facing forward only is allowed
*/
double getDifference(double setpoint, double current)
{
    difference = setpoint - current;
    if(std::abs(difference) < 180) { return difference; }
    return difference + (std::signbit(difference) ? 360 : -360);
}

