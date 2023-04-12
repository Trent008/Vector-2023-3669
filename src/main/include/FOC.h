// Field Oriented Control and Motion Smoothing
#pragma once
#include "Parameters.h"

/**
 * Field Oriented Control:
 * converts field velocity input to velocity relative to the robot
 * after smoothing the field velocity input
 * */
class FOC

{
private:
    Pose fieldVelocity;             // smoothed/accellerated field velocity
    Vector robotVelocity;           // field-oriented robot velocity
    double robotAccel = params.robotAccel;   // rate to accelerate the pose rate

public:
    /**
     *  sets the field oriented and smoothed x velocity,
     *  y velocity, and rotation rate for the robot
     * */
    Pose getRobotVelocity(Pose velocitySetpoint, Angle navXAngle)
    {
            fieldVelocity.moveToward(velocitySetpoint, robotAccel);
            /**------------Field Oriented Control------------**/
            robotVelocity = fieldVelocity.getPosition();
            robotVelocity - navXAngle;
            return Pose{robotVelocity, fieldVelocity.getAngle()};
    }


};