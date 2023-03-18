// Field Oriented Control and Motion Smoothing
#pragma once
#include "PoseTypes.h"
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
    Vector robotVelocity;           // field re-oriented velocity
    double rotationalAccelleration = params.robotTurnAccel; // rate to accelerate the rotation rate input
    double velocityAccelleration = params.robotAccel;   // rate to accelerate the velocity input

public:
    /**
     *  sets the field oriented and smoothed x velocity,
     *  y velocity, and rotation rate for the robot
     * */
    Pose getRobotPoseVelocity(Pose velocitySetpoint, double navXAngle)
    {
            fieldVelocity.moveToward(velocitySetpoint, velocityAccelleration, rotationalAccelleration);
            /**------------Field Oriented Control------------**/
            robotVelocity = fieldVelocity.getPosition();
            robotVelocity.rotate(-navXAngle);
            return Pose{robotVelocity, fieldVelocity.getAngle()};
    }
};