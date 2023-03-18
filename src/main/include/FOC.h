// Field Oriented Control and Motion Smoothing
#pragma once
#include "AHRS.h"
#include "PoseTypes.h"

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
    double rotationalAccelleration; // rate to accelerate the rotation rate input
    double velocityAccelleration;   // rate to accelerate the velocity input

public:
    FOC(double velocityAcceleration, double rotationalAccelleration)
    {
        this->velocityAccelleration = velocityAcceleration;
        this->rotationalAccelleration = rotationalAccelleration;
    }

    /**
     *  sets the field oriented and smoothed x velocity,
     *  y velocity, and rotation rate for the robot
     * */
    Pose getRobotPoseVelocity(Pose velocitySetpoint, double navXAngle, bool isAutonomous)
    {
        if (!isAutonomous)
        {
            fieldVelocity.moveToward(velocitySetpoint, velocityAccelleration, rotationalAccelleration);
            /**------------Field Oriented Control------------**/
            robotVelocity = fieldVelocity.getPosition();
            robotVelocity.rotate(-navXAngle);
            return Pose{robotVelocity, fieldVelocity.getAngle()};
        }
        else
        {
            robotVelocity = velocitySetpoint.getPosition();
            robotVelocity.rotate(-navXAngle);
            return Pose{robotVelocity, velocitySetpoint.getAngle()};
        }
    }
};