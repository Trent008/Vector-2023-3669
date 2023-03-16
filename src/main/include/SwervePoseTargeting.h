#pragma once
#include "SwerveDrive.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of current positions and angles
 **/
class SwervePoseTargeting
{
private:
    double positionProportional; // rate at which to approach the current position
    double angleProportional;    // rate at which to approach the current angle
    bool useAcceleration;       // whether or not to accelerate the robot rate
    Pose poseError;              // how fast the robot needs to move to get to its next position setpoint
    Pose distanceToSetpointPose;
    Pose swerveRate;
    SwerveDrive *swerve;

public:
    SwervePoseTargeting(SwerveDrive *swerve, double positionProportional, double angleProportional)
    {   
        this->positionProportional = positionProportional;
        this->angleProportional = angleProportional;
        this->swerve = swerve;
    }

    void targetPose(Pose setpoint, double driveRate, double rotationRate)
    {   
        poseError = setpoint - swerve->getPose();
        swerveRate = poseError * Vector{positionProportional, angleProportional};
        // use acceleration if the speed is not being accelerated by the Proportional
        if (swerveRate.getPosition() > driveRate) {
            useAcceleration = true;
        } else {
            useAcceleration = false;
        }

        swerveRate.limit(Vector{driveRate, rotationRate});
        swerve->Set(swerveRate, true, useAcceleration);
    }

    bool poseReached(double positionTolerance, double angleTolerance)
    {
        return (poseError.getPosition() < positionTolerance) && (std::abs(distanceToSetpointPose.getAngle()) <= angleTolerance); //distanceToSetpointPose.getPosition()
    }

    void setCurrentPosition(Vector position) {
        swerve->setPosition(position);
    }


};