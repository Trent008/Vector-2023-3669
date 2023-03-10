#pragma once
#include "SwerveDrive.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of target positions and angles
 **/
class RobotPoseTargeting
{
private:
    double positionProportional; // rate at which to approach the target position
    double angleProportional;    // rate at which to approach the target angle
    Pose poseError;              // how fast the robot needs to move to get to its next position setpoint
    Pose distanceToSetpointPose;
    Pose swerveRate;
    Pose target;
    SwerveDrive *swerve;

public:
    RobotPoseTargeting(SwerveDrive *swerve, double positionProportional, double angleProportional)
    {   
        target = params.startingPose;
        this->positionProportional = positionProportional;
        this->angleProportional = angleProportional;
        this->swerve = swerve;
    }

    void targetPose(Pose setpoint, double driveRate, double rotationRate)
    {
        target.moveToward(setpoint, driveRate / 50, rotationRate / 50);
        distanceToSetpointPose = setpoint - target;
        poseError = target - swerve->getPose();
        swerveRate = poseError * Vector{positionProportional, angleProportional};
        swerveRate.limit(Vector{0.5, 0.5});
        swerve->Set(swerveRate, true);
    }

    bool poseReached(double positionTolerance, double angleTolerance)
    {
        return (distanceToSetpointPose.getPosition() < positionTolerance) && (std::abs(distanceToSetpointPose.getAngle()) <= angleTolerance);
    }
};