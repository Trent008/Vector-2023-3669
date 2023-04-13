#pragma once
#include "AHRS.h"
#include "Pose.h"
#include "Parameters.h"
#include "SwerveModule.h"

/**
 * creates a swerve drive object that controls
 * an array of swerve drive modules
 * Parameters:
 *    FOC *mc : for motion smoothing and F.O.C.
 *    SwerveModule *module[4] : array of 4 modules to control
 * */
class SwerveDrive
{
private:
    Pose robotRate;
    Pose fieldRate;
    Vector moduleVelocity;           // stores the velocity of each module in turn
    Vector wheelPositionChange;      // stores the encoder position change vector of each wheel in turn
    Vector largestVector;            // fastest module velocity to be limited to 1
    Vector averagePositionChange;    // average module position change
    Vector robotDisplacement{};      // field location in inches from the starting point
    AHRS navx{frc::SPI::Port::kMXP}; // NavX V2 object
    Angle navXAngle;
    Angle fieldAngle;

public:
    /**
     * runs the swerve modules using the values from the motion controller
     **/
    void Set(SwerveModule modules[4], Pose driveRate = {})
    {
        navXAngle = navx.GetYaw();
        fieldAngle = navXAngle + params.startingPose.getAngle();

        robotRate = driveRate.getRotatedCCW(fieldAngle); // robot orient the drive rate

        largestVector = Vector{1, 0};
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            moduleVelocity = modules[i].getVector(robotRate);
            if (moduleVelocity > largestVector)
            {
                largestVector = moduleVelocity;
            }
        }
        driveRate /= abs(largestVector);                    // limit the rates to never max out modules
        fieldRate.moveToward(driveRate, params.robotAccel); // accelerate toward the drive rate target
        robotRate = fieldRate.getRotatedCCW(fieldAngle);    // robot orient the drive rate

        averagePositionChange = Vector{}; // reset the average to zero before averaging again
        for (int i = 0; i < 4; i++)       // loop through the module indexes again
        {
            modules[i].Set(modules[i].getVector(robotRate));                          // set each module to the accelerated robot velocity
            averagePositionChange += modules[i].getwheelPositionChange() + navXAngle; // add the wheel velocity to the total sum
        }
        averagePositionChange /= 4; // find the average position change
        averagePositionChange *= (1 / 6.75 / 2048 * M_PI * 3.9);
        robotDisplacement += averagePositionChange; // adds the distance traveled this cycle to the total distance to find the position
    }

    void zeroYaw()
    {
        navx.ZeroYaw();
    }

    Pose getFieldPose()
    {
        return Pose{robotDisplacement + params.startingPose.getAngle(), fieldAngle};
    }
};