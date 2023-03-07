#pragma once
#include "FOC.h"
#include "SwerveModule.h"
#include "Pose.h"
#include "frc/smartdashboard/SmartDashboard.h"

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
  SwerveModule *module[4]; // array of 4 swerve modules
  Pose robotPoseVelocity;
  Vector moduleVelocity[4];                // stores the velocity of all 4 modules
  Vector moduleTurnVector;                 // stores the turn-vector value for each module in turn
  Vector fieldwheelPositionChange;         // stores the velocity of each wheel in encoders/100ms in turn
  Vector largestVector;                    // fastest module velocity to be scaled to <= 1
  Vector averagePositionChange;            // average module velocity
  Vector fieldLocation = Vector{90, 54.7}; // field location in inches from the starting point
  AHRS navx{frc::SPI::Port::kMXP};         // NavX V2 object
  double navXAngle;                        // angle reported from the NavX2
  double offsetAngle;

public:
  FOC *mc; // updates the robot velocity and rotation rate

  SwerveDrive(FOC *mc, SwerveModule *module[4])
  {
    this->mc = mc;
    for (int i = 0; i < 4; i++)
    {
      this->module[i] = module[i];
    }
  }

  /**
   * runs the swerve modules using the values from the motion controller
   **/
  void Set(Pose fieldPoseVelocity = {}, bool isAutonomous = false)
  {
    navXAngle = navx.GetYaw();

    robotPoseVelocity = mc->getRobotPoseVelocity(fieldPoseVelocity, getOffsetRobotAngle(isAutonomous ? -90 : -180), !isAutonomous);
    largestVector = Vector{1, 0};
    for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
    {
      moduleTurnVector = module[i]->getTurnVector() * robotPoseVelocity.getAngle();
      moduleVelocity[i] = robotPoseVelocity.getPosition() + moduleTurnVector;
      if (moduleVelocity[i] > largestVector)
      {
        largestVector = moduleVelocity[i];
      }
    }

    averagePositionChange = Vector{}; // reset the average to zero before averaging again

    for (int i = 0; i < 4; i++) // loop through the module indexes again
    {
      moduleVelocity[i] /= abs(largestVector); // scale the vector sizes down to 1
      module[i]->Set(moduleVelocity[i]);       // drive the modules

      fieldwheelPositionChange = module[i]->getwheelPositionChange(); // get the wheel velocity
      fieldwheelPositionChange.rotate(getOffsetRobotAngle(-90));      // orient the wheel velocity in the robot's direction
      averagePositionChange += fieldwheelPositionChange;              // add the wheel velocity to the total sum
    }
    averagePositionChange /= 4; // find the average position change
    averagePositionChange *= (1 / 8.41 / 2048 * M_PI * 3.9);

    fieldLocation += averagePositionChange; // adds the distance travelled this cycle to the total distance to find the position
  }

  /**
   * Returns:
   * NavX2 yaw angle
   * -180 - 180 degrees
   * */
  double getOffsetRobotAngle(double offset)
  {
    offsetAngle = navXAngle + offset;
    return (offsetAngle > 180) ? offsetAngle - 360 : (offsetAngle < -180) ? offsetAngle + 360
                                                                          : offsetAngle;
  }

  void setPosition(Vector position)
  {
    fieldLocation = position;
  }

  void zeroYaw()
  {
    navx.ZeroYaw();
  }

  Pose getPose()
  {
    return Pose{fieldLocation, getOffsetRobotAngle(-90)};
  }

  Vector getPosition()
  {
    return fieldLocation;
  }
};