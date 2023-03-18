#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
    frc::CameraServer::StartAutomaticCapture();

    // swerve motor config
    driveMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor4.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    // arm PID config
    arm.left_J1.SetP(0.1);
    arm.right_J1.SetP(0.1);
    arm.j2.SetP(0.05);
    arm.j3.SetP(0.1);
    arm.j4.SetP(0.1);
}

void Robot::RobotPeriodic(){
    frc::SmartDashboard::PutBoolean("object type:", isCone);
}

void Robot::AutonomousInit()
{
  params.isAutonomous = true;
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);
  swerve.zeroYaw();
  // swerveTargeting.setCurrentPosition(limelight.GetRobotPosition());
}

void Robot::AutonomousPeriodic()
{
  // move the swerve drive twards the next setpoint
  if (params.setpoints[i].useLimelight)
  {
    if (limelight_left.GetRobotPosition() > 10 && limelight_right.GetRobotPosition() > 10)
    {
      swerve.setPosition((limelight_right.GetRobotPosition() + limelight_left.GetRobotPosition()) / 2);
    }
    if (limelight_left.GetRobotPosition() > 10)
    {
      swerve.setPosition(limelight_left.GetRobotPosition());
    }
    if (limelight_right.GetRobotPosition() > 10)
    {
      swerve.setPosition(limelight_right.GetRobotPosition());
    }
  }
  swerveTargeting.targetPose(params.setpoints[i].pose, params.setpoints[i].driveRate, params.setpoints[i].rotationRate);
  // set the arm pose
  arm.setArmPosition(params.setpoints[i].armPose);
  arm.run(t > 25);
  // go to next setpoint if this setpoint has been reached
  if (arm.poseReached(1) && swerveTargeting.poseReached(3, 5) && (i < 14))
  {
    i++;
  }
  t++;
}

void Robot::TeleopInit()
{
  params.isAutonomous = false;
  SMPro.initialize();
}

void Robot::TeleopPeriodic()
{
  swerve.Set(xboxC.getFieldVelocity());

  SMPro.update();
  
  // switch between cone mode and cube mode
  arm.setCupState(buttonPad.getSuctionCupState());

  // arm position buttons
  if (buttonPad.getFloorPressed())
  {
    arm.setArmPosition(floor(buttonPad.getIsCone()));
    armSetpointType = 1;
    isHomingFromFloor = false;
  }
  if (buttonPad.getFeederStationPressed())
  {
    arm.setArmPosition(feederStation(buttonPad.getIsCone()));
    armSetpointType = 2;
    isHomingFromFloor = false;
  }
  if (buttonPad.getLowRowPressed())
  {
    arm.setArmPosition(bottom(buttonPad.getIsCone()));
    armSetpointType = 3;
    isHomingFromFloor = false;
  }
  if (buttonPad.getMidRowPressed())
  {
    arm.setArmPosition(middle(buttonPad.getIsCone()));
    armSetpointType = 3;
    isHomingFromFloor = false;
  }
  if (buttonPad.getHiRowPressed())
  {
    arm.setArmPosition(top(buttonPad.getIsCone()));
    armSetpointType = 3;
    isHomingFromFloor = false;
  }
  if (buttonPad.getHomePressed())
  {
    if (armSetpointType == 0)
    {
      arm.setArmPosition(home(buttonPad.getIsCone()));
    }
    if (armSetpointType == 1)
    {
      isHomingFromFloor = true;
      arm.setArmPosition(ArmPose{{8, 16}, 10, true});
    }
    if (armSetpointType == 2)
    {
      arm.setArmPosition(home(buttonPad.getIsCone()));
    }
    if (armSetpointType == 3)
    {
      arm.setArmPosition(home(buttonPad.getIsCone()));
    }
    
    armSetpointType = 0;
  }
  arm.setCupState(buttonPad.getSuctionCupState());
  arm.run(true, Vector{SMPro.getY(), SMPro.getZ()}, SMPro.getYR(), SMPro.getXR());
  if (isHomingFromFloor && arm.poseReached(1))
  {
    isHomingFromFloor = false;
    arm.setArmPosition(home(buttonPad.getIsCone()));
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
