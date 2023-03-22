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

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit()
{
  params.isAutonomous = true;
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);
  swerve.zeroYaw();
}

void Robot::AutonomousPeriodic()
{
  // move the swerve drive twards the next setpoint
  // if (params.setpoints[i].useLimelight)
  // {
  //   if (limelight_left.GetRobotPosition() > 10 && limelight_right.GetRobotPosition() > 10)
  //   {
  //     swerve.setPosition((limelight_right.GetRobotPosition() + limelight_left.GetRobotPosition()) / 2);
  //   }
  //   if (limelight_left.GetRobotPosition() > 10)
  //   {
  //     swerve.setPosition(limelight_left.GetRobotPosition());
  //   }
  //   if (limelight_right.GetRobotPosition() > 10)
  //   {
  //     swerve.setPosition(limelight_right.GetRobotPosition());
  //   }
  // }
  swerveTargeting.targetPose(params.setpoints[i].pose, params.setpoints[i].driveRate, params.setpoints[i].rotationRate);
  // set the arm pose
  arm.setPose(params.setpoints[i].armPose);
  arm.setCupState(params.setpoints[i].cupState);
  arm.run(cycles > 25);
  // go to next setpoint if this setpoint has been reached
  if (arm.poseReached(1) && swerveTargeting.poseReached(3, 5) && (i < 14))
  {
    i++;
  }
  cycles++;
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

  yellowLeft.Set(buttonPad.getIsCone());
  yellowRight.Set(buttonPad.getIsCone());
  blueLeft.Set(!buttonPad.getIsCone());
  blueRight.Set(!buttonPad.getIsCone());

  if (SMPro.getMenuPressed()) {
    swerve.zeroYaw();
  }

  // arm position buttons
  if (buttonPad.getHomePressed())
  {
    if (armSetpoint2 == 1 || armSetpoint2 == 5) {armSetpoint1 = 4;}
    else {armSetpoint1 = 0;}
    armSetpoint2 = 0;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getLowRowPressed())
  {
    if (armSetpoint2 == 0) {armSetpoint1 = 4;}
    else {armSetpoint1 = 1;}
    armSetpoint2 = 1;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getMidRowPressed())
  {
    armSetpoint1 = 2;
    armSetpoint2 = 2;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getHiRowPressed())
  {
    armSetpoint1 = 3;
    armSetpoint2 = 3;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getFloorPressed())
  {
    if (armSetpoint2 == 0) {armSetpoint1 = 4;}
    else {armSetpoint1 = 5;}
    armSetpoint2 = 5;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getFeederStationPressed())
  {
    armSetpoint1 = 6;
    armSetpoint2 = 6;
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
  }
  
  arm.setCupState(buttonPad.getSuctionCupState());
  arm.run(true, Vector{SMPro.getY(), SMPro.getZ()}, SMPro.getYR(), SMPro.getXR());
  if (armSetpoint1 == 4 && arm.poseReached(1))
  {
    arm.setPose(armPresets[armSetpoint2][buttonPad.getIsCone()]);
    armSetpoint1 = armSetpoint2;
  }
  if (buttonPad.getIsCone() != lastButtonState)
  {
    arm.setPose(armPresets[armSetpoint1][buttonPad.getIsCone()]);
    lastButtonState = buttonPad.getIsCone();
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
