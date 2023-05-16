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
    driveMotor1.Config_kP(0, 0.12, 20);
    driveMotor2.Config_kP(0, 0.12, 20);
    driveMotor3.Config_kP(0, 0.12, 20);
    driveMotor4.Config_kP(0, 0.12, 20);
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
  driveMotor1.SetRotorPosition(0_tr);
  driveMotor2.SetRotorPosition(0_tr);
  driveMotor3.SetRotorPosition(0_tr);
  driveMotor4.SetRotorPosition(0_tr);
  swerve.zeroYaw();
}

void Robot::AutonomousPeriodic()
{
  // move the swerve drive twards the next setpoint
  swerveTargeting.targetPose(params.setpoints[i].pose, params.setpoints[i].driveRate, params.setpoints[i].rotationRate);
  // set the arm pose
  arm.setPose(params.setpoints[i].armPose);
  arm.setCupState(params.setpoints[i].cupState);
  arm.run(cycles > 25);
  // go to next setpoint if this setpoint has been reached
  if (arm.poseReached(0.01) && swerveTargeting.poseReached(3, 5) && (i < 14))
  {
    i++;
  }
  cycles++;
}

void Robot::TeleopInit()
{
  params.isAutonomous = false;
}

void Robot::TeleopPeriodic()
{
  swerve.Set(xboxC.getFieldVelocity());
  // switch between cone mode and cube mode
  arm.setCupState(buttonPad.getSuctionCupState());

  yellowLeft.Set(buttonPad.getIsCone());
  yellowRight.Set(buttonPad.getIsCone());
  blueLeft.Set(!buttonPad.getIsCone());
  blueRight.Set(!buttonPad.getIsCone());

  if (xboxC.zero()) {
    swerve.zeroYaw();
  }

  // arm position buttons
  if (buttonPad.getHomePressed())
  {
    as1 = (as2 == 1 || as2 == 5)? 4 : (as2 == 3)? 7 : 0;
    as2 = 0;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getLowRowPressed())
  {
    if (as2 == 0) {as1 = 4;}
    else {as1 = 1;}
    as2 = 1;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getMidRowPressed())
  {
    as1 = 2;
    as2 = 2;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getHiRowPressed())
  {
    as1 = (as2 == 0)? 7 : 3;
    as2 = 3;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getFloorPressed())
  {
    if (as2 == 0) {as1 = 4;}
    else {as1 = 5;}
    as2 = 5;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }

  if (buttonPad.getFeederStationPressed())
  {
    as1 = 6;
    as2 = 6;
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
  }
  
  arm.setCupState(buttonPad.getSuctionCupState());
  arm.run(true, Vector{SMEnt.getY(), SMEnt.getZ()}, SMEnt.getYR(), SMEnt.getXR()); //logi.getY(), logi.getZ()}, -4*logi2.getY(), 4 * logi2.getZ());
  if (as1 == 4 && arm.poseReached(1))
  {
    arm.setPose(armPresets[as2][buttonPad.getIsCone()]);
    as1 = as2;
  } 
  if (as1 == 7 && arm.poseReached(1))
  {
    arm.setPose(armPresets[as2][buttonPad.getIsCone()] + Pose{{0, 0}, 15});
    as1 = as2;
  }
  if (buttonPad.getIsCone() != lastButtonState)
  {
    arm.setPose(armPresets[as1][buttonPad.getIsCone()]);
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
