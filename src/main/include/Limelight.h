#pragma once
#include "Pose.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "string"

class Limelight {
private:
    std::shared_ptr<nt::NetworkTable> table_;
    std::string bot_pose_type;

public:
    Limelight(bool blue_alliance = true) {
        // Initialize NetworkTables
        bot_pose_type = "botpose_wpired";
        if (blue_alliance){
            bot_pose_type = "botpose_wpiblue";
        }
        table_ = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
    }

    double GetRobotX(){
        std::vector<double> robot_pose = table_->GetNumberArray(bot_pose_type, {});
        double x = robot_pose[0];
        return x;
    }
    double GetRobotY(){
        std::vector<double> robot_pose = table_->GetNumberArray(bot_pose_type, {});
        double y = robot_pose[1];
        return y;
    }
    double GetRobotAngle(){
        std::vector<double> robot_pose = table_->GetNumberArray(bot_pose_type, {});
        double z = robot_pose[2];
        return z;
    }

    bool targetExists() {
        return false;//table_->GetBoolean();
    }
    
    Vector GetRobotPosition() {
        return Vector{GetRobotX(), GetRobotY()} / 0.0254;
    }
};