#pragma once
#include "Pose.h"
#include <networktables/NetworkTableInstance.h>

class Limelight {
public:
    Limelight() {
        // Initialize NetworkTables
        table_ = nt::NetworkTableInstance::GetDefault().GetTable("limelight-laurel");
    }

    double GetRobotX(){
        std::vector<double> robot_pose = table_->GetNumberArray("botpose", {});
        double x = robot_pose[0];
        return x;
    }
    double GetRobotY(){
        std::vector<double> robot_pose = table_->GetNumberArray("botpose", {});
        double y = robot_pose[1];
        return y;
    }
    double GetRobotAngle(){
        std::vector<double> robot_pose = table_->GetNumberArray("botpose", {});
        double z = robot_pose[2];
        return z;
    }
    
    Pose GetRobotPose() {
        return Pose{{GetRobotX(), GetRobotY()}, GetRobotAngle()};
    }
private:
    std::shared_ptr<nt::NetworkTable> table_;
};