#pragma once
#include <networktables/NetworkTableInstance.h>
class Limelight {
public:
    Limelight() {
        // Initialize NetworkTables
        table_ = nt::NetworkTableInstance::GetDefault().GetTable("limelight-laurel");
    }

    double getXAngle() {
        return table_->GetNumber("tx", 0.0);
    }

    double getYAngle() {
        return table_->GetNumber("ty", 0.0);
    }

    double getArea() {
        return table_->GetNumber("ta", 0.0);
    }
    
    double GetX(){
        std::vector<double> target_pose = table_->GetNumberArray("targetpose_robotspace", {});
        double x = target_pose[0];
        return x;
    }
    double GetY(){
        std::vector<double> target_pose = table_->GetNumberArray("targetpose_robotspace", {});
        double y = target_pose[1];
        return y;
    }
    double GetZ(){
        std::vector<double> target_pose = table_->GetNumberArray("targetpose_robotspace", {});
        double z = target_pose[2];
        return z;
    }

    double GetRobotX(){
        std::vector<double> target_pose = table_->GetNumberArray("botpose", {});
        double x = target_pose[0];
        return x;
    }
    double GetRobotY(){
        std::vector<double> target_pose = table_->GetNumberArray("botpose", {});
        double y = target_pose[1];
        return y;
    }
    double GetRobotAngle(){
        std::vector<double> target_pose = table_->GetNumberArray("botpose", {});
        double z = target_pose[2];
        return z;
    }
private:
    std::shared_ptr<nt::NetworkTable> table_;
};