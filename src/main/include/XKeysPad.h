#pragma once
#include "frc/Joystick.h"

class XKeysPad{
private:
    frc::Joystick *joy;
    double a;

public:
    XKeysPad(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    bool getSuctionCupState() {
        return joy->GetRawButton(7);
    }

    bool getHiRowPressed() {
        return joy->GetRawButton(3);
    }
    
    bool getMidRowPressed() {
        return joy->GetRawButton(2);
    }
    
    bool getLowRowPressed() {
        return joy->GetRawButton(1);
    }
    
    bool getFeederStationPressed() {
        return joy->GetRawButton(5);
    }

    bool getFloorPressed() {
        return joy->GetRawButton(4);
    }

    bool getHomePressed() {
        return joy->GetRawButton(6);
    }
    
    bool getIsCone() {
        return joy->GetRawButton(8);
    }
};