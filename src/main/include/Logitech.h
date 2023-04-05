#pragma once
#include "frc/Joystick.h"
#include "math.h"
#include "cmath"

class Logitech
{
private:
    frc::Joystick *joy;
    double a;

public:
    Logitech(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    double getY()
    {
        a = joy->GetRawAxis(0);
        return 0.5 * (std::abs(a) > 0.08 ? a : 0);
    }

    double getZ()
    {
        a = joy->GetRawAxis(1);
        return 0.5 * (std::abs(a) > 0.08 ? a : 0);
    }
};