#pragma once
#include "frc/Joystick.h"
#include "utilities.h"
using namespace math;

class SpaceMousePro
{
private:
    frc::Joystick *joy;
    double speed;
    double deadband = 0.01;
    double change[6];
    double lastValue[6], value[6];

public:
    // SpaceMouse as joystick
    SpaceMousePro(frc::Joystick *joy, double speed)
    {
        this->joy = joy;
        this->speed = speed;
    }

    /**
     * resets the spaceMouse last position variable
     * to the current output
     * */
    void initialize()
    {
        for (int i = 0; i < 6; i++)
        {
            lastValue[i] = joy->GetRawAxis(i);
        }
    }

    void update()
    {
        for (int i = 0; i < 6; i++)
        {
            value[i] = joy->GetRawAxis(i);
            change[i] = value[i] - lastValue[i];
            if (change[i] > 1)
            {
                change[i] -= 2;
            }
            if (change[i] < -1)
            {
                change[i] += 2;
            }
            if (std::abs(change[i]) < deadband)
            {
                change[i] = 0;
            }
            lastValue[i] = value[i];
            change[i] *= speed;
        }
    }

    double getAxis(int number)
    {
        return change[number];
    }

    double getX()
    {
        return getAxis(0);
    }

    double getY()
    {
        return -getAxis(1);
    }

    double getZ()
    {
        return -getAxis(2);
    }

    double getXR()
    {
        return 8 * getAxis(3);
    }

    double getYR()
    {
        return -8 * getAxis(4);
    }

    double getZR()
    {
        return 8 * getAxis(5);
    }

    bool getMenuPressed()
    {
        return joy->GetRawButtonPressed(1);
    }

    bool getCTRLPressed()
    {
        return joy->GetRawButtonPressed(14);
    }

    bool getAltPressed()
    {
        return joy->GetRawButtonPressed(12);
    }

    bool getESCPressed()
    {
        return joy->GetRawButtonPressed(11);
    }

    bool getShiftPressed()
    {
        return joy->GetRawButtonPressed(13);
    }

    bool get1Pressed()
    {
        return joy->GetRawButtonPressed(7);
    }

    bool get2Pressed()
    {
        return joy->GetRawButtonPressed(8);
    }

    bool get3Pressed()
    {
        return joy->GetRawButtonPressed(9);
    }

    bool get4Pressed()
    {
        return joy->GetRawButtonPressed(10);
    }
};