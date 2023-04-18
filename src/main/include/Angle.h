#pragma once
#include "cmath"
#include "math.h"

class Angle
{
protected:
    double a;

public:
    Angle(double angle = 0)
    {
        a = angle;
    }

    operator double() const
    {
        return a;
    }

    Angle operator+(Angle const &obj)
    {
        Angle res;
        res.a = a + obj.a;
        res.a += res.a > 180 ? -360 : res.a < -180 ? 360
                                       : 0;
        return res;
    }

    void operator+=(Angle const &obj)
    {
        a += obj.a;
        a += a > 180 ? -360 : a < -180 ? 360
                                       : 0;
    }

    Angle operator-(Angle const &obj)
    {
        Angle res;
        res.a = a - obj.a;
        res.a += res.a > 180 ? -360 : res.a < -180 ? 360
                                       : 0;
        return res;
    }

    Angle operator-=(Angle const &obj)
    {
        a -= obj.a;
        a += a > 180 ? -360 : a < -180 ? 360
                                       : 0;
    }

    Angle operator*(Angle const &obj)
    {
        return Angle{a * obj.a};
    }

    void operator*=(double const &value)
    {
        a *= value;
    }

    void operator/=(double const &value)
    {
        a /= value;
    }

    double magnitude()
    {
        return std::abs(a);
    }

    void moveToward(Angle target, double rate)
    {
        target.subtract(*this);
        if (target.magnitude() > 2 * rate)
        {
            target.divide(target.magnitude());
            target.scale(rate);
            this->add(target);
        }
        else
        {
            target.divide(2);
            this->add(target);
        }
    }

    // returns the angle's value
    double get()
    {
        return a;
    }
};