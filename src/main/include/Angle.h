#pragma once
#include "math.h"
#include <cmath>

class Angle
{
private:
    double angle;

public:
    Angle(double angle = 0)
    {
        this->angle = angle;
    }

    void operator=(Angle const &obj)
    {
        angle = obj.angle;
    }

    void operator=(double val)
    {
        angle = val;
    }

    Angle operator+(Angle const &obj)
    {
        Angle res;
        res.angle = angle + obj.angle;
        res.angle = fmod(res.angle, 360);
        if (std::abs(res.angle) > 180) {
            res.angle += std::signbit(res.angle) ? 360: -360;
        }
        return res;
    }

    Angle operator-(Angle const &obj) {
        Angle res;
        res.angle = angle - obj.angle;
        res.angle = fmod(res.angle, 360);
        if (std::abs(res.angle) > 180) {
            res.angle += std::signbit(res.angle) ? 360: -360;
        }
        return res;
    }

    void operator+=(Angle const &obj)
    {
        *this = *this + obj;
    }

    void operator-=(Angle const &obj)
    {
        *this = *this - obj;
    }

    Angle operator*(Angle const &obj)
    {
        Angle res;
        res.angle = angle * obj.angle;
        return res;
    }

    Angle operator/(Angle const &obj)
    {
        Angle res;
        res.angle = angle / obj.angle;
        return res;
    }

    void operator*=(double const &val)
    {
        angle *= val;
    }

    void operator/=(double const &val)
    {
        angle /= val;
    }

    bool operator>(Angle const &obj)
    {
        return std::abs(angle) > std::abs(obj.angle);
    }

    bool operator>(double const &val)
    {
        return std::abs(angle) > val;
    }

    bool operator<(Angle const &obj)
    {
        return std::abs(angle) < std::abs(obj.angle);
    }

    bool operator<(double const &val)
    {
        return std::abs(angle) < val;
    }

    double getValue()
    {
        return angle;
    }
};

double abs(Angle obj)
{
    return std::abs(obj.getValue());
}