#pragma once
#include "cmath"
#include "math.h"
#include "Angle.h"

class Vector: protected Angle
{
private:
    double x, y, x_new, y_new;

public:
    Vector(double x = 0, double y = 0)
    {
        this->x = x;
        this->y = y;
    }

    Vector operator+(Vector const &obj)
    {
        return Vector{x + obj.x, y + obj.y};
    }

    void operator+=(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    Vector operator-(Vector const &obj)
    {
        return Vector{x - obj.x, y - obj.y};
    }

    void operator-=(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    Vector operator*(double const &value)
    {
        return Vector{x*value, y*value};
    }

    Vector operator*(Angle &value)
    {
        return Vector{x*value.get(), y*value.get()};
    }

    void operator*=(double const &value)
    {
        x *= value;
        y *= value;
    }

    Vector operator/(double value)
    {
        return Vector{x/value, y/value};
    }

    void operator/=(double value)
    {
        x /= value;
        y /= value;
    }

    double magnitude()
    {
        return std::hypot(x, y);
    }

    double getX() {
        return x;
    }

    double getY() {
        return y;
    }

    void setX(double x)
    {
        this->x = x;
    }

    void setY(double y)
    {
        this->y = y;
    }
    
    void moveToward(Vector target, double rate)
    {
        target -= *this;
        if (target.magnitude() > 2 * rate)
        {
            *this += target / target.magnitude() * rate;
        }
        else
        {
            *this += target / 2;
        }
    }

    double getAngle()
    {
        return atan2(x, y) * 180 / M_PI;
    }

    void rotateCW(double angle)
    {
        *this = this->getRotatedCW(angle);
    }

    Vector getRotatedCW(double angle)
    {
        angle *= M_PI / 180;
        return Vector{x*cos(angle) + y*sin(angle), y*cos(angle) - x*sin(angle)};
    }

    void rotateCCW(double angle)
    {
        *this = this->getRotatedCCW(angle);
    }

    Vector getRotatedCCW(double angle)
    {
        angle *= M_PI / 180;
        return Vector{x*cos(angle) - y*sin(angle), y*cos(angle) + x*sin(angle)};
    }
};