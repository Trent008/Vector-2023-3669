#pragma once
#include "cmath"
#include "math.h"
#include "Angle.h"

class Vector
{
private:
    double x, y, x_new, y_new;

public:
    /**
     * vector.x = x
     * vector.y = y
     **/
    Vector(double x = 0, double y = 0)
    {
        this->x = x;
        this->y = y;
    }

    // Method to rotate the vector by a given angle
    void rotate(double angle)
    {
        x_new = x * cos(-angle * M_PI / 180) - y * sin(-angle * M_PI / 180);
        y_new = x * sin(-angle * M_PI / 180) + y * cos(-angle * M_PI / 180);
        x = x_new;
        y = y_new;
    }

    Vector getRotated(double angle)
    {
        x_new = x * cos(-angle * M_PI / 180) - y * sin(-angle * M_PI / 180);
        y_new = x * sin(-angle * M_PI / 180) + y * cos(-angle * M_PI / 180);
        return Vector{x_new, y_new};
    }

    void operator=(Vector const &obj)
    {
        x = obj.x;
        y = obj.y;
    }

    Vector operator+(Vector const &obj)
    {
        Vector res;
        res.x = x + obj.x;
        res.y = y + obj.y;
        return res;
    }

    Vector operator+(Angle obj)
    {
        Vector res;
        res.x = x * cos(-obj.getValue() * M_PI / 180) - y * sin(-obj.getValue() * M_PI / 180);
        res.y = x * sin(-obj.getValue() * M_PI / 180) + y * cos(-obj.getValue() * M_PI / 180);
        return res;
    }

    Vector operator-(Vector const &obj)
    {
        Vector res;
        res.x = x - obj.x;
        res.y = y - obj.y;
        return res;
    }

    Vector operator-(Angle obj)
    {
        Vector res;
        res.x = x * cos(obj.getValue() * M_PI / 180) - y * sin(obj.getValue() * M_PI / 180);
        res.y = x * sin(obj.getValue() * M_PI / 180) + y * cos(obj.getValue() * M_PI / 180);
        return res;
    }

    Vector operator*(double const &val)
    {
        Vector res;
        res.x = x * val;
        res.y = y * val;
        return res;
    }

    Vector operator/(double const &val)
    {
        Vector res;
        res.x = x / val;
        res.y = y / val;
        return res;
    }

    void operator*=(double const &val)
    {
        x *= val;
        y *= val;
    }

    void operator/=(double const &val)
    {
        x /= val;
        y /= val;
    }

    void operator+=(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    void operator+=(Angle const &obj)
    {
        *this = *this + obj;
    }

    void operator-=(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    void operator=(Angle const &obj)
    {
        *this = *this - obj;
    }

    bool operator>(Vector const &obj)
    {
        return hypot(x, y) > hypot(obj.x, obj.y);
    }

    bool operator>(double const &val)
    {
        return hypot(x, y) > val;
    }

    bool operator<(Vector const &obj)
    {
        return hypot(x, y) < hypot(obj.x, obj.y);
    }

    bool operator<(double const &val)
    {
        return hypot(x, y) < val;
    }

    double getX() { return x; }

    double getY() { return y; }

    void setX(double x)
    {
        this->x = x;
    }

    void setY(double y)
    {
        this->y = y;
    }
};

double abs(Vector obj)
{
    return hypot(obj.getX(), obj.getY());
}

Angle angle(Vector &obj)
{
    return Angle{atan2(obj.getX(), obj.getY()) * 180 / M_PI};
}