#pragma once
#include "Vector.h"

// object that contains a vector for position, and a double for angle
class Pose
{
private:
    Vector position;
    Angle angle;
    Vector positionError;
    Angle angleError;

public:
    Pose(Vector position = {}, Angle angle = 0)
    {
        this->position = position;
        this->angle = angle;
    }

    void operator=(Pose const &obj)
    {
        position = obj.position;
        angle = obj.angle;
    }

    Pose operator-(Pose const &obj)
    {
        Pose res;
        res.position = position - obj.position;
        res.angle = angle - obj.angle;
        return res;
    }

    Pose operator-(Vector const &obj)
    {
        Pose res;
        res.position = position - obj;
        return res;
    }

    Pose operator+(Pose const &obj)
    {
        Pose res;
        res.position = position + obj.position;
        res.angle = angle + obj.angle;
        return res;
    }

    Pose operator+(Vector const &obj)
    {
        Pose res;
        res.position = position + obj;
        return res;
    }

    void operator*=(Vector &obj)
    {
        position *= obj.getX();
        angle *= obj.getY();
    }
    
    void operator*=(double const &val)
    {
        position *= val;
        angle *= val;
    }

    void operator/=(double const &val)
    {
        position /= val;
        angle /= val;
    }

    Pose operator*(Vector obj)
    {
        Pose res;
        res.position = position * obj.getX();
        res.angle = angle * obj.getY();
        return res;
    }
    
    Pose operator/(double constant)
    {
        Pose res;
        res.position = position / constant;
        res.angle = angle / constant;
        return res;
    }

    bool operator<(double constant)
    {
        return abs(position) + abs(angle) < constant;
    }

    void limit(Vector obj)
    {
        if (abs(position) > obj.getX())
        {
            position *= obj.getX() / abs(position);
        }
        if (abs(angle) > obj.getY())
        {
            angle *= obj.getY() / abs(angle);
        }
    }

    Vector getPosition()
    {
        return position;
    }

    Angle getAngle()
    {
        return angle;
    }

    void moveToward(Pose target, double rate)
    {
        positionError = target.position - position;
        if (abs(positionError) > 2 * rate)
        {
            position += positionError / abs(positionError) * rate;
        }
        else
        {
            position += positionError / 2;
        }
        angleError = target.angle - angle;
        if (abs(angleError) > 2 * rate)
        {
            angle += angleError / abs(angleError) * rate;
        }
        else
        {
            angle += angleError / 2;
        }
    }

    void setPosition(Vector position) {
        this->position = position;
    }

    // rotates about the origin by the given angle
    void rotate(Angle angle)
    {
        this->angle += angle;
        position += angle;
    }

    // returns this Pose rotated about (0, 0) by the given angle
    Pose getRotated(Angle angle)
    {
        Pose res;
        res.angle = this->angle + angle;
        res.position += angle;
        return res;
    }
};