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
        if (positionError > 2 * rate)
        {
            position += positionError / abs(positionError) * rate;
        }
        else
        {
            position += positionError / 2;
        }
        angleError = target.angle - angle;
        if (angleError > 2 * rate)
        {
            angle += angleError / abs(angleError) * rate;
        }
        else
        {
            angle += angleError / 2;
        }
    }

    bool operator<(double constant)
    {
        return abs(position) + abs(angle) < constant;
    }

    Pose getRotatedCCW(Angle const &obj)
    {
        return Pose{position - obj, angle};
    }

    Pose getRotatedCW(Angle const &obj)
    {
        return Pose{position + obj, angle};
    }

    void setPosition(Vector position) {
        this->position = position;
    }
};