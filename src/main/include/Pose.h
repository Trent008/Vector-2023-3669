#pragma once
#include "Vector.h"
#include "AngleOptimization.h"

// object that contains a vector for position, and a double for angle
class Pose
{
private:
    Vector position;
    double angle;
    Vector positionError;
    double angleError;
    AngleOptimizer optimize;

public:
    Pose(Vector position = {}, double angle = 0)
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
        res.angle = getDifference(angle, obj.angle);
        return res;
    }

    Pose operator+(Pose const &obj)
    {
        Pose res;
        res.position = position + obj.position;
        res.angle = angle + obj.angle;
        return res;
    }

    void operator*=(Vector &obj)
    {
        position *= obj.getX();
        angle *= obj.getY();
    }
    
    void operator*=(double constant)
    {
        position *= constant;
        angle *= constant;
    }

    void operator/=(double constant)
    {
        position /= constant;
        angle /= constant;
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
        return abs(position) + std::abs(angle) < constant;
    }

    void limit(Vector obj)
    {
        if (abs(position) > obj.getX())
        {
            position *= obj.getX() / abs(position);
        }
        if (std::abs(angle) > obj.getY())
        {
            angle *= obj.getY() / std::abs(angle);
        }
    }

    Vector getPosition()
    {
        return position;
    }

    double getAngle()
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
        angleError = getDifference(target.angle, angle);
        if (std::abs(angleError) > 2 * rate)
        {
            angle += angleError / std::abs(angleError) * rate;
        }
        else
        {
            angle += angleError / 2;
        }
    }

    void setPosition(Vector position) {
        this->position = position;
    }
};