#pragma once
#include "Vector.h"
#include "Angle.h"

class Pose
{
protected:
    Vector vector;
    Angle angle;
public:
    Pose(Vector vector = Vector{}, Angle angle = Angle{})
    {
        this->vector = vector;
        this->angle = angle;
    }

    void operator+=(Pose obj)
    {
        vector += obj.vector;
        angle += obj.angle;
    }

    Pose operator+(Pose obj)
    {
        Pose res;
        res += obj;
        return res;
    }

    void operator-=(Pose pose)
    {
        vector -= pose.vector;
        angle -= pose.angle;
    }

    Pose operator-(Pose obj)
    {
        Pose res;
        res -= obj;
        return res;
    }

    void operator*=(Vector obj)
    {
        this->vector *= vector.getX();
        this->angle *= vector.getY();
    }

    void operator/=(double divisor)
    {
        vector /= divisor;
        angle /= divisor;
    }

    void moveToward(Pose pose, double rate) {
        vector.moveToward(pose.vector, rate);
        angle.moveToward(pose.angle, rate);
    }

    double magnitude()
    {
        return vector.magnitude() + angle.magnitude();
    }

    Vector getVector()
    {
        return vector;
    }

    Angle getAngle()
    {
        return angle;
    }

    void limit(Vector lim)
    {
        if (vector.magnitude() > lim.getX())
        {
            vector *= lim.getX() / vector.magnitude();
        }
        if (angle.magnitude() > lim.getY())
        {
            angle *= lim.getY() / angle.magnitude();
        }
    }

    void rotateVectorCW(double angle)
    {
        vector.rotateCW(angle);
    }

    void rotateVectorCCW(double angle)
    {
        vector.rotateCCW(angle);
    }

    Pose getRotatedCW(Angle angle)
    {
        return Pose{vector.getRotatedCW(angle.get()), this->angle};
    }

    Pose getRotatedCCW(Angle angle)
    {
        return Pose{vector.getRotatedCCW(angle.get()), this->angle};
    }
};