#pragma once
#include "Vector.h"
#include "AngleChooser.h"

// object that contains a vector for position, and a double for angle
class Pose
{
private:
    Vector position;
    double angle;
    Vector positionError;
    double angleError;
    AngleChooser optimize;

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
        res.angle = optimize.getShortestDirection(obj.angle, angle);
        return res;
    }

    Pose operator+(Vector const &obj)
    {
        Pose res;
        res.position = position + obj;
        res.angle = angle;
    }

    void operator*=(Vector &obj)
    {
        position *= obj.getX();
        angle *= obj.getY();
    }

    Pose operator*(Vector obj)
    {
        Pose res;
        res.position = position * obj.getX();
        res.angle = angle * obj.getY();
        return res;
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

    void moveToward(Pose target, double positionSpeed, double angleSpeed)
    {
        positionError = target.position - position;
        if (abs(positionError) > 2 * positionSpeed)
        {
            position += positionError / abs(positionError) * positionSpeed;
        }
        else
        {
            position += positionError / 2;
        }
        angleError = optimize.getShortestDirection(angle, target.angle);
        if (std::abs(angleError) > 2 * angleSpeed)
        {
            angle += angleError / std::abs(angleError) * angleSpeed;
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

class ArmPose
{
private:
    Vector position;
    double wrist;
    double twist;
    bool suctionCupState;

public:
    ArmPose(Vector position = {-9, 11}, bool suctionCupState = false, double wrist = 0, double twist = 0)
    {
        this->position = position;
        this->wrist = wrist;
        this->twist = twist;
        this->suctionCupState = suctionCupState;
    }

    Vector getPosition()
    {
        return position;
    }
    
    double getWrist() {
        return wrist;
    }

    double getTwist() {
        return twist;
    }

    bool getSuctionCupState() {
        return suctionCupState;
    }

    ArmPose operator+(Vector const &obj)
    {
        ArmPose res;
        res.position = position + obj;
        res.wrist = wrist;
        res.twist = twist;
        res.suctionCupState = suctionCupState;
    }
};