#include "moveToGoal.h"

using namespace gazebo;
using namespace std;

MoveToGoal::MoveToGoal()
    : Behavior(1.0d)
{
    this->_vGoal = math::Vector3(0.0d, 0.0d, 0.0d);
}

MoveToGoal::MoveToGoal(double kGoal, math::Vector3 vGoal)
    : Behavior(kGoal)
{
    this->_vGoal = vGoal;
}

math::Vector3 MoveToGoal::getGoal()
{
    return this->_vGoal;
}

math::Vector3 MoveToGoal::moveToGoalSubsumption(double maxSpeed, math::Vector3 currentPosition)
{
    // Check for goal position
    if (currentPosition == _vGoal)
        // Return no movement
        return math::Vector3(0.0d, 0.0d, 0.0d);
    // Not at the goal position
    else
    {
        // Get the vector from current position to goal
        math::Vector3 vToGoal = this->_vGoal - currentPosition;

        // Normalize the vector
        vToGoal = vToGoal.Normalize();

        // Scale to max speed
        vToGoal *= maxSpeed;

        // Return the scaled vector
        return vToGoal;
    }
}

math::Vector3 MoveToGoal::moveToGoalDamn()
{

}

math::Vector3 MoveToGoal::moveToGoalMotorSchema(double maxSpeed, math::Vector3 currentPosition)
{
    gzmsg << "MoveToGoal - Motor Schema" << endl;
    // Method Variables
    double closeEnough = maxSpeed;
    double distToGoal;

    math::Vector3 V;
    math::Vector3 toGoal;

    // Compute Distance to Goal
    toGoal = this->_vGoal - currentPosition;
    distToGoal = toGoal.GetLength();

    // Compute Speed
    if( distToGoal < closeEnough )
    {
        V = toGoal;
    }
    else
    {
        V = maxSpeed*( toGoal/distToGoal );
    }

    // Return Result
    return V;
}
