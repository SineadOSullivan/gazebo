#include "moveToGoal.h"

using namespace gazebo;

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

math::Vector3 MoveToGoal::moveToGoalSubsumption()
{
    // Check for goal position
    if (Statics::CURRENT_POS == _vGoal)
        // Return no movement
        return math::Vector3(0.0d, 0.0d, 0.0d);
    // Not at the goal position
    else
    {
        // Get the vector from current position to goal
        math::Vector3 vToGoal = this->_vGoal - Statics::CURRENT_POS;

        // Normalize the vector
        vToGoal = vToGoal.Normalize();

        // Scale to max speed
        vToGoal *= Statics::MAX_SPEED;

        // Return the scaled vector
        return vToGoal;
    }
}

math::Vector3 MoveToGoal::moveToGoalDamn()
{

}

math::Vector3 MoveToGoal::moveToGoalMotorSchema()
{
    // Method Variables
    double closeEnough = Statics::MAX_SPEED;
    double distToGoal;

    math::Vector3 V;
    math::Vector3 toGoal;

    // Compute Distance to Goal
    toGoal = this->_vGoal - Statics::CURRENT_POS;
    distToGoal = toGoal.GetLength();

    // Compute Speed
    if( distToGoal < closeEnough )
    {
        V = toGoal;
    }
    else
    {
        V = Statics::MAX_SPEED*( toGoal/distToGoal );
    }

    // Return Result
    return V;
}
