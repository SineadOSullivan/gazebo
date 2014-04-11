#include "moveToGoal.h"

using namespace gazebo;

MoveToGoal::MoveToGoal(double kGoal)
    : Behavior(kGoal)
{
}

math::Vector3 MoveToGoal::moveToGoal(sensors::GpsSensorPtr gps)
{
    // Method Variables
    double k = this->_kGain;
    math::Vector3 V = math::Vector3(0, 0, 0);


    // Return Result
    return V;
}
