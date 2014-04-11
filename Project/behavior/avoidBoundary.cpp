#include "avoidBoundary.h"

using namespace gazebo;

AvoidBoundary::AvoidBoundary(double kBoundary)
    : Behavior(kBoundary)
{
}

math::Vector3 AvoidBoundary::avoidBoundary(sensors::RaySensorPtr lidar)
{
    // Method Variables
    double k = this->_kGain;
    math::Vector3 V = math::Vector3(0, 0, 0);


    // Return Result
    return V;
}
