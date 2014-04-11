#include "avoidObstacles.h"

using namespace gazebo;

AvoidObstacles::AvoidObstacles(double kAvoid)
    : Behavior(kAvoid)
{
}

math::Vector3 AvoidObstacles::avoidObstacles(sensors::RaySensorPtr lidar)
{
    // Method Variables
    double k = this->_kGain;
    math::Vector3 V = math::Vector3(0, 0, 0);


    // Return Result
    return V;
}
