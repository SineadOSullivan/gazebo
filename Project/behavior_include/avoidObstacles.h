#ifndef AVOID_OBSTACLES_H
#define AVOID_OBSTACLES_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "behavior.h"

namespace gazebo
{
    class AvoidObstacles : public Behavior
    {
    public:
        AvoidObstacles(double kAvoid);
        math::Vector3 avoidObstacles(sensors::RaySensorPtr lidar);
    private:

    };
}
#endif
