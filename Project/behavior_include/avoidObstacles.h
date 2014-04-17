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
        AvoidObstacles();
        AvoidObstacles(double kAvoid);

        math::Vector3 avoidObstaclesSubsumption(sensors::RaySensorPtr lidar);
        void avoidObstaclesDamn(sensors::RaySensorPtr lidar, std::vector< std::vector<double> > & votes, std::vector<double>& R, std::vector<double>& T);
        math::Vector3 avoidObstaclesMotorSchema(physics::ModelPtr model, sensors::RaySensorPtr lidar, double kOpen);
    private:

    };
}
#endif
