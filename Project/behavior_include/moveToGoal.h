#ifndef MOVE_TO_GOAL_H
#define MOVE_TO_GOAL_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "behavior.h"

namespace gazebo
{
    class MoveToGoal : public Behavior
    {
    public:
        MoveToGoal();
        MoveToGoal(double kGoal, math::Vector3 vGoal);
        math::Vector3 moveToGoalSubsumption(double maxSpeed, math::Vector3 currentPosition);
        void moveToGoalDamn(math::Vector3 currentPosition, std::vector< std::vector<double> > & votes, std::vector<double> R, std::vector<double> T);
        math::Vector3 moveToGoalMotorSchema(double maxSpeed, math::Vector3 currentPosition);

        math::Vector3 getGoal();
    private:
        math::Vector3 _vGoal;
    };
}
#endif
