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
    private:

    };
}
#endif
