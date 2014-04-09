#ifndef AVOID_BOUNDARY_H
#define AVOID_BOUNDARY_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "behavior.h"

namespace gazebo
{
    class AvoidBoundary : public Behavior
    {
    public:
        AvoidBoundary();
    private:

    };
}
#endif
