#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>


namespace gazebo
{
    class Behavior
    {
    public:
        Behavior(double kGain);
    protected:
        double _kGain;
    private:

    };
}
#endif
