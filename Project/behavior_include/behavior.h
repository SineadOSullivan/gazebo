#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <statics.h>

namespace gazebo
{
    class Behavior
    {
    public:
        Behavior(double kGain);
    protected:
        double _kGain;

        const static double MAX_VELOCITY = 10.0d;
    private:

    };
}
#endif
