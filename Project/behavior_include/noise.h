#ifndef NOISE_H
#define NOISE_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "behavior.h"

namespace gazebo
{
    class Noise : public Behavior
    {
    public:
        Noise();
        Noise(double kNoise);
        double getNoise();
        math::Vector3 addNoise();
    private:
        math::Rand _rand;
    };
}
#endif
