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
        AvoidBoundary(double kBoundary);
        math::Vector3 avoidBoundarySubsumption();
        void avoidBoundaryDamn(math::Vector3 currentPosition, std::vector< std::vector<double> >& votes, std::vector<double>& R, std::vector<double>& T);
        math::Vector3 avoidBoundaryMotorSchema(math::Vector3 currentPosition);
    private:

    };
}
#endif
