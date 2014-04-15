#ifndef SUBSUMPTION_ARCH_H
#define SUBSUMPTION_ARCH_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include "architecture.h"

namespace gazebo
{
    class SubsumptionArch : public ModelPlugin, public Architecture
    {
    public:
        /**
         * Load the plugin
         * @brief Load
         * @param _parent
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        /**
         * Called by the world update start event
         * @brief OnUpdate
         */
        void OnUpdate(const common::UpdateInfo &);
    private:

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(SubsumptionArch)
}
#endif
