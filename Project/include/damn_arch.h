#ifndef DAMN_ARCH_H
#define DAMN_ARCH_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
    class DamnArch : public ModelPlugin
    {
    public:
        /**
         * Load the plugin
         * @brief Load
         * @param _parent
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr);
        /**
         * Called by the world update start event
         * @brief OnUpdate
         */
        void OnUpdate(const common::UpdateInfo &);
    private:
        /**
         * Pointer to the model
         * @brief model
         */
        physics::ModelPtr model;
        /**
         * Pointer to the update event connection
         * @brief updateConnection
         */
        event::ConnectionPtr updateConnection;
    };
}
#endif
