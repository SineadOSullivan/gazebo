#ifndef MOTOR_SCHEMA_ARCH_H
#define MOTOR_SCHEMA_ARCH_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <stdio.h>
#include <avoidBoundary.h>
#include <avoidObstacles.h>
#include <moveToGoal.h>
#include <architecture.h>

namespace gazebo
{
    class MotorSchemaArch : public ModelPlugin, public Architecture
    {
    public:
        /**
         * Load the plugin
         * @brief Load
         * @param _parent
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr);
        /**
         * Load(namespace::class name, namespace::classname)
         * Called by the world update start event
         * @brief OnUpdate
         */
        void OnUpdate(const common::UpdateInfo &);
        /** & symbol means pass in reference instead of value
         * const tells it you can't modify info, constant, giving it the location and memory 
         
         Load and OnUpdate defines what you do, what are we doing every iteration, every time plugin is called*/
        void UpdateSensors();
        bool LoadParams(sdf::ElementPtr _sdf);

    private:
        // Model Parameters
        /**
         * GPS Coords of the Goal
         * @brief goalGPS
         */
        sdf::Vector3 goalGPS;
        /**
         * Maximum Model Speed
         * @brief maxSpeed
         */
        double maxSpeed;

        // Control Gains
        double kAvoid;
        double kGoal;
        double kBoundary;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MotorSchemaArch)
}
#endif
