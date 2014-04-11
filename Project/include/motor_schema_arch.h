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

namespace gazebo
{
    class MotorSchemaArch : public ModelPlugin
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
        void UpdateSensors();
        //bool LoadSensor(sdf::ElementPtr _sdf, const std::string element, sensors::Sensor & _sensor);
        bool LoadParams(sdf::ElementPtr _sdf);

        bool LoadIMU(sdf::ElementPtr _sdf, sensors::ImuSensorPtr & _sensor);
        bool LoadGPS(sdf::ElementPtr _sdf, sensors::GpsSensorPtr & _sensor);
        bool LoadLIDAR(sdf::ElementPtr _sdf, sensors::RaySensorPtr & _sensor);

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
        /**
         * Pointer to the LIDAR Sensor
         * @brief lidar
         */
        sensors::RaySensorPtr lidar;
        /**
         * Pointer to the GPS Sensor
         * @brief gps
         */
        sensors::GpsSensorPtr gps;
        /**
         * Pointer to the IMU Sensor
         * @brief imu
         */
        sensors::ImuSensorPtr imu;

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
}
#endif
