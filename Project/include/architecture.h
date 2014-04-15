#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <avoidBoundary.h>
#include <avoidObstacles.h>
#include <moveToGoal.h>
#include <noise.h>
#include <statics.h>

namespace gazebo
{
    class Architecture
    {
    public:
        bool initialize(sdf::ElementPtr _sdf);
    protected:
        /**
         * @brief LoadIMU
         * @param _sdf
         * @return
         */
        bool LoadIMU(sdf::ElementPtr _sdf);
        /**
         * @brief LoadGPS
         * @param _sdf
         * @return
         */
        bool LoadGPS(sdf::ElementPtr _sdf);
        /**
         * @brief LoadLIDAR
         * @param _sdf
         * @return
         */
        bool LoadLIDAR(sdf::ElementPtr _sdf);
        /**
         * @brief LoadParams
         * @param _sdf
         * @return
         */
        bool LoadParams(sdf::ElementPtr _sdf);

        /**
         * @brief UpdatePosition
         * @param _info
         */
        void UpdatePosition(const common::UpdateInfo & _info);

        /**
         * Pointer to the model
         * @brief model
         */
        physics::ModelPtr _model;
        /**
         * Pointer to the update event connection
         * @brief updateConnection
         */
        event::ConnectionPtr _updateConnection;
        /**
         * Pointer to the LIDAR Sensor
         * @brief lidar
         */
        sensors::RaySensorPtr _lidar;
        /**
         * Pointer to the GPS Sensor
         * @brief gps
         */
        sensors::GpsSensorPtr _gps;
        /**
         * Pointer to the IMU Sensor
         * @brief imu
         */
        sensors::ImuSensorPtr _imu;
        /**
         * @brief _avoidBdry
         */
        gazebo::AvoidBoundary _avoidBdry;
        /**
         * @brief _avoidObs
         */
        gazebo::AvoidObstacles _avoidObs;
        /**
         * @brief _moveToGoal
         */
        gazebo::MoveToGoal _moveToGoal;
        /**
         * @brief _noise
         */
        Noise _noise;

    private:

    };
}
#endif
