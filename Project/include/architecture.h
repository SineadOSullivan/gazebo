#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H

//#define LOGGING

#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <avoidBoundary.h>
#include <avoidObstacles.h>
#include <moveToGoal.h>
#include <noise.h>

namespace gazebo
{
    class Architecture
    {
    public:
        bool initialize(std::string architectureName, physics::ModelPtr _parent, sdf::ElementPtr _sdf);
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

        bool LoadMetrics();

        /**
         * @brief UpdatePosition
         * @param _info
         */
        void UpdatePosition(const common::UpdateInfo & _info);

        void CheckMetrics();

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

        math::Vector3 _currentPosition;
        double _maxSpeed;

    private:
        std::string _archName;
        std::string _outputLocation;
        double _startBound;
        double _goalBound;
        common::Time _startTime;
        common::Time _goalTime;
        common::Time _executionTime;
        math::Vector3 _goalLocation;
        math::Vector3 _previousLocation;
        double _distanceTraveled;
    };
}
#endif
