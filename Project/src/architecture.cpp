#include "architecture.h"

using namespace std;
namespace gazebo
{
    bool Architecture::initialize(sdf::ElementPtr _sdf)
    {
        gzmsg << "Initializing the architecture" << endl;
        return (this->LoadLIDAR(_sdf) &&
                this->LoadGPS(_sdf) &&
                this->LoadIMU(_sdf) &&
                this->LoadParams(_sdf));
    }

    bool Architecture::LoadLIDAR(sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading LIDAR..." << endl;
        // Check for the Element
        if( !_sdf->HasElement("lidar") )
        {
            gzerr << "param [lidar] not found\n";
            return false;
        }
        else
        {
            // Get Sensor Name
            std::string sensorName = _sdf->GetElement("lidar")->Get<std::string>();

            // Get Pointer to the Sensor
            sensors::SensorPtr sens = sensors::SensorManager::Instance()->GetSensor("LIDAR");

            // Check if Sensor Exists
            if( !sens )
            {
                gzerr << "sensor by name [" << sensorName << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "ray")
            {
                // Dynamically cast the pointer
                this->_lidar = boost::dynamic_pointer_cast<sensors::RaySensor>(sens);
                return true;
            }
            else
            {
                gzerr << "Sensor by name ["<< sensorName << "] is wrong type ["<< sens->GetType() << "]\n";
                return false;
            }
        }
    }

    bool Architecture::LoadGPS(sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GPS..." << endl;
        // Check for the Element
        if( !_sdf->HasElement("gps") )
        {
            gzerr << "param [gps] not found\n";
            return false;
        }
        else
        {
            // Get Sensor Name
            std::string sensorName = _sdf->GetElement("gps")->Get<std::string>();

            // Get Pointer to the Sensor
            sensors::SensorPtr sens = sensors::SensorManager::Instance()->GetSensor(sensorName);

            // Check if Sensor Exists
            if( !sens )
            {
                gzerr << "sensor by name [" << sensorName << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "gps" )
            {
                // Dynamically cast the pointer
                this->_gps = boost::dynamic_pointer_cast<sensors::GpsSensor>(sens);
                return true;
            }
            else
            {
                gzerr << "Sensor by name ["<< sensorName << "] is wrong type ["<< sens->GetType() << "]\n";
                return false;
            }
        }
    }

    bool Architecture::LoadIMU(sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading IMU..." << endl;
        // Check for the Element
        if( !_sdf->HasElement("imu") )
        {
            gzerr << "param [imu] not found\n";
            return false;
        }
        else
        {
            // Get Sensor Name
            std::string sensorName = _sdf->GetElement("imu")->Get<std::string>();

            // Get Pointer to the Sensor
            sensors::SensorPtr sens = sensors::SensorManager::Instance()->GetSensor(sensorName);

            // Check if Sensor Exists
            if( !sens )
            {
                gzerr << "sensor by name [" << sensorName << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "imu" )
            {
                // Dynamically cast the pointer
                this->_imu = boost::dynamic_pointer_cast<sensors::ImuSensor>(sens);
                return true;
            }
            else
            {
                gzerr << "Sensor by name ["<< sensorName << "] is wrong type ["<< sens->GetType() << "]\n";
                return false;
            }
        }
    }

    bool Architecture::LoadParams(sdf::ElementPtr _sdf)
    {
        // Load Gains
        if( _sdf->HasElement("gain_avoid") )
        {
            gzmsg << "Avoidance Gain: " << _sdf->GetElement("gain_avoid")->Get<double>() << endl;
            this->_avoidObs = AvoidObstacles(_sdf->GetElement("gain_avoid")->Get<double>());
        }
        else
        {
            gzerr << "No gain_avoid parameter defined" << endl;
            return false;
        }

        if( _sdf->HasElement("gain_boundary") )
        {
            gzmsg << "Boundary Gain: " << _sdf->GetElement("gain_boundary")->Get<double>() << endl;
            this->_avoidBdry = AvoidBoundary(_sdf->GetElement("gain_boundary")->Get<double>());
        }
        else
        {
            gzerr << "No gain_boundary parameter defined" << endl;
            return false;
        }

        if( _sdf->HasElement("gain_noise") )
        {
            gzmsg << "Noise Gain: " << _sdf->GetElement("gain_noise")->Get<double>() << endl;
            this->_noise = Noise(_sdf->GetElement("gain_noise")->Get<double>());
        }
        else
        {
            gzerr << "No gain_noise parameter defined" << endl;
            return false;
        }

        // Load Goal Coordinate
        if( _sdf->HasElement("goal") && _sdf->HasElement("gain_goal") )
        {
            gzmsg << "Goal Gain: " << _sdf->GetElement("gain_goal")->Get<double>() << endl;
            gzmsg << "Goal: " << _sdf->GetElement("goal")->Get<math::Vector3>() << endl;
            this->_moveToGoal = MoveToGoal(_sdf->GetElement("gain_goal")->Get<double>(), _sdf->GetElement("goal")->Get<math::Vector3>());
        }
        else
        {
            gzerr << "No goal and gain_goal parameter defined" << endl;
            return false;
        }

        // Load Maximum Speed
        if( _sdf->HasElement("max_speed") )
        {
            gzmsg << "Max Speed: " << _sdf->GetElement("max_speed")->Get<double>() << endl;
            _maxSpeed = _sdf->GetElement("max_speed")->Get<double>();
        }
        else
        {
            gzerr << "No max_speed parameter defined" << endl;
            return false;
        }
        return true;
    }

    void Architecture::UpdatePosition(const common::UpdateInfo & _info)
    {
        // Update Position from GPS.  Could eventually implement Kalman filter
        // Method Variables
        const double earthRad = 6371009; //Radius of the earth in meters
        // Convert GPS to x,y,z
        math::Angle lat = this->_gps->GetLatitude();
        math::Angle lon = this->_gps->GetLongitude();

        // Update the position
        _currentPosition = math::Vector3( earthRad*lon.Radian(),
                                          earthRad*lat.Radian(),
                                          this->_gps->GetAltitude() );
    }
}
