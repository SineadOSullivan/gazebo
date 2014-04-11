#include "architecture.h"

using namespace std;
namespace gazebo
{
    bool Architecture::initialize(sdf::ElementPtr _sdf)
    {
        gzmsg << "Initializing the architecture" << endl;
        return (this->LoadLIDAR(_sdf) &&
                this->LoadGPS(_sdf) &&
                this->LoadIMU(_sdf));
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
            sensors::SensorPtr sens = sensors::SensorManager::Instance()->GetSensor(sensorName);

            // Check if Sensor Exists
            if( !sens )
            {
                gzerr << "sensor by name ["
                    << sensorName
                    << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "RaySensor")
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
                gzerr << "sensor by name ["
                    << sensorName
                    << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "GpsSensor" )
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
                gzerr << "sensor by name ["
                      << sensorName
                      << "] not found in model\n";
                return false;
            }

            // Check if sensor is the correct type
            if( sens->GetType() == "ImuSensor" )
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
}
