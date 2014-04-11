#include "motor_schema_arch.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MotorSchemaArch)

void MotorSchemaArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _parent;

    // Load Parameters
    if( this->LoadParams(_sdf) )
    {
        // Listen to the update event every iteration
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorSchemaArch::OnUpdate, this, _1));
    }
}

bool MotorSchemaArch::LoadParams(sdf::ElementPtr _sdf)
{
    //Load Gains
    if( _sdf->HasElement("gain_avoid") )
    {
        this->kAvoid = _sdf->GetElement("gain_avoid")->Get<double>();
    }
    else
        this->kAvoid = 1.0d;
    if( _sdf->HasElement("gain_goal") )
    {
        this->kGoal = _sdf->GetElement("gain_goal")->Get<double>();
    }
    else
        this->kGoal = 1.0d;
    if( _sdf->HasElement("gain_boundary") )
    {
        this->kBoundary = _sdf->GetElement("gain_boundary")->Get<double>();
    }
    else
        this->kBoundary = 1.0d;

    // Load Goal Coordinate
    if( _sdf->HasElement("goal") )
    {
        this->goalGPS = _sdf->GetElement("goal")->Get<sdf::Vector3>();
    }
    else
        this->goalGPS = sdf::Vector3(0, 0, 0);

    // Load Maximum Speed
    if( _sdf->HasElement("max_speed") )
    {
        this->maxSpeed = _sdf->GetElement("max_speed")->Get<double>();
    }
    else
        this->maxSpeed = 1.0d;

    // Load Sensors
    return( LoadLIDAR(_sdf, this->lidar) &&
            LoadGPS(_sdf, this->gps) &&
            LoadIMU(_sdf, this->imu));
}

bool MotorSchemaArch::LoadLIDAR(sdf::ElementPtr _sdf, sensors::RaySensorPtr & _sensor)
{
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
        if( _sensor->GetType() == sens->GetType() )
        {
            // Dynamically cast the pointer
            _sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(sens);
            return true;
        }
        else
        {
            gzerr << "Sensor by name ["<< sensorName << "] is wrong type\n";
            return false;
        }
    }
}

bool MotorSchemaArch::LoadGPS(sdf::ElementPtr _sdf, sensors::GpsSensorPtr & _sensor)
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
        if( _sensor->GetType() == sens->GetType() )
        {
            // Dynamically cast the pointer
            _sensor = boost::dynamic_pointer_cast<sensors::GpsSensor>(sens);
            return true;
        }
        else
        {
            gzerr << "Sensor by name ["<< sensorName << "] is wrong type\n";
            return false;
        }
    }
}

bool MotorSchemaArch::LoadIMU(sdf::ElementPtr _sdf, sensors::ImuSensorPtr & _sensor)
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
        if( _sensor->GetType() == sens->GetType() )
        {
            // Dynamically cast the pointer
            _sensor = boost::dynamic_pointer_cast<sensors::ImuSensor>(sens);
            return true;
        }
        else
        {
            gzerr << "Sensor by name ["<< sensorName << "] is wrong type\n";
            return false;
        }
    }
}

void MotorSchemaArch::UpdateSensors()
{

}

// Called by the world update start event
void MotorSchemaArch::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Method Variables
    math::Vector3 V = math::Vector3(0, 0, 0);

    // Update Sensor Data
    UpdateSensors();

    // Run Behaviors
    AvoidBoundary ab(kBoundary);
    AvoidObstacles ao(kAvoid);
    MoveToGoal mtg(kGoal);
    V += ab.avoidBoundary(this->lidar) + 
         ao.avoidObstacles(this->lidar) +
	     mtg.moveToGoal(this->gps);

    // Normalize Vector Result
    double spd = V.GetLength();
    if( spd > maxSpeed )
    {
        V = maxSpeed*V/spd;
    }

    // Apply vector velocity to the model
    this->model->SetLinearVel(V);
}
