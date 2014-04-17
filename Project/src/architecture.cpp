#include "architecture.h"

using namespace std;
namespace gazebo
{
    bool Architecture::initialize(std::string architectureName, physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Initializing the architecture" << endl;
        this->_archName = architectureName;
        this->_model = _parent;
        return (this->LoadMetrics() &&
                this->LoadLIDAR(_sdf) &&
                this->LoadGPS(_sdf) &&
                this->LoadIMU(_sdf) &&
                this->LoadParams(_sdf));
    }

    bool Architecture::LoadMetrics()
    {
        // Load start boundary
        physics::ModelPtr start = _model->GetWorld()->GetModel("start_boundary");
        this->_startBound = start->GetWorldPose().pos[1];
        gzmsg << "Start Bound: " << this->_startBound << endl;

        // Load goal boundary
        physics::ModelPtr goal = _model->GetWorld()->GetModel("goal_boundary");
        this->_goalBound = goal->GetWorldPose().pos[1];
        gzmsg << "Goal Bound: " << this->_goalBound << endl;

        // Load goal location
        physics::ModelPtr goalLocation = _model->GetWorld()->GetModel("goal_location");
        this->_goalLocation = goalLocation->GetWorldPose().pos;
        gzmsg << "Goal: " << this->_goalLocation << endl;

        // Set times to invalid
        _startTime.Set(-1.0d);
        _goalTime.Set(-1.0d);
        _executionTime.Set(-1.0d);
        _previousLocation = _model->GetWorldPose().pos;
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
                // Sensor Parameters
                gzmsg << "Lidar Range Count: " << _lidar->GetRangeCount() << endl;
                gzmsg << "Lidar Max Range: " << _lidar->GetRangeMax() << endl;
                gzmsg << "Lidar Min Range: " << _lidar->GetRangeMin() << endl;
                gzmsg << "Lidar Angle Resolution: " << _lidar->GetAngleResolution() << endl;
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

        if( _sdf->HasElement("output") )
        {
            gzmsg << "Output: " << _sdf->GetElement("output")->Get<std::string>() << endl;
            this->_outputLocation = _sdf->GetElement("output")->Get<std::string>();
        }
        else
        {
            gzwarn << "No output parameter defined, defaulting to 'Output.txt'" << endl;
            this->_outputLocation = "Output.txt";
        }

        // Load Goal Coordinate
        if( _sdf->HasElement("gain_goal") )
        {
            gzmsg << "Goal Gain: " << _sdf->GetElement("gain_goal")->Get<double>() << endl;
            this->_moveToGoal = MoveToGoal(_sdf->GetElement("gain_goal")->Get<double>(), this->_goalLocation);
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

    void Architecture::CheckMetrics()
    {
        // See if haven't clocked a start time yet, and have passed the start bound
        if (_startTime.sec == -1.0d && _currentPosition[1] >= _startBound)
        {
            // Clock the start time
            this->_startTime = this->_model->GetWorld()->GetRealTime();
            gzmsg << "Start Time: " << this->_startTime.sec << "." << this->_startTime.nsec << endl;
        }
        else if (_goalTime.sec == -1.0d && _currentPosition[1] >= _goalBound)
        {
            // Clock the goal time
            this->_goalTime = this->_model->GetWorld()->GetRealTime();
            gzmsg << "Stop Time: " << this->_goalTime.sec << "." << this->_goalTime.nsec << endl;
            // Calculate the execution time
            _executionTime = (this->_goalTime - this->_startTime);
            gzmsg << "Total Time: " << _executionTime.sec << "." << _executionTime.nsec << " seconds" << endl;
            gzmsg << "Distance Traveled: " << _distanceTraveled << endl;

            try
            {
                // Save out the data
                ofstream outputFile;
                outputFile.open(_outputLocation.c_str(), ios::out | ios::app);
                // Save the World Name
                outputFile << _model->GetWorld()->GetName() << " ";
                // Save the Architecture
                outputFile << _archName << " ";
                // Save the Execution Time
                outputFile << _executionTime.sec << "." << _executionTime.nsec << " ";
                // Save the Distance Traveled
                outputFile << _distanceTraveled << " ";
                // Save the Average speed
                outputFile << (_distanceTraveled / _executionTime.sec) << " ";
                // End the line
                outputFile << endl;
                // Close the file
                outputFile.close();
            }
            catch(int e){}
        }

        // Only track changes in position if we have started but not finished
        if (_startTime.sec != -1.0d && _goalTime.sec == -1.0d)
        {
            // Get the delta change
            _distanceTraveled += _model->GetWorldPose().pos.Distance(_previousLocation);

            // Save the current location as the previous
            _previousLocation = _model->GetWorldPose().pos;
        }
    }
}
