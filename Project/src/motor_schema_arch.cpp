#include "motor_schema_arch.h"

using namespace gazebo;
using namespace std;

void MotorSchemaArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->_model = _parent;

    // Load Parameters
    if( this->initialize("MotorSchema", _parent, _sdf) )
    {
        if( _sdf->HasElement("gain_open") )
        {
            gzmsg << "Open Gain: " << _sdf->GetElement("gain_open")->Get<double>() << endl;
            this->kOpen = _sdf->GetElement("gain_open")->Get<double>();
        }
        else
        {
            gzwarn << "No gain_open parameter defined, defaulting to 1.0d" << endl;
            this->kOpen = 1.0d;
        }
        // Listen to the update event every iteration
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorSchemaArch::OnUpdate, this, _1));
    }
}

// Called by the world update start event
void MotorSchemaArch::OnUpdate(const common::UpdateInfo & _info)
{
    // Check for Lidar Functionality
    if (_lidar->GetRange(0) == 0.0d)
        return;

    // Method Variables
    math::Vector3 V = math::Vector3(0, 0, 0);

    // Update the position
    UpdatePosition(_info);
    // Check Metrics
    CheckMetrics();

    // Run Behaviors
    V += _avoidBdry.avoidBoundaryMotorSchema(_currentPosition) +
         _avoidObs.avoidObstaclesMotorSchema(_model, this->_lidar, kOpen) +
         _moveToGoal.moveToGoalMotorSchema(_maxSpeed, _currentPosition) +
         _noise.addNoise();

    // Normalize Vector Result
    double spd = V.GetLength();
    if( spd > _maxSpeed )
    {
        V = _maxSpeed*V/spd;
    }

    // Apply vector velocity to the model
    this->_model->SetLinearVel(V);
    this->_model->SetAngularVel( math::Vector3(0,0,0) );
}
