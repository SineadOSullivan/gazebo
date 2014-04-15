#include "motor_schema_arch.h"

using namespace gazebo;
using namespace std;

void MotorSchemaArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->_model = _parent;

    // Load Parameters
    if( this->initialize(_sdf) )
    {
        // Listen to the update event every iteration
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorSchemaArch::OnUpdate, this, _1));
    }
}

// Called by the world update start event
void MotorSchemaArch::OnUpdate(const common::UpdateInfo & _info)
{
    gzmsg << "Motor Schema OnUpdate" << endl;
    // Method Variables
    math::Vector3 V = math::Vector3(0, 0, 0);

    gzmsg << "Calling Update Position" << endl;
    // Update position
    UpdatePosition(_info);

    gzmsg << "Running the behaviors" <<endl;
    // Run Behaviors
    V += _avoidBdry.avoidBoundaryMotorSchema(_currentPosition) +
         _avoidObs.avoidObstaclesMotorSchema(this->_lidar) +
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
}
