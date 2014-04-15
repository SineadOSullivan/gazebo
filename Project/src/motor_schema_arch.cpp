#include "motor_schema_arch.h"

using namespace gazebo;

void MotorSchemaArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->_model = _parent;

    // Load Parameters
    if( this->LoadParams(_sdf) )
    {
        // Listen to the update event every iteration
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorSchemaArch::OnUpdate, this, _1));
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

    // Initialize the architecture
    return initialize(_sdf);
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
    MoveToGoal mtg(kGoal, math::Angle::HalfPi, math::Angle::Pi);
    V += ab.avoidBoundary(this->_lidar) +
         ao.avoidObstacles(this->_lidar) +
         mtg.moveToGoal(this->_gps);

    // Normalize Vector Result
    double spd = V.GetLength();
    if( spd > maxSpeed )
    {
        V = maxSpeed*V/spd;
    }

    // Apply vector velocity to the model
    this->_model->SetLinearVel(V);
}
