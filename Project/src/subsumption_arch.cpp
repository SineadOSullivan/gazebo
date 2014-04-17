#include "subsumption_arch.h"

using namespace std;

namespace gazebo
{
    void SubsumptionArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Subsumption Architecture plugin..." << endl;

        // Initialize this architecture
        if (this->initialize("Subsumption", _parent, _sdf))
        {
            gzmsg << "Successfully loaded parameters!" << endl;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SubsumptionArch::OnUpdate, this, _1));
        }
    }

    // Called by the world update start event
    void SubsumptionArch::OnUpdate(const common::UpdateInfo & _info)
    {
        // Check for Lidar Functionality
        if (_lidar->GetRange(0) == 0.0d)
            return;
        // Update the position
        UpdatePosition(_info);
        // Check Metrics
        CheckMetrics();

        // No Op
        math::Vector3 v0(0.0d, 0.0d, 0.0d);
        math::Vector3 vOutput;

        // Get the output for Avoid Obstacle behavior
        math::Vector3 v1 = _avoidObs.avoidObstaclesSubsumption(_lidar);
        // Get the output for Avoid Boundary behavior
        math::Vector3 v2 = _avoidBdry.avoidBoundarySubsumption(math::Vector2d(0.0d, -50.0d), math::Vector2d(9.0d, 150.0d), _currentPosition);
        // Get the output for Avoid Boundary behavior
        math::Vector3 v3 = _moveToGoal.moveToGoalSubsumption(_maxSpeed, _currentPosition);
        // Check for valid Avoid Obstacle output
        if (v1 != v0)
        {
#ifdef LOGGING
            gzmsg << "Set the avoid obstacle velocity: " << v1 << endl;
#endif
            vOutput = v1;
        }
        // Check for valid Avoid Boundary output
        else if (v2 != v0)
        {
#ifdef LOGGING
            gzmsg << "Set the avoid boundary velocity: " << v2 << endl;
#endif
            vOutput = v2;
        }
        // Check for valid Move To Goal output
        else if (v3 != v0)
        {
#ifdef LOGGING
            gzmsg << "Set the move to goal velocity: " << v3 << endl;
#endif
            vOutput = v3;
        }
        // Fall through, no good behavior outputs
        else
        {
#ifdef LOGGING
            // Fall back to no op at this point
            gzmsg << "Fall back to no op velocity: " << v0 << endl;
#endif
            vOutput = v0;
        }
#ifdef LOGGING
        gzmsg << "Move from [" << _currentPosition << "] to [" << _moveToGoal.getGoal() << "] = [" << vOutput << "]" << endl;
#endif
        // Set the output velocity velocity
        this->_model->SetLinearVel(vOutput);

    }
}
