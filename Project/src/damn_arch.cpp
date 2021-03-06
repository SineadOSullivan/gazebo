#include "damn_arch.h"

using namespace gazebo;
using namespace std;

void DamnArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Initialize Architecture
    // Initialize this architecture
    if (this->initialize("DAMN", _parent, _sdf))
    {
        gzmsg << "Successfully loaded parameters!" << endl;

        // Set DAMN Parameters
        this->R.push_back(0.1);
        this->R.push_back(0.4);
        this->R.push_back(0.8);
        this->R.push_back(1.3);
        this->R.push_back(2.0);
        this->R.push_back(3.0);
        this->R.push_back(5.0);
        this->R.push_back(8.0);

        for( unsigned int i=0; i < this->_lidar->GetRangeCount(); i++)
        {
            this->T.push_back((this->_lidar->GetAngleMin()).Radian() + i*this->_lidar->GetAngleResolution() );
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DamnArch::OnUpdate, this, _1));
    }
}

// Called by the world update start event
void DamnArch::OnUpdate(const common::UpdateInfo & _info)
{
    // Wait for LIDAR
    while( this->_lidar->GetRange(0) == 0.0d ) return;

    // Update Position
    this->UpdatePosition(_info);
    // Check Metrics
    CheckMetrics();

    // Construct initial vote matrix
    std::vector< std::vector<double> > votes;
        votes.resize(this->R.size());
        for( int i = 0; i < R.size(); i++)
        {
            votes[i].resize(T.size());
        }

    // Run Behaviors
    this->_moveToGoal.moveToGoalDamn(this->_currentPosition, votes, this->R, this->T);
    this->_avoidBdry.avoidBoundaryDamn(this->_currentPosition, votes, this->R, this->T);
    this->_avoidObs.avoidObstaclesDamn(this->_lidar, votes, this->R, this->T);

    //  Find Maximum Vote
    double rS, tS;    // Selected R and Theta
    double maxVote = -100;  // current maximum vote

    for( unsigned int i = 0; i < this->R.size(); i++ )
    {
        for( unsigned int j = 0; j < this->T.size(); j++ )
        {
            // Add Some Noise
            votes[i][j] += this->_noise.getNoise();

            //Check Vote
            if( votes[i][j] > maxVote )
            {
                maxVote = votes[i][j];
                rS = this->R[i];
                tS = this->T[j];
            }
        }
    }


    // Convert R, Theta into velocity vector
    double spd = this->_maxSpeed*( rS / this->R.back() );
    math::Vector3 V = spd*math::Vector3(cos(tS), sin(tS), 0);


    // Set heading
    math::Pose p = this->_model->GetWorldPose();
    p.Set( p.pos, math::Vector3( 0, 0, 0 ) );
    this->_model->SetWorldPose(p);

    /**
    // Rotate Velocity to global frame
    p = p.GetInverse();
    p.Set(V, p.rot);
    p.RotatePositionAboutOrigin(p.rot);
    V = p.pos;
    **/

    // Apply a linear velocity to the model.
    this->_model->SetLinearVel(V);
    this->_model->SetAngularVel(math::Vector3(0,0,0));
}
