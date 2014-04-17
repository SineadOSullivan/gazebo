#include "moveToGoal.h"

using namespace gazebo;
using namespace std;

MoveToGoal::MoveToGoal()
    : Behavior(1.0d)
{
    this->_vGoal = math::Vector3(0.0d, 0.0d, 0.0d);
}

MoveToGoal::MoveToGoal(double kGoal, math::Vector3 vGoal)
    : Behavior(kGoal)
{
    this->_vGoal = vGoal;
}

math::Vector3 MoveToGoal::getGoal()
{
    return this->_vGoal;
}

math::Vector3 MoveToGoal::moveToGoalSubsumption(double maxSpeed, math::Vector3 currentPosition)
{
    // Check for goal position
    if (currentPosition == _vGoal)
        // Return no movement
        return math::Vector3(0.0d, 0.0d, 0.0d);
    // Not at the goal position
    else
    {
        // Get the vector from current position to goal
        math::Vector3 vToGoal = this->_vGoal - currentPosition;

        // Check to see if length is greater than max speed
        if (vToGoal.GetLength() > maxSpeed)
        {
            // Normalize the vector
            vToGoal = vToGoal.Normalize();

            // Scale to max speed
            vToGoal *= maxSpeed;
        }

        // Return the scaled vector
        return vToGoal;
    }
}

void MoveToGoal::moveToGoalDamn(math::Vector3 currentPosition, std::vector< std::vector<double> >& votes, std::vector<double>& R, std::vector<double>& T)
{    
    // Method Variables
    double rMax = R.back();

    // Compute Distance to Goal
    math::Vector3 toGoal = this->_vGoal - currentPosition;
    double distToGoal = toGoal.GetLength();
    double goalHead = atan2( toGoal[1], toGoal[0]);

    // Loop through Vote matrix
    for( unsigned int i = 0; i < R.size(); i++ )
    {
        for( unsigned int j = 0; j < T.size(); j++ )
        {
            if( distToGoal > rMax )
            {
                // Scale votes between 1 and -1 around heading
                votes[i][j] += R[i]*( 1 - 2.0*std::abs(goalHead - T[j])/M_PI )/rMax;
            }
            else if( R[i] <= distToGoal )
            {
                votes[i][j] += R[i]*( 1 - 2.0*std::abs(goalHead - T[j])/M_PI )/distToGoal;
            }
        }
    }
}

math::Vector3 MoveToGoal::moveToGoalMotorSchema(double maxSpeed, math::Vector3 currentPosition)
{
    // Method Variables
    double closeEnough = maxSpeed;
    double distToGoal;

    math::Vector3 V;
    math::Vector3 toGoal;

    // Compute Distance to Goal
    toGoal = this->_vGoal - currentPosition;
    distToGoal = toGoal.GetLength();

    // Compute Speed
    if( distToGoal < closeEnough )
    {
        V = toGoal;
    }
    else
    {
        V = maxSpeed*( toGoal/distToGoal );
    }

    // Return Result
    return V;
}
