#include "avoidBoundary.h"

using namespace gazebo;
using namespace std;

AvoidBoundary::AvoidBoundary()
    : Behavior(1.0d)
{
}

AvoidBoundary::AvoidBoundary(double kBoundary)
    : Behavior(kBoundary)
{
}

math::Vector3 AvoidBoundary::avoidBoundarySubsumption(math::Vector2d minimum, math::Vector2d maximum, math::Vector3 currentPosition)
{
    const double boundaryThreshold = 1.0d;
    // Check for being within minimum threshold
    if ((currentPosition[0] - minimum[0]) < boundaryThreshold)
    {
        return (math::Vector3(1.0d, 0.0d, 0.0d));
    }
    else if ((currentPosition[1] - minimum[1]) < boundaryThreshold)
    {
        return (math::Vector3(0.0d, 1.0d, 0.0d));
    }
    // Check for being within maximum threshold
    else if ((maximum[0] - currentPosition[0]) < boundaryThreshold)
    {
        return (math::Vector3(-1.0d, 0.0d, 0.0d));
    }
    else if ((maximum[1] - currentPosition[1]) < boundaryThreshold)
    {
        return (math::Vector3(0.0d, -1.0d, 0.0d));
    }
    // Nothing to worry about
    else
        return math::Vector3(0.0d, 0.0d, 0.0d);
}

void AvoidBoundary::avoidBoundaryDamn(math::Vector3 currentPosition, std::vector< std::vector<double> >& votes, std::vector<double>& R, std::vector<double>& T)
{
    // Method Variables
    double width = 9.0;
    double x = currentPosition[0];

    // Loop through Vote matrix
    for( unsigned int i = 0; i < R.size(); i++ )
    {
        for( unsigned int j = 0; j < T.size(); j++ )
        {
            if( R[i]*cos(T[j]) >= 0.95*width-x || R[i]*cos(M_PI-T[j]) >= x-0.05*width )     // Boundary cross condition
            {
                votes[i][j] -= this->_kGain;
            }
            else if( R[i]*cos(T[j]) <= 0.8*width-x || R[i]*cos(M_PI-T[j]) <= x-0.2*width )  // Interior condition
            {
                votes[i][j] += 0.5;
            }         
        }
    }
}

math::Vector3 AvoidBoundary::avoidBoundaryMotorSchema(math::Vector3 currentPosition)
{
    /**Assume the boundary is given by a box oriented along the Y axis**/
    // Fixed Parameters
    const double thresh = 0.25;		// threshold fraction
    const double width = 9.0;		// X-width of the boundary

    // Method Variables
    double x = currentPosition[0];		// Current X position
    math::Vector3 V = math::Vector3(0,0,0);

    //Check boundary condition
    if( x <= 0 || x >= width )
    {
        V.Set( -10*_kGain*( x - 0.5*width ), 0, 0 );
    }
    else if( x < thresh*width )
    {
        V.Set( _kGain/pow(x-0.05*width,2), 0, 0 );
    }
    else if( x > (1-thresh)*width )
    {
        V.Set( -_kGain/pow(x-0.95*width,2), 0, 0 );
    }

    // Return Result
    return V;
}
