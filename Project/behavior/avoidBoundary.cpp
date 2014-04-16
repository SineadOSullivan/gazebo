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

math::Vector3 AvoidBoundary::avoidBoundaryDamn()
{

}

math::Vector3 AvoidBoundary::avoidBoundaryMotorSchema(math::Vector3 currentPosition)
{
    gzmsg << "Avoid Boundary - Motor Schema" << endl;
    /**Assume the boundary is given by a box oriented along the Y axis**/
    // Method Variables
    double width = 9.0;                     // X-width of the boundary
    double x = currentPosition[0];     // Current X position

    math::Vector3 V = math::Vector3(0,0,0);

    //Check boundary condition
    if( x <= 0 || x >= width )
    {
        V.Set( -_kGain*( currentPosition[0] - 0.5*width ), 0, 0 );
    }
    else if( x < width/2 )
    {
        V.Set( _kGain/x, 0, 0 );
    }
    else
    {
        V.Set( -_kGain/(x-width), 0, 0 );
    }

    // Return Result
    return V;
}
