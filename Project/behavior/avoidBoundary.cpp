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

math::Vector3 AvoidBoundary::avoidBoundarySubsumption()
{

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
