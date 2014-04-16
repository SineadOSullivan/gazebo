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
    return math::Vector3(0.0d, 0.0d, 0.0d);
}

void AvoidBoundary::avoidBoundaryDamn(math::Vector3 currentPosition, std::vector< std::vector<double> > & votes, std::vector<double> R, std::vector<double> T)
{
    // Method Variables
    double width = 9.0;
    double x = currentPosition[0];

    // Loop through Vote matrix
    for( unsigned int i = 0; i < R.size(); i++ )
    {
        for( unsigned int j = 0; j < T.size(); j++ )
        {
            if( R[i]*cos(T[i]) >= width-x )
            {
                votes[i][j] += -this->_kGain;
            }
            else if( R[i]*cos(M_PI-T[i]) >= x )
            {
                votes[i][j] += -this->_kGain;
            }
        }
    }
}

math::Vector3 AvoidBoundary::avoidBoundaryMotorSchema(math::Vector3 currentPosition)
{
    gzmsg << "Avoid Boundary - Motor Schema" << endl;
    /**Assume the boundary is given by a box oriented along the Y axis**/
    // Method Variables
    double width = 9.0;                     // X-width of the boundary
    double x = currentPosition[0];          // Current X position

    math::Vector3 V = math::Vector3(0,0,0);

    //Check boundary condition
    if( x <= 0 || x >= width )
    {
        V.Set( -_kGain*( x - 0.5*width ), 0, 0 );
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
