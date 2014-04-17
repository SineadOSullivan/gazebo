#include "avoidObstacles.h"

using namespace gazebo;
using namespace std;

AvoidObstacles::AvoidObstacles()
    : Behavior(1.0d)
{
}

AvoidObstacles::AvoidObstacles(double kAvoid)
    : Behavior(kAvoid)
{
}

math::Vector3 AvoidObstacles::avoidObstaclesSubsumption(sensors::RaySensorPtr lidar)
{
    return math::Vector3(0.0d, 0.0d, 0.0d);
}

void AvoidObstacles::avoidObstaclesDamn(sensors::RaySensorPtr lidar, std::vector< std::vector<double> >& votes, std::vector<double>& R, std::vector<double>& T)
{
    // Model Parameter for avoid distance
    int N = T.size();
    double dAvoid = 0.5;    // Avoidance distance

    // Method Variables
    int kOff;               // Offset for avoid distance
    double lRange;

    // Loop over each bearing
    for( int j = 0; j < N; j++ )
    {
        // Get sensor data
        lRange = lidar->GetRange(j);
        kOff = std::ceil( atan2(dAvoid,lRange) / lidar->GetAngleResolution() );

        // Loop over all ranges for that bearing
        for( int i = 0; i < R.size(); i++ )
        {
            if( R[i] >= lRange - dAvoid && lRange < 4.0 )
            {
                // Loop over the offset
                for( int k = std::max(0, j-kOff); k < std::min(N, j+kOff); k++)
                {
                    votes[i][k] -= this->_kGain;
                }
            }
            else if ( lRange > 10.0 )
            {
                votes[i][j] += ( R[i]*lRange / 160 );
            }
        }
    }
}




math::Vector3 AvoidObstacles::avoidObstaclesMotorSchema(sensors::RaySensorPtr lidar)
{
    gzmsg << "Avoid Obstacles - Motor Schema" << endl;
    // Sensor Parameters
    const int n = lidar->GetRangeCount();
    gzmsg << "N: " << n << endl;
    const double maxR = lidar->GetRangeMax();
    gzmsg << "MaxR: " << maxR << endl;
    const double minR = lidar->GetRangeMin();
    gzmsg << "MinR: " << minR << endl;
    const double res = lidar->GetAngleResolution();
    gzmsg << "Res: " << res << endl;

    gzmsg << "Initialize method variables" << endl;
    // Method Variables
    double mag = 0.0;
    double angle = (lidar->GetAngleMin()).Radian();

    //std::vector<double> ranges(n);
    math::Vector3 V = math::Vector3(0, 0, 0);

    /*
    gzmsg << "Update Sensor Data" << endl;
    gzmsg << "Ranges: " << ranges.size() << endl;
    // Update Sensor Data
    lidar->GetRanges(ranges);
    gzmsg << "NewRanges: " << ranges.size() << endl;
    */

    // Loop over range data
    for( unsigned int i=0; i < lidar->GetRangeCount(); i++ )
    {
        //Check for minimum range
        if( lidar->GetRange(i) < minR )//ranges[i] < minR )
        {
            gzmsg << "Range below the minimum..." << endl;
            //ranges[i] = maxR;
        }

        // Compute Repulsion Magnitude
        if(lidar->GetRange(i) < 0.95 * maxR)//ranges[i] < 0.95 * maxR)
        {
            mag = _kGain / pow(lidar->GetRange(i) - 0.95 * minR, 2);//ranges[i] - 0.95 * minR, 2);
        }
        else
        {
            mag = 0.0;
        }

        // Subtract repulsing velocity
        V -= mag * math::Vector3(cos(angle), sin(angle), 0);

        //Increment angle for next range
        angle += res;
    }

    // Return Result
    return V;
}
