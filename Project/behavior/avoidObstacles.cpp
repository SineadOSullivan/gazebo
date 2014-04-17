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
    double min_dist = 1e6;
    double max_dist = (0.05d * lidar->GetRangeMax());
    // Loop over range data
    for(unsigned int i = (lidar->GetRangeCount() / 3); i < (lidar->GetRangeCount() / 3) * 2; i++)
    {
        if (lidar->GetRange(i) < min_dist)
            min_dist = lidar->GetRange(i);
    }

    // See if we need to care about the obstacles
    if (min_dist < max_dist)
    {
        vector< vector<double> > segments;
        vector<double> seg;
        bool previousClear = false;
        // Loop over the range data to build the line segments
        for(unsigned int i = (lidar->GetRangeCount() / 3); i < (lidar->GetRangeCount() / 3) * 2; i++)
        {
            if (previousClear)
            {
                // Check for any areas greater than our max distance
                if (lidar->GetRange(i) > max_dist)
                {
                    previousClear = true;
                }
                else
                {
                    seg.push_back(i - 1);
                    segments.push_back(seg);
                    seg.clear();
                    previousClear = false;
                }
            }
            else
            {
                // Check for any areas greater than our max distance
                if (lidar->GetRange(i) > max_dist)
                {
                    seg.push_back(i);
                    previousClear = true;
                }
            }
        }
        // Check to see if we ended on a previous Clear
        if (previousClear)
        {
            // Finish off the segment
            seg.push_back((lidar->GetRangeCount() / 3) * 2 - 1);
            segments.push_back(seg);
            seg.clear();
            previousClear = false;
        }
        double segDist = 0.0d;
        // Find the maximum segment size
        for (int j = 0; j < segments.size(); j++)
        {
            // Make sure we only have the end points
            if (segments[j].size() == 2)
            {
                // See if the current segment dist is greater
                if ((segments[j][1] - segments[j][0]) > segDist)
                {
                    // Save the distance
                    segDist = (segments[j][1] - segments[j][0]);
                    // Save the segment
                    seg = segments[j];
                }
            }
        }
#ifdef LOGGING
        // Longest Segment
        gzmsg << "Segment: " << seg[0] << "," << seg[1] << " = " << segDist << " [" << (seg[0] + seg[1])/2.0d << "]" << endl;
#endif
        double midPoint = (seg[0] + seg[1])/2.0d;
        // Now let's calculate the vector to this point
        double angle = lidar->GetAngleMin().Radian() + midPoint * lidar->GetAngleResolution();
#ifdef LOGGING
        gzmsg << "Angle: " << angle << endl;
#endif
        math::Vector3 V(std::cos(angle), std::sin(angle), 0.0d);
        return V;
    }
    // Don't bother considering this behavior
    else
        return (math::Vector3(0.0d, 0.0d, 0.0d));
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

math::Vector3 AvoidObstacles::avoidObstaclesMotorSchema(physics::ModelPtr model, sensors::RaySensorPtr lidar, double kOpen)
{
    // Sensor Parameters
    const int n = lidar->GetRangeCount();
    const double minR = lidar->GetRangeMin();
    const double maxR = lidar->GetRangeMax();

    const double minA = (lidar->GetAngleMin()).Radian();
    const double maxA = (lidar->GetAngleMax()).Radian();
    const double res = lidar->GetAngleResolution();

    // Method Variables
    double range, mag = 0.0;
    double angle = minA;

    math::Pose P = model->GetWorldPose().GetInverse();
    math::Vector3 V = math::Vector3(0, 0, 0);

    // Loop over range data
    for( unsigned int i=0; i < n; i++ )
    {
        // Get Range Data
        range = lidar->GetRange(i);

        // Compute Magnitude
        if( range > minR && range < 0.1*maxR )
        {
            mag = -_kGain/pow(range - minR, 2);
        }
        else if( range > 0.5*maxR )
        {
            mag = kOpen*range;
        }

        // Add to velocity along range bearing
        V += mag*math::Vector3( cos(angle), sin(angle), 0 );

        //Increment angle for next range
        angle += res;
    }

    // Correction factor for velocity bias
    V -= math::Vector3(kOpen*maxR*(sin(maxA) - sin(minA))/res, 0, 0);

    // Rotate Velocity into global frame
    P.Set(V, P.rot);
    P.RotatePositionAboutOrigin(P.rot);
    V = P.pos;

    // Return Result
    return V;
}
