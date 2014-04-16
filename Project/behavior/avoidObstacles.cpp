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
        return math::Vector3(std::cos(angle), std::sin(angle), 0.0d);
    }
    // Don't bother considering this behavior
    else
        return (math::Vector3(0.0d, 0.0d, 0.0d));
}

void AvoidObstacles::avoidObstaclesDamn(sensors::RaySensorPtr lidar, std::vector< std::vector<double> > & votes, std::vector<double> R, std::vector<double> T)
{

    // Sensor Parameters
    const double maxR = lidar->GetRangeMax();
    const double minR = lidar->GetRangeMin();

    double lRange;


    // Loop through Vote matrix
    for( unsigned int j = 0; j < T.size(); j++ )
    {
        // Get sensor data
        lRange = lidar->GetRange(j);

        // Check for sensor error
        if( lRange < minR )
        {
            lRange = maxR;
        }

        for( unsigned int i = 0; i < R.size(); i++ )
        {
            if( R[i]>lRange)
            {
                votes[i][j] += -this->_kGain;
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
