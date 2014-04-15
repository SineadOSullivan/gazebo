#include "avoidObstacles.h"

using namespace gazebo;

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

}

math::Vector3 AvoidObstacles::avoidObstaclesDamn(sensors::RaySensorPtr lidar)
{

}

math::Vector3 AvoidObstacles::avoidObstaclesMotorSchema(sensors::RaySensorPtr lidar)
{
    // Sensor Parameters
    const int n = lidar->GetRangeCount();
    const double maxR = lidar->GetRangeMax();
    const double minR = lidar->GetRangeMin();
    const double res = lidar->GetAngleResolution();

    // Method Variables
    double mag = 0.0;
    double angle = (lidar->GetAngleMin()).Radian();

    std::vector<double> ranges;
    math::Vector3 V = math::Vector3(0, 0, 0);

    // Update Sensor Data
    lidar->GetRanges(ranges);

    // Loop over range data
    for( unsigned int i=0; i < n; i++ )
    {
        //Check for minimum range
        if( ranges[i] < minR )
        {
            ranges[i] = maxR;
        }

        // Compute Repulsion Magnitude
        if(ranges[i] < 0.95 * maxR)
        {
            mag = _kGain / pow(ranges[i] - 0.95 * minR, 2);
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
