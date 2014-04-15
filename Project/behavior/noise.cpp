#include "noise.h"

using namespace gazebo;

Noise::Noise()
    : Behavior(1.0d)
{
}

Noise::Noise(double kNoise)
    : Behavior(kNoise)
{
}

math::Vector3 Noise::addNoise()
{
    // Return Result
    return this->_kGain * math::Vector3(this->_rand.GetDblNormal(0,1), this->_rand.GetDblNormal(0,1), 0.0d);
}
