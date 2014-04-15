#include "subsumption_arch.h"

using namespace std;

namespace gazebo
{
    void SubsumptionArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Subsumption Architecture plugin..." << endl;
        // Store the pointer to the model
        this->_model = _parent;

        // Initialize this architecture
        if (this->initialize(_sdf))
        {
            gzmsg << "Successfully loaded parameters!" << endl;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SubsumptionArch::OnUpdate, this, _1));
        }
    }

    // Called by the world update start event
    void SubsumptionArch::OnUpdate(const common::UpdateInfo & _info)
    {
        // Apply a small linear velocity to the model.
        this->_model->SetLinearVel(math::Vector3(.5, 0, 0));
    }
}
