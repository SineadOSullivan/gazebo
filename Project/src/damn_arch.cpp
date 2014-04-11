#include "damn_arch.h"

using namespace gazebo;

void DamnArch::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->_model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DamnArch::OnUpdate, this, _1));
}

// Called by the world update start event
void DamnArch::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Apply a small linear velocity to the model.
    this->_model->SetLinearVel(math::Vector3(.03, 0, 0));
}
