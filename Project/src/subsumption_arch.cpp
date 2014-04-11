#include "subsumption_arch.h"

using namespace std;

namespace gazebo
{
    void SubsumptionArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Subsumption Architecture plugin..." << endl;
        // Store the pointer to the model
        this->_model = _parent;

        gzmsg << "Number Sensors: " << _parent->GetSensorCount() << endl;

        for (int i = 0; i < _parent->GetChildCount(); i++)
        {
            physics::BasePtr ch = _parent->GetChild(i);
            gzmsg << "ID: " << ch->GetId() << endl;
            gzmsg << "Scoped Name: " << ch->GetScopedName() << endl;
            for (int j = 0; j < ch->GetChildCount(); j++)
            {
                physics::BasePtr ch2 = ch->GetChild(j);
                gzmsg << "ID: " << ch2->GetId() << endl;
                gzmsg << "Name: " << ch2->GetName() << endl;
                gzmsg << "Scoped Name: " << ch2->GetScopedName() << endl;
            }
        }
        // Load parameters for this plugin
        if (this->LoadParams(_sdf))
        {
            gzmsg << "Successfully loaded parameters!" << endl;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SubsumptionArch::OnUpdate, this, _1));
        }
    }

    bool SubsumptionArch::LoadParams(sdf::ElementPtr _sdf)
    {
        // Initialize the architecture
        return initialize(_sdf);
    }

    // Called by the world update start event
    void SubsumptionArch::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Apply a small linear velocity to the model.
        this->_model->SetLinearVel(math::Vector3(.5, 0, 0));
    }
}
