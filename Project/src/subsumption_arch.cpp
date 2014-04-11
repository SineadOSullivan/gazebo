#include "subsumption_arch.h"

//using namespace gazebo;
namespace gazebo
{

void SubsumptionArch::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    printf("Loading Subsumption Architecture plugin...\n");
    // Store the pointer to the model
    this->model = _parent;

    // Load parameters for this plugin
    if (this->LoadParams(_sdf))
    {
        printf("Successfully loaded parameters!\n");
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SubsumptionArch::OnUpdate, this, _1));
    }
}

bool SubsumptionArch::LoadParams(sdf::ElementPtr _sdf)
{
    return true;
    // Find controller gain
    if (!_sdf->HasElement("gain"))
    {
        printf("param[gain] not found\n");
        gzerr << "param [gain] not found\n";
        return false;
    }
    else
    {
        // Get sensor name
        //this->gain = _sdf->GetElement("gain")->GetValueDouble();
    }

    // Find sensor name from plugin param
    if (!_sdf->HasElement("ray_sensor"))
    {
        printf("param [ray_sensor] not found\n");
        gzerr << "param [ray_sensor] not found\n";
        return false;
    }
    else
    {
        // Get sensor name
        std::string sensorName = _sdf->GetElement("ray_sensor")->GetValueString();

        // Get pointer to sensor using the SensorMangaer
        //sensors::RaySensorPtr sensor = sensors::
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(sensorName);

        if (!sensor)
        {
            gzerr << "sensor by name ["
                  << sensorName
                  << "] not found in model\n";
            return false;
        }

        /*
        this->laser = boost::shared_dynamic_cast<sensors::RaySensor>(sensor);
        if (!this->laser)
        {
            gzerr << "laser by name ["
                  << sensorName
                  << "] not found in model\n";
            return false;
        }
        */
    }

    // success
    return true;
}

// Called by the world update start event
void SubsumptionArch::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Apply a small linear velocity to the model.
    this->model->SetLinearVel(math::Vector3(.5, 0, 0));
}
}
