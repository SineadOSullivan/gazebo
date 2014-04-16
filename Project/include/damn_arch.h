#ifndef DAMN_ARCH_H
#define DAMN_ARCH_H

#include "architecture.h"

namespace gazebo
{
    class DamnArch : public ModelPlugin, public Architecture
    {
    public:
        /**
         * Load the plugin
         * @brief Load
         * @param _parent
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr);
        /**
         * Called by the world update start event
         * @brief OnUpdate
         */
        void OnUpdate(const common::UpdateInfo &);
    private:
        std::vector<double> R, T;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DamnArch)
}
#endif
