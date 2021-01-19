#include <kinematic_model/kinematic_model.hpp>

#include <dlfcn.h>

using namespace kinematic_model;

kinematic_model_t::kinematic_model_t()
{
    // Grab private node handle.
    kinematic_model_t::m_node = std::make_unique<ros::NodeHandle>("~");

    // Load the model plugin.
    std::string p_plugin_path = kinematic_model_t::m_node->param<std::string>("model_plugin_path", "");
    kinematic_model_t::m_model_plugin = model_plugin_t::load(p_plugin_path);
    if(!kinematic_model_t::m_model_plugin)
    {
        ROS_FATAL("failed to load model plugin");
        ros::shutdown();
    }
}

void kinematic_model_t::run()
{
    ros::spin();
}