#include <kinematic_model/kinematic_model.hpp>

using namespace kinematic_model;

kinematic_model_t::kinematic_model_t()
{
    // Set up private node handle.
    kinematic_model_t::m_node = std::make_unique<ros::NodeHandle>("~");
}

bool kinematic_model_t::initialize()
{
    // Load the model plugin.
    std::string p_model_plugin_path = kinematic_model_t::m_node->param<std::string>("model_plugin", "");
    kinematic_model_t::m_model_plugin = model_plugin_t::load_kinematic_model(p_model_plugin_path);
    if(!kinematic_model_t::m_model_plugin)
    {
        ROS_ERROR("kinematic model failed to load model plugin");
        return false;
    }

    // Initialize geometry.
    if(!kinematic_model_t::m_geometry.initialize(kinematic_model_t::m_model_plugin))
    {
        ROS_ERROR("kinematic model failed to initialize geometry");
        return false;
    }

    return true;
}

void kinematic_model_t::run()
{
    ros::spin();
}