#include <kinematic_model/kinematic_model.hpp>

int32_t main(int32_t argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "kinematic_model");

    // Get name of plugin.
    ros::NodeHandle private_node("~");
    std::string p_plugin_name = private_node.param<std::string>("plugin_path", "");

    // Load plugin.
    std::shared_ptr<kinematic_model::kinematic_model_t> kinematic_model = kinematic_model::kinematic_model_t::load_plugin(p_plugin_name);

    // Check if plugin was loaded properly.
    if(!kinematic_model)
    {
        ROS_FATAL_STREAM("failed to start kinematic model");
        return 1;
    }

    ROS_INFO_STREAM("loaded kinematic model plugin [" << p_plugin_name << "]");

    // Run plugin.
    try
    {
        kinematic_model->run();
    }
    catch(const std::exception& error)
    {
        ROS_FATAL_STREAM("run failed (" << error.what() << ")");
        return 1;
    }

    return 0;
}