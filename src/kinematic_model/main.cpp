#include <kinematic_model/kinematic_model.hpp>

int32_t main(int32_t argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "kinematic_model");

    // Get name of plugin.
    ros::NodeHandle private_node("~");
    std::string p_plugin_path = private_node.param<std::string>("plugin_path", "");

    // Load plugin.
    std::shared_ptr<kinematic_model::kinematic_model_t> kinematic_model = kinematic_model::kinematic_model_t::load_plugin(p_plugin_path);

    // Check if plugin was loaded properly.
    if(!kinematic_model)
    {
        ROS_FATAL_STREAM("failed to load kinematic model plugin [" << p_plugin_path << "]");
        return 1;
    }

    ROS_INFO_STREAM("loaded kinematic model plugin [" << p_plugin_path << "]");

    // Initialize the plugin.
    if(!kinematic_model->initialize())
    {
        ROS_FATAL("failed to initialize plugin");
        return 1;
    }

    ROS_INFO("initialized kinematic model plugin");

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