#include <kinematic_model/kinematic_model.hpp>

#include <dlfcn.h>

using namespace kinematic_model;

kinematic_model_t::kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors)
    : ukf_t(n_state_variables, n_sensors)
{
    // Set up private node handle.
    kinematic_model_t::m_node = std::make_unique<ros::NodeHandle>("~");

    // Initialize delta time.
    kinematic_model_t::dt = 0; 

    // Set up services.
    kinematic_model_t::m_service_get_transform = kinematic_model_t::m_node->advertiseService("get_transform", &kinematic_model_t::service_get_transform, this);
}
std::shared_ptr<kinematic_model_t> kinematic_model_t::load_plugin(const std::string& plugin_path)
{
    // Check that path was provided (dl gets handle to program if empty)
    if(plugin_path.empty())
    {
        ROS_ERROR("attempted to load plugin with empty path");
        return nullptr;
    }

    // Open plugin shared object library.
    void* so_handle = dlopen(plugin_path.c_str(), RTLD_NOW);
    if(!so_handle)
    {
        ROS_ERROR_STREAM("failed to load model plugin (" << dlerror() << ")");
        return nullptr;
    }

    // Get a reference to the instantiate symbol.
    typedef kinematic_model_t* (*instantiate_t)();
    instantiate_t instantiate = reinterpret_cast<instantiate_t>(dlsym(so_handle, "instantiate"));
    if(!instantiate)
    {
        ROS_ERROR_STREAM("failed to load model plugin (" << dlerror() << ")");
        dlclose(so_handle);
        return nullptr;
    }

    // Try to instantiate the plugin.
    kinematic_model_t* plugin = nullptr;
    try
    {
        plugin = instantiate();
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to instantiate model plugin (" << error.what() << ")");
        dlclose(so_handle);
        return nullptr;
    }

    // Return the plugin as a shared ptr with a custom deleter.
    return std::shared_ptr<kinematic_model_t>(plugin,
                                           [so_handle](kinematic_model_t* plugin){delete plugin; dlclose(so_handle);});
}

void kinematic_model_t::initialize()
{
    // First attempt to build geometry design.
    geometry::design_t design;
    try
    {
        build_geometry(design);
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("plugin::build_geometry failed (" << error.what() << ")");
    }

    // Build graph.
    kinematic_model_t::m_graph.build(design);
}
void kinematic_model_t::run()
{
    // Create loop timer for state estimator.
    auto p_loop_rate = kinematic_model_t::m_node->param<double_t>("state_estimation_rate", 100);
    auto loop_timer = kinematic_model_t::m_node->createTimer(ros::Duration(1.0/p_loop_rate), &kinematic_model_t::timer_state_estimation, this);
    // Initialize state estimation timestamp.
    kinematic_model_t::m_timestamp_state_estimation = ros::Time::now();

    // Spin node.
    ros::spin();
}

// PROTECTED METHODS
bool kinematic_model_t::get_transform(const std::string& source_frame, const std::string& target_frame, geometry::transform_t& transform)
{

}

// ROS CALLBACKS
void kinematic_model_t::timer_state_estimation(const ros::TimerEvent& event)
{
    // Calculate current delta time for this iteration.
    auto current_timestamp = ros::Time::now();
    kinematic_model_t::dt = (current_timestamp - kinematic_model_t::m_timestamp_state_estimation).toSec();
    kinematic_model_t::m_timestamp_state_estimation = current_timestamp;

    // Run iteration for state estimator.
    try
    {
        kinematic_model_t::iterate();
    }
    catch(const std::exception& error)
    {
        ROS_FATAL_STREAM("state estimator failed (" << error.what() << ")");
        ros::shutdown();
    }
}
bool kinematic_model_t::service_get_transform(kinematic_model_msgs::get_transformRequest& request, kinematic_model_msgs::get_transformResponse& response)
{
    
    return true;
}