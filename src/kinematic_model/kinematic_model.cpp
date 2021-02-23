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
void kinematic_model_t::on_state_update()
{
    // Do nothing; this is an optional base class space holder.
}
bool kinematic_model_t::get_transform(const std::string& source_frame, const std::string& target_frame, geometry::transform_t& transform)
{
    // Get the transform path from the graph.
    // NOTE: graph internally uses caching on path solving.
    auto path = kinematic_model_t::m_graph.solve_path(source_frame, target_frame);

    // Check if path was found.
    if(!path)
    {
        return false;
    }

    // Iterate through the solved path.
    for(auto connection = path->cbegin(); connection != path->cend(); ++connection)
    {
        // Get the transform from the connection's attachment using the current state.
        auto attachment_transform = connection->attachment->get_transform(kinematic_model_t::x);

        // Invert the transform if needed.
        if(connection->direction == geometry::graph::connection_t::direction_t::CHILD_PARENT)
        {
            attachment_transform.invert();
        }

        // Chain the transform.
        attachment_transform.transform(transform);
    }

    return true;
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

    // Reset the transform cache.
    kinematic_model_t::m_transform_cache.clear();

    // Call user code.
    on_state_update();
}
bool kinematic_model_t::service_get_transform(kinematic_model_msgs::get_transformRequest& request, kinematic_model_msgs::get_transformResponse& response)
{
    // Create an output transform (defaults to identity).
    geometry::transform_t transform;

    // Create flag for tracking success of transform retrieval.
    bool success = false;

    // Check the transform cache.
    std::string cache_key = request.source_frame + ":" + request.target_frame;
    auto cache_iterator = kinematic_model_t::m_transform_cache.find(cache_key);
    if(cache_iterator != kinematic_model_t::m_transform_cache.end())
    {
        // Transform found in cache.

        // Read transform from cache.
        transform = cache_iterator->second;

        // Flag successful transform.
        success = true;
    }
    else
    {
        // Transform not found in cache.

        // Calculate the transform.
        if(kinematic_model_t::get_transform(request.source_frame, request.target_frame, transform))
        {
            // Transform successfully calculated.

            // NOTE: transform stored in output via function.

            // Flag successful transform.
            success = true;

            // Add transform forward/reverse to cache.
            std::string cache_key_reverse = request.target_frame + ":" + request.source_frame;
            kinematic_model_t::m_transform_cache[cache_key] = transform;
            kinematic_model_t::m_transform_cache[cache_key_reverse] = transform.inverse();
        }
    }

    // Set up response.
    auto& translation = transform.translation();
    response.transform.x = translation.x();
    response.transform.y = translation.y();
    response.transform.z = translation.z();
    auto& rotation = transform.rotation();
    response.transform.qw = rotation.w();
    response.transform.qx = rotation.x();
    response.transform.qy = rotation.y();
    response.transform.qz = rotation.z();

    // Indicate if transform calculation was a success.
    return success;
}