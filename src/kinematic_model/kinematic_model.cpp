#include <kinematic_model/kinematic_model.hpp>

#include <dlfcn.h>

using namespace kinematic_model;

kinematic_model_t::kinematic_model_t()
{
    // Set up private node handle.
    kinematic_model_t::m_node = std::make_unique<ros::NodeHandle>("~");

    // Initialize pointers.
    kinematic_model_t::m_model_plugin = nullptr;
    kinematic_model_t::m_handle_model_plugin = nullptr;

    // Load the model plugin.
    if(!kinematic_model_t::load_model_plugin())
    {
        ROS_FATAL("failed to load model plugin");
        ros::shutdown();
    }
}
kinematic_model_t::~kinematic_model_t()
{
    // Unload the model plugin if it's loaded.
    kinematic_model_t::unload_model_plugin();
}

void kinematic_model_t::run()
{
    ros::spin();
}

bool kinematic_model_t::load_model_plugin()
{
    std::string p_plugin_path = kinematic_model_t::m_node->param<std::string>("model_plugin", "");

    // Check that path was provided (otherwise dlsym gets handle to program)
    if(p_plugin_path.empty())
    {
        ROS_ERROR("required parameter ~/model_plugin is empty");
        return false;
    }

    // Open plugin's shared object library.
    // NOTE: dlclose is not needed.
    void* so_handle = dlopen(p_plugin_path.c_str(), RTLD_NOW);
    if(!so_handle)
    {
        ROS_ERROR_STREAM("failed to open model plugin (" << dlerror() << ")");
        return false;
    }

    // Get a reference to the create_model_plugin function symbol.
    void* create_model_plugin = dlsym(so_handle, "create_model_plugin");
    if(!create_model_plugin)
    {
        ROS_ERROR_STREAM("failed to get model plugin instantiator (" << dlerror() << ")");
        return false;
    }

    // Get callable version of create_model_plugin.
    std::function<model_plugin_t*()> instantiate = reinterpret_cast<model_plugin_t*(*)()>(create_model_plugin);

    // Instantiate the plugin and store it locally.
    try
    {
        kinematic_model_t::m_model_plugin = instantiate();
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to instantiate model plugin (" << error.what() << ")");
        return false;
    }
    
    // Store SO handle.
    kinematic_model_t::m_handle_model_plugin = so_handle;

    return true;
}
void kinematic_model_t::unload_model_plugin()
{
    // Delete the model plugin if it exists.
    if(kinematic_model_t::m_model_plugin)
    {
        delete kinematic_model_t::m_model_plugin;
        kinematic_model_t::m_model_plugin = nullptr;
    }

    // Close the SO handle if it exists.
    if(kinematic_model_t::m_handle_model_plugin)
    {
        dlclose(kinematic_model_t::m_handle_model_plugin);
        kinematic_model_t::m_handle_model_plugin = nullptr;
    }
}