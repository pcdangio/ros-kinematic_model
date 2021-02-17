#include <kinematic_model/kinematic_model.hpp>

#include <dlfcn.h>

using namespace kinematic_model;

kinematic_model_t::kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors)
    : ukf_t(n_state_variables, n_sensors)
{
    // Set up private node handle.
    kinematic_model_t::m_node = std::make_unique<ros::NodeHandle>("~");
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

void kinematic_model_t::run()
{
    ros::spin();
}