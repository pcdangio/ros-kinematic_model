#include <kinematic_model/model_plugin.hpp>

using namespace kinematic_model;

std::shared_ptr<model_plugin_t> model_plugin_t::load(const std::string& plugin_path)
{
    // Check that path was provided (otherwise dlsym gets handle to program)
    if(plugin_path.empty())
    {
        ROS_ERROR("failed to open plugin's so file (path is empty)");
        return nullptr;
    }

    // Open plugin's shared object library.
    // NOTE: dlclose is not needed.
    void* so_handle = dlopen(plugin_path.c_str(), RTLD_NOW);
    if(!so_handle)
    {
        ROS_ERROR_STREAM("failed to open plugin's so file (" << dlerror() << ")");
        return nullptr;
    }

    // Get a reference to the create_model_plugin function symbol.
    void* create_model_plugin = dlsym(so_handle, "create_model_plugin");
    if(!create_model_plugin)
    {
        ROS_ERROR_STREAM("failed to get reference to plugin's create_model_plugin function (" << dlerror() << ")");
        return nullptr;
    }

    // Get callable version of create_model_plugin.
    std::function<std::shared_ptr<model_plugin_t>()> instantiate = reinterpret_cast<std::shared_ptr<model_plugin_t>(*)()>(create_model_plugin);

    // Try to instantiate the plugin.
    try
    {
        // Return the created instance.
        return instantiate();
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to instantiate model plugin (" << error.what() << ")");
        return nullptr;
    }
}