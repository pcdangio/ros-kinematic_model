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
    std::string p_plugin_path = kinematic_model_t::m_node->param<std::string>("model_plugin", "");
    try
    {
        kinematic_model_t::load_model_plugin(p_plugin_path);
    }
    catch(std::exception& error)
    {
        ROS_FATAL_STREAM("failed to load model plugin (" << error.what() << ")");
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

void kinematic_model_t::load_model_plugin(const std::string& plugin_path)
{
    // Check that path was provided (otherwise dlsym gets handle to program)
    if(plugin_path.empty())
    {
        throw std::runtime_error("plugin path is empty");
    }

    // Open plugin's shared object library.
    // NOTE: dlclose is not needed.
    void* so_handle = dlopen(plugin_path.c_str(), RTLD_NOW);
    if(!so_handle)
    {
        throw std::runtime_error(dlerror());
    }

    // Get a reference to the create_model_plugin function symbol.
    void* create_model_plugin = dlsym(so_handle, "create_model_plugin");
    if(!create_model_plugin)
    {
        throw std::runtime_error(dlerror());
    }

    // Get callable version of create_model_plugin.
    std::function<model_plugin_t*()> instantiate = reinterpret_cast<model_plugin_t*(*)()>(create_model_plugin);

    // Instantiate the plugin and store it locally.
    kinematic_model_t::m_model_plugin = instantiate();

    // If this point reached without exception, plugin has loaded successfully.
    // Store SO handle.
    kinematic_model_t::m_handle_model_plugin = so_handle;
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