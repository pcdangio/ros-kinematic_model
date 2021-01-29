#include <kinematic_model/geometry/geometry.hpp>

#include <ros/console.h>

using namespace kinematic_model::geometry;

bool geometry_t::initialize(const std::shared_ptr<model_plugin_t>& model_plugin)
{
    // Verify that the model plugin is valid.
    if(!model_plugin)
    {
        ROS_ERROR("geometry was initialized with an invalid model plugin");
        return false;
    }

    // Load the geometry design from the plugin.
    kinematic_model::geometry::design_t model_design;
    try
    {
        model_plugin->build_geometry(model_design);
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("model plugin failed to build geometry (" << error.what() << ")");
        return false;
    }

    // Build the graph.
    geometry_t::m_graph.build(model_design);

    return true;
}