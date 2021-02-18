#ifndef KINEMATIC_MODEL___KINEMATIC_MODEL_H
#define KINEMATIC_MODEL___KINEMATIC_MODEL_H

#include <kinematic_model/geometry/design.hpp>
#include <kinematic_model/geometry/graph/graph.hpp>

#include <kalman_filter/ukf.hpp>

#include <ros/ros.h>

namespace kinematic_model {

class kinematic_model_t
    : protected kalman_filter::ukf_t
{
public:
    kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors);
    /// \brief Loads and instantiates a kinematic model plugin.
    /// \param plugin_path The path to the plugin's *.so shared object library.
    /// \returns A shared pointer to the loaded plugin.
    static std::shared_ptr<kinematic_model_t> load_plugin(const std::string& plugin_path);

    void initialize();
    void run();

protected:
    /// \brief Specifies the design of the physical geometry to model.
    /// \param design The design object for adding instructions to.
    /// \details This method is where plugins shall specify links, joints, and frames.
    virtual void build_geometry(geometry::design_t& design) const = 0;

    double_t dt() const;

private:
    // ROS
    std::unique_ptr<ros::NodeHandle> m_node;

    // GEOMETRY
    geometry::graph::graph_t m_graph;

    void timer_state_estimator(const ros::TimerEvent& event);

    double_t m_dt;
};

/// \brief Registers a plugin for loading.
/// \param class_name The name of the class definition for the plugin class.
#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif