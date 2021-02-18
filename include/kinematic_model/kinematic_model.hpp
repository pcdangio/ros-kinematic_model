/// \file kinematic_model/kinematic_model.hpp
/// \brief Defines the kinematic_model::kinematic_model_t class.
#ifndef KINEMATIC_MODEL___KINEMATIC_MODEL_H
#define KINEMATIC_MODEL___KINEMATIC_MODEL_H

#include <kinematic_model/geometry/design.hpp>
#include <kinematic_model/geometry/graph/graph.hpp>

#include <kalman_filter/ukf.hpp>

#include <ros/ros.h>
#include <kinematic_model_msgs/get_transform.h>

/// \brief Components for implementing the kinematic model.
namespace kinematic_model {

/// \brief Provides state estimation and transforms for a kinematic model.
class kinematic_model_t
    : protected kalman_filter::ukf_t
{
public:
    // CONSTRUCTORS
    kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors);
    /// \brief Loads and instantiates a kinematic model plugin.
    /// \param plugin_path The path to the plugin's *.so shared object library.
    /// \returns A shared pointer to the loaded plugin.
    static std::shared_ptr<kinematic_model_t> load_plugin(const std::string& plugin_path);

    // METHODS
    /// \brief Initializes the kinematic model for operation.
    void initialize();
    /// \brief Runs the kinematic model.
    void run();

protected:
    // VARIABLES
    /// \brief The amount of time elapsed (sec) since the last state estimation.
    double_t dt;

    // METHODS
    /// \brief Specifies the design of the physical geometry to model.
    /// \param design The design object for adding instructions to.
    /// \details This method is where plugins shall specify links, joints, and frames.
    virtual void build_geometry(geometry::design_t& design) const = 0;

private:
    // ROS
    /// \brief The node's private handle.
    std::unique_ptr<ros::NodeHandle> m_node;
    
    // STATE ESTIMATION
    /// \brief The timestamp of the last state estimation.
    ros::Time m_timestamp_state_estimation;
    /// \brief A timer callback for running the state estimator.
    /// \param event The timed loop event data.
    void timer_state_estimation(const ros::TimerEvent& event);

    // GEOMETRY
    /// \brief The graph of geometry objects.
    geometry::graph::graph_t m_graph;
    /// \brief The service server for the get_transform service.
    ros::ServiceServer m_service_get_transform;
    /// \brief A callback for the get_transform service.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service succeeded, otherwise FALSE.
    bool service_get_transform(kinematic_model_msgs::get_transformRequest& request, kinematic_model_msgs::get_transformResponse& response);
};

/// \brief Registers a plugin for loading.
/// \param class_name The name of the class definition for the plugin class.
#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif