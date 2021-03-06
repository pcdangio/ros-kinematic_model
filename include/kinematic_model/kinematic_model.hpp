/// \file kinematic_model/kinematic_model.hpp
/// \brief Defines the kinematic_model::kinematic_model_t class.
#ifndef KINEMATIC_MODEL___KINEMATIC_MODEL_H
#define KINEMATIC_MODEL___KINEMATIC_MODEL_H

#include <kinematic_model/geometry/design.hpp>
#include <kinematic_model/geometry/graph/graph.hpp>

#include <kalman_filter/ukf.hpp>

#include <ros/ros.h>

#include <geometry_msgs_ext/get_transform.h>

/// \brief Components for implementing the kinematic model.
namespace kinematic_model {

/// \brief Provides state estimation and transforms for a kinematic model.
class kinematic_model_t
    : protected kalman_filter::ukf_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new kinematic_model object.
    /// \param n_state_variables The number of variables in the model's state vector.
    /// \param n_sensors The number of sensors providing observations to the model's state.
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
    /// \brief This method is called internally each time the model's state is updated.
    /// \details User plugins may optionally override this method to have their own code called each time a new state is calculated.
    virtual void on_state_update();
    /// \brief Gets a transform between two frames.
    /// \param source_frame The desired source frame of the transform.
    /// \param target_frame The desired target frame of the transform.
    /// \param state_vector The state vector to base transforms on.
    /// \param transform OUTPUT The calculated transform.
    /// \returns TRUE if the transform was able to be calculated, otherwise FALSE.
    /// \note This method uses the state estimation cache.
    bool get_transform(const std::string& source_frame, const std::string& target_frame, const Eigen::VectorXd& state_vector, transform::transform_t& transform) const;

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
    /// \brief A cache of calculated transforms for the get_transform service.
    std::unordered_map<std::string, transform::transform_t> m_transform_cache;
    /// \brief The service server for the get_transform service.
    ros::ServiceServer m_service_get_transform;
    /// \brief A callback for the get_transform service.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service succeeded, otherwise FALSE.
    /// \note This method uses the transform service cache.
    bool service_get_transform(geometry_msgs_ext::get_transformRequest& request, geometry_msgs_ext::get_transformResponse& response);
};

/// \brief Registers a plugin for loading.
/// \param class_name The name of the class definition for the plugin class.
#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif