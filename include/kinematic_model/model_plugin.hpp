/// \file kinematic_model/model_plugin.hpp
/// \brief Defines the kinematic_model::model_plugin_t class.
#ifndef KINEMATIC_MODEL___MODEL_PLUGIN_H
#define KINEMATIC_MODEL___MODEL_PLUGIN_H

#include <kinematic_model/geometry/design.hpp>

#include <kalman_filter/ukf/model_plugin.hpp>

namespace kinematic_model {

/// \brief The base class for all kinematic model plugins.
/// \note This class extends the UKF model plugin from the kalman_filter ROS package.
class model_plugin_t
    : public kalman_filter::ukf::model_plugin_t
{
public:
    // FACTORY
    /// \brief Loads and instantiates a kinematic model plugin.
    /// \param plugin_path The path to the model plugin's *.so shared object library.
    /// \returns A shared pointer to the loaded model plugin.
    static std::shared_ptr<model_plugin_t> load_kinematic_model(const std::string& plugin_path);

    // METHODS
    /// \brief Specifies the design of the physical geometry to model.
    /// \param design The design object for adding instructions to.
    /// \details This method is where plugins shall specify links, joints, and frames.
    virtual void build_geometry(geometry::design_t& design) = 0;

protected:
    // CONSTRUCTORS
    /// \brief Instantiates a new model_plugin based on the number of state and measurement variables.
    /// \param n_state_variables The number of state variables to be estimated by the model's UKF.
    /// \param n_measurement_variables The number of measurement variables to incorporate in the model's UKF.
    model_plugin_t(uint32_t n_state_variables, uint32_t n_measurement_variables);
};

}

#endif