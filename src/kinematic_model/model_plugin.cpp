#include <kinematic_model/model_plugin.hpp>

using namespace kinematic_model;

// CONSTRUCTORS
model_plugin_t::model_plugin_t(uint32_t n_state_variables, uint32_t n_measurement_variables)
    : kalman_filter::ukf::model_plugin_t(n_state_variables, n_measurement_variables)
{

}

// FACTORY
std::shared_ptr<model_plugin_t> model_plugin_t::load_kinematic_model(const std::string& plugin_path)
{
    return std::dynamic_pointer_cast<kinematic_model::model_plugin_t>(model_plugin_t::load_base_model(plugin_path));
}