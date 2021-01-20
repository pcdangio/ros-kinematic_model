#ifndef KINEMATIC_MODEL___MODEL_PLUGIN_H
#define KINEMATIC_MODEL___MODEL_PLUGIN_H

#include <kinematic_model/geometry/design.hpp>

#include <kalman_filter/ukf/model_plugin.hpp>

namespace kinematic_model {

class model_plugin_t
    : public kalman_filter::ukf::model_plugin_t
{
public:
    model_plugin_t(uint32_t n_state_variables, uint32_t n_measurement_variables);
    
    virtual bool build_geometry(geometry::design_t& design) = 0;
};

#define REGISTER_MODEL_PLUGIN(class_name) extern "C" kalman_filter::ukf::model_plugin_t* create_model_plugin() {return new class_name();}

}

#endif