#ifndef KINEMATIC_MODEL___MODEL_PLUGIN_H
#define KINEMATIC_MODEL___MODEL_PLUGIN_H

#include <kinematic_model/geometry/design.hpp>
#include <dlfcn.h>
#include <ros/console.h>

namespace kinematic_model {

class model_plugin_t
{
public:
    static std::shared_ptr<model_plugin_t> load(const std::string& plugin_path);
    
    virtual bool build(const std::shared_ptr<geometry::design_t>& design) = 0;
};

#define REGISTER_MODEL_PLUGIN(class_name) extern "C" std::shared_ptr<kinematic_model::model_plugin_t> create_model_plugin() {return std::make_shared<class_name>();}

}

#endif