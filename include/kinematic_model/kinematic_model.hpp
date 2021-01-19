#ifndef KINEMATIC_MODEL___KINEMATIC_MODEL_H
#define KINEMATIC_MODEL___KINEMATIC_MODEL_H

#include <kinematic_model/model_plugin.hpp>

#include <ros/ros.h>

namespace kinematic_model {

class kinematic_model_t
{
public:
    kinematic_model_t();

    void run();

private:
    // ROS
    std::unique_ptr<ros::NodeHandle> m_node;

    // MODEL PLUGIN
    std::shared_ptr<model_plugin_t> m_model_plugin;
};

}

#endif