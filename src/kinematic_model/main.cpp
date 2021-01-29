#include <kinematic_model/kinematic_model.hpp>

int32_t main(int32_t argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "kinematic_model");

    // Create kinematic_model.
    kinematic_model::kinematic_model_t kinematic_model;

    // Initialize kinematic model.
    if(kinematic_model.initialize())
    {
        // Run kinematic_model.
        kinematic_model.run();
    }

    return 0;
}