#include <kinematic_model/geometry/object/joint.hpp>

#include <ros/console.h>

using namespace kinematic_model::geometry::object;

joint_t::joint_t(const std::string& name, type_t type, uint32_t state_index)
    : joint_t::object_t(name, object_t::type_t::JOINT),
      joint_t::attachment_t(attachment_t::type_t::DYNAMIC)
{
    // Store joint type.
    joint_t::m_joint_type = type;

    // Store joint's state index.
    joint_t::m_state_index = state_index;

    // Set default axis definition.
    joint_t::m_axis_definition = Eigen::Vector3d::UnitZ();
}

joint_t::type_t joint_t::joint_type() const
{
    return joint_t::m_joint_type;
}

bool joint_t::set_axis_definition(double x, double y, double z)
{
    if(joint_t::is_locked())
    {
        ROS_ERROR_STREAM("failed to set axis for joint [" << joint_t::name() << "] (editing is locked)");
        return false;
    }

    // Store axis.
    joint_t::m_axis_definition = {x, y, z};

    // Normalize axis.
    joint_t::m_axis_definition.normalize();

    return false;
}

kinematic_model::geometry::transform_t joint_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Calculate transform from joint parent frame to child frame.

    // Handle based on the joint's type.
    switch(joint_t::m_joint_type)
    {
        case joint_t::type_t::REVOLUTE:
        {
            // Transform is a rotation about the defined axis.
            double angle = state_vector[joint_t::m_state_index];
            Eigen::Quaterniond rotation;
            rotation = Eigen::AngleAxisd(angle, joint_t::m_axis_definition);

            // Create and return transform using rotation.
            return transform_t(rotation);
        }
        case joint_t::type_t::PRISMATIC:
        {
            // Transform is a translation along the defined axis.
            double distance = state_vector[joint_t::m_state_index];
            Eigen::Vector3d translation = joint_t::m_axis_definition * distance;

            // Create and return transform using translation.
            return transform_t(translation);
        }
    }
}