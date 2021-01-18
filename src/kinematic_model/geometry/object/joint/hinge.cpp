#include <kinematic_model/geometry/object/joint/hinge.hpp>

#include <ros/console.h>

using namespace kinematic_model::geometry;
using namespace kinematic_model::geometry::object::joint;

hinge_t::hinge_t(const std::string& name, uint32_t state_index)
    : hinge_t::joint_t(name, joint_t::type_t::HINGE)
{
    // Store state index.
    hinge_t::m_state_index = state_index;

    // Set default axis definition.
    hinge_t::m_axis_definition = Eigen::Vector3d::UnitZ();
}

std::shared_ptr<hinge_t> hinge_t::create(const std::string& name, uint32_t state_index)
{
    return std::make_shared<hinge_t>(name, state_index);
}

bool hinge_t::set_axis_definition(double x, double y, double z)
{
    if(hinge_t::is_locked())
    {
        ROS_ERROR_STREAM("failed to set axis for hinge joint [" << hinge_t::name() << "] (editing is locked)");
        return false;
    }

    // Store axis.
    m_axis_definition = {x, y, z};

    // Normalize axis.
    m_axis_definition.normalize();

    return false;
}

transform_t hinge_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Calculate transform from joint parent frame to child frame.

    // Transform is a rotation about the defined axis.
    double angle = state_vector[hinge_t::m_state_index];
    Eigen::Quaterniond rotation;
    rotation = Eigen::AngleAxisd(angle, hinge_t::m_axis_definition);

    // Create and return transform using rotation.
    return transform_t(rotation);
}