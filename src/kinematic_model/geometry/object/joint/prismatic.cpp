#include <kinematic_model/geometry/object/joint/prismatic.h>

#include <ros/console.h>

using namespace kinematic_model::geometry;
using namespace kinematic_model::geometry::object::joint;

prismatic_t::prismatic_t(const std::string& name, uint32_t state_index)
    : prismatic_t::joint_t(name, joint_t::type_t::PRISMATIC)
{
    // Store state index.
    prismatic_t::m_state_index = state_index;

    // Set default axis definition.
    prismatic_t::m_axis_definition = Eigen::Vector3d::UnitZ();
}

bool prismatic_t::set_axis_definition(double x, double y, double z)
{
    if(prismatic_t::is_locked())
    {
        ROS_ERROR_STREAM("failed to set axis for prismatic joint [" << prismatic_t::name() << "] (editing is locked)");
        return false;
    }

    // Store axis.
    m_axis_definition = {x, y, z};

    // Normalize axis.
    m_axis_definition.normalize();

    return false;
}

transform_t prismatic_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Calculate transform from joint parent frame to child frame.

    // Transform is a translation along the defined axis.
    double distance = state_vector[prismatic_t::m_state_index];
    Eigen::Vector3d translation = prismatic_t::m_axis_definition * distance;

    // Create and return transform using translation.
    return transform_t(translation);
}