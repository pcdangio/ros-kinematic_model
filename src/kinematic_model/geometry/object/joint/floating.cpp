#include <kinematic_model/geometry/object/joint/floating.h>

using namespace kinematic_model::geometry;
using namespace kinematic_model::geometry::object::joint;

floating_t::floating_t(const std::string& name, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
    : floating_t::joint_t(name, joint_t::type_t::FLOAT)
{
    // Store state indices.
    floating_t::m_state_index_x = state_index_x;
    floating_t::m_state_index_y = state_index_y;
    floating_t::m_state_index_z = state_index_z;
    floating_t::m_state_index_qw = state_index_qw;
    floating_t::m_state_index_qx = state_index_qx;
    floating_t::m_state_index_qy = state_index_qy;
    floating_t::m_state_index_qz = state_index_qz;
}

transform_t floating_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // The transform is a pose offset.

    // Create position vector.
    Eigen::Vector3d position;
    position.x() = state_vector[floating_t::m_state_index_x];
    position.y() = state_vector[floating_t::m_state_index_y];
    position.z() = state_vector[floating_t::m_state_index_z];

    // Create orientation quaternion.
    Eigen::Quaterniond orientation;
    orientation.w() = state_vector[floating_t::m_state_index_qw];
    orientation.x() = state_vector[floating_t::m_state_index_qx];
    orientation.y() = state_vector[floating_t::m_state_index_qy];
    orientation.z() = state_vector[floating_t::m_state_index_qz];

    // Create and return transform.
    return transform_t(position, orientation);
}