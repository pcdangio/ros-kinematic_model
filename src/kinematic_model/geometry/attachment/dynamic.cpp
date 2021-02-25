#include <kinematic_model/geometry/attachment/dynamic.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dynamic_t::dynamic_t(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
    : dynamic_t::attachment_t(attachment_t::type_t::DYNAMIC)
{
    // Store translation indices.
    dynamic_t::m_state_index_x = state_index_x;
    dynamic_t::m_state_index_y = state_index_y;
    dynamic_t::m_state_index_z = state_index_z;

    // Store rotation indices.
    dynamic_t::m_state_index_qw = state_index_qw;
    dynamic_t::m_state_index_qx = state_index_qx;
    dynamic_t::m_state_index_qy = state_index_qy;
    dynamic_t::m_state_index_qz = state_index_qz;
}

// METHODS
transform::transform_t dynamic_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Create position vector.
    Eigen::Vector3d position;
    position.x() = state_vector[dynamic_t::m_state_index_x];
    position.y() = state_vector[dynamic_t::m_state_index_y];
    position.z() = state_vector[dynamic_t::m_state_index_z];

    // Create orientation quaternion.
    Eigen::Quaterniond orientation;
    orientation.w() = state_vector[dynamic_t::m_state_index_qw];
    orientation.x() = state_vector[dynamic_t::m_state_index_qx];
    orientation.y() = state_vector[dynamic_t::m_state_index_qy];
    orientation.z() = state_vector[dynamic_t::m_state_index_qz];

    // Create and return transform.
    return transform::transform_t(position, orientation);
}