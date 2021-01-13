#include <kinematic_model/geometry/object/joint/float.h>

using namespace kinematic_model::geometry;
using namespace kinematic_model::geometry::object;

joint::float_t::float_t(const std::string& name, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
    : float_t::joint_t(name, joint_t::type_t::FLOAT)
{
    // Store state indices.
    float_t::m_state_index_x = state_index_x;
    float_t::m_state_index_y = state_index_y;
    float_t::m_state_index_z = state_index_z;
    float_t::m_state_index_qw = state_index_qw;
    float_t::m_state_index_qx = state_index_qx;
    float_t::m_state_index_qy = state_index_qy;
    float_t::m_state_index_qz = state_index_qz;
}

transform_t joint::float_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // The transform is a pose offset.

    // Create position vector.
    Eigen::Vector3d position;
    position.x() = state_vector[float_t::m_state_index_x];
    position.y() = state_vector[float_t::m_state_index_y];
    position.z() = state_vector[float_t::m_state_index_z];

    // Create orientation quaternion.
    Eigen::Quaterniond orientation;
    orientation.w() = state_vector[float_t::m_state_index_qw];
    orientation.x() = state_vector[float_t::m_state_index_qx];
    orientation.y() = state_vector[float_t::m_state_index_qy];
    orientation.z() = state_vector[float_t::m_state_index_qz];

    // Create and return transform.
    return transform_t(position, orientation);
}