#include <kinematic_model/geometry/attachment/dtfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dtfr_t::dtfr_t(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t qw, double_t qx, double_t qy, double_t qz)
    : attachment_t(attachment_t::type_t::DTFR)
{
    // Store values.
    dtfr_t::m_state_index_x = state_index_x;
    dtfr_t::m_state_index_y = state_index_y;
    dtfr_t::m_state_index_z = state_index_z;
    dtfr_t::m_qw = qw;
    dtfr_t::m_qx = qx;
    dtfr_t::m_qy = qy;
    dtfr_t::m_qz = qz;
}

// METHODS
transform::transform_t dtfr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Create translation.
    Eigen::Vector3d translation;
    translation.x() = state_vector[dtfr_t::m_state_index_x];
    translation.y() = state_vector[dtfr_t::m_state_index_y];
    translation.z() = state_vector[dtfr_t::m_state_index_z];

    // Create rotation.
    Eigen::Quaterniond rotation;
    rotation.w() = dtfr_t::m_qw;
    rotation.x() = dtfr_t::m_qx;
    rotation.y() = dtfr_t::m_qy;
    rotation.z() = dtfr_t::m_qz;

    // Return transform.
    return transform::transform_t(translation, rotation);
}