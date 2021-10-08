#include <kinematic_model/geometry/attachment/dtfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dtfr_t::dtfr_t()
    :attachment_t(attachment_t::type_t::DYNAMIC)
{
    
}

// FACTORY
std::shared_ptr<dtfr_t> dtfr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t qw, double_t qx, double_t qy, double_t qz)
{
    // Create instance.
    std::shared_ptr<dtfr_t> dtfr = std::make_shared<dtfr_t>();

    // Store translation state indices.
    dtfr->m_state_index_x = state_index_x;
    dtfr->m_state_index_y = state_index_y;
    dtfr->m_state_index_z = state_index_z;

    // Store rotation values.
    dtfr->m_qw = qw;
    dtfr->m_qx = qx;
    dtfr->m_qy = qy;
    dtfr->m_qz = qz;

    // Return instance.
    return dtfr;
}
std::shared_ptr<dtfr_t> dtfr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t roll, double_t pitch, double_t yaw)
{
    // Convert euler rotation to quaternion.
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // Use base factory method to create instance.
    return dtfr_t::create(state_index_x, state_index_y, state_index_z, q.w(), q.x(), q.y(), q.z());
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