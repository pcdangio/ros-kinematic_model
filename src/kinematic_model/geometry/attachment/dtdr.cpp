#include <kinematic_model/geometry/attachment/dtdr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dtdr_t::dtdr_t()
    : attachment_t(attachment_t::type_t::DYNAMIC)
{
    
}

// FACTORY
std::shared_ptr<dtdr_t> dtdr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
{
    // Create instance.
    std::shared_ptr<dtdr_t> dtdr;

    // Store translation indices.
    dtdr->m_state_index_x = state_index_x;
    dtdr->m_state_index_y = state_index_y;
    dtdr->m_state_index_z = state_index_z;

    // Store rotation indices.
    dtdr->m_state_index_qw = state_index_qw;
    dtdr->m_state_index_qx = state_index_qx;
    dtdr->m_state_index_qy = state_index_qy;
    dtdr->m_state_index_qz = state_index_qz;

    // Return instance.
    return dtdr;
}

// METHODS
transform::transform_t dtdr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Create translation.
    Eigen::Vector3d position;
    position.x() = state_vector[dtdr_t::m_state_index_x];
    position.y() = state_vector[dtdr_t::m_state_index_y];
    position.z() = state_vector[dtdr_t::m_state_index_z];

    // Create rotation.
    Eigen::Quaterniond orientation;
    orientation.w() = state_vector[dtdr_t::m_state_index_qw];
    orientation.x() = state_vector[dtdr_t::m_state_index_qx];
    orientation.y() = state_vector[dtdr_t::m_state_index_qy];
    orientation.z() = state_vector[dtdr_t::m_state_index_qz];

    // Return transform.
    return transform::transform_t(position, orientation);
}