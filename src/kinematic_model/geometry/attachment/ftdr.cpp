#include <kinematic_model/geometry/attachment/ftdr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
ftdr_t::ftdr_t()
    : attachment_t(attachment_t::type_t::DYNAMIC)
{

}

// FACTORY
std::shared_ptr<ftdr_t> ftdr_t::create(double x, double y, double z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<ftdr_t> ftdr(new ftdr_t());

    // Store translation values.
    ftdr->m_x = x;
    ftdr->m_y = y;
    ftdr->m_z = z;

    // Store rotation state indices.
    ftdr->m_state_index_qw = state_index_qw;
    ftdr->m_state_index_qx = state_index_qx;
    ftdr->m_state_index_qy = state_index_qy;
    ftdr->m_state_index_qz = state_index_qz;

    // Return instance.
    return ftdr;
}

// METHODS
transform::transform_t ftdr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Create translation.
    Eigen::Vector3d translation;
    translation.x() = ftdr_t::m_x;
    translation.y() = ftdr_t::m_y;
    translation.z() = ftdr_t::m_z;

    // Create rotation.
    Eigen::Quaterniond rotation;
    rotation.w() = state_vector[ftdr_t::m_state_index_qw];
    rotation.x() = state_vector[ftdr_t::m_state_index_qx];
    rotation.y() = state_vector[ftdr_t::m_state_index_qy];
    rotation.z() = state_vector[ftdr_t::m_state_index_qz];

    // Return transform.
    return transform::transform_t(translation, rotation);
}