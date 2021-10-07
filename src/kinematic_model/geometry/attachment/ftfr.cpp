#include <kinematic_model/geometry/attachment/ftfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
ftfr_t::ftfr_t(double x, double y, double z, double qw, double qx, double qy, double qz)
    : ftfr_t::attachment_t(attachment_t::type_t::FTFR)
{
    // Create the attachment's fixed transform.
    ftfr_t::m_transform = transform::transform_t({x, y, z}, {qw, qx, qy, qz});
}

// METHODS
transform::transform_t ftfr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    return ftfr_t::m_transform;
}