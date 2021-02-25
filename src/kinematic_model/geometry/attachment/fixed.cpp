#include <kinematic_model/geometry/attachment/fixed.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
fixed_t::fixed_t(double x, double y, double z, double qw, double qx, double qy, double qz)
    : fixed_t::attachment_t(attachment_t::type_t::FIXED)
{
    // Create the attachment's fixed transform.
    fixed_t::m_transform = transform::transform_t({x, y, z}, {qw, qx, qy, qz});
}

// METHODS
transform::transform_t fixed_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    return fixed_t::m_transform;
}