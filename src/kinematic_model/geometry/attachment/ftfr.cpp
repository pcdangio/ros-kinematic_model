#include <kinematic_model/geometry/attachment/ftfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
ftfr_t::ftfr_t()
    : attachment_t(attachment_t::type_t::FIXED)
{

}

// FACTORY
std::shared_ptr<ftfr_t> ftfr_t::create(double x, double y, double z, double qw, double qx, double qy, double qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<ftfr_t> ftfr(new ftfr_t());

    // Set fixed transform.
    ftfr->m_transform = transform::transform_t({x, y, z}, {qw, qx, qy, qz});

    // Return instance.
    return ftfr;
}
std::shared_ptr<ftfr_t> ftfr_t::create(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Convert euler rotation to quaternion.
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // Use base factory method to create instance.
    return ftfr_t::create(x, y, z, q.w(), q.x(), q.y(), q.z());
}

// METHODS
transform::transform_t ftfr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    return ftfr_t::m_transform;
}