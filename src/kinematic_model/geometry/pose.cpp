#include <kinematic_model/geometry/pose.h>

using namespace kinematic_model::geometry;

// CONSTRUCTORS
pose_t::pose_t(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Store position.
    pose_t::m_position = {x, y, z};

    // Convert Euler orientation to quaternion.
    pose_t::m_orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}
pose_t::pose_t(const Eigen::Vector3d& position, const Eigen::Quaterniond orientation)
{
    pose_t::m_position = position;
    pose_t::m_orientation = orientation;
}
    
// METHODS
const Eigen::Vector3d& pose_t::position() const
{
    return pose_t::m_position;
}
const Eigen::Quaterniond& pose_t::orientation() const
{
    return pose_t::m_orientation;
}