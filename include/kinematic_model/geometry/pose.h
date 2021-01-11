#ifndef KINEMATIC_MODEL___GEOMETRY___POSE_H
#define KINEMATIC_MODEL___GEOMETRY___POSE_H

#include <eigen3/Eigen/Dense>

namespace kinematic_model {
namespace geometry {

class pose_t
{
public:
    pose_t(double x, double y, double z, double roll, double pitch, double yaw);
    pose_t(const Eigen::Vector3d& position, const Eigen::Quaterniond orientation);
    
    const Eigen::Vector3d& position() const;
    const Eigen::Quaterniond& orientation() const;
    
    friend class transform_t;
private:
    Eigen::Vector3d m_position;
    Eigen::Quaterniond m_orientation;
};

}}

#endif