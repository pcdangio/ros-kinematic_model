#ifndef KINEMATIC_MODEL___GEOMETRY___TRANSFORM_H
#define KINEMATIC_MODEL___GEOMETRY___TRANSFORM_H

#include <kinematic_model/geometry/pose.hpp>

#include <eigen3/Eigen/Dense>

namespace kinematic_model {
namespace geometry {

class transform_t
{
public:
    transform_t();
    transform_t(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
    transform_t(const Eigen::Vector3d& translation);
    transform_t(const Eigen::Quaterniond& rotation);

    void transform(transform_t& transform) const;
    void transform(pose_t& pose) const;

    transform_t inverse() const;
    
private:
    Eigen::Vector3d m_translation;
    Eigen::Quaterniond m_rotation;
};

}}

#endif