#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FIXED_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FIXED_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

class fixed_t
    : public attachment_t
{
public:
    fixed_t(double x, double y, double z, double qw, double qx, double qy, double qz);
    
    transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    transform_t m_transform;
};

}}}

#endif