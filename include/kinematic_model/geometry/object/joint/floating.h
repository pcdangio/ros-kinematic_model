#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___FLOATING_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___FLOATING_H

#include <kinematic_model/geometry/object/joint/joint.h>

namespace kinematic_model {
namespace geometry {
namespace object {
namespace joint {

class floating_t
    : public joint_t
{
public:
    floating_t(const std::string& name, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);

    transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    uint32_t m_state_index_x;
    uint32_t m_state_index_y;
    uint32_t m_state_index_z;
    uint32_t m_state_index_qw;
    uint32_t m_state_index_qx;
    uint32_t m_state_index_qy;
    uint32_t m_state_index_qz;
};

}}}}

#endif