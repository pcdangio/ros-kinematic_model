#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DYNAMIC_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DYNAMIC_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

class dynamic_t
    : public attachment_t
{
public:
    dynamic_t(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);

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

}}}

#endif