#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___PRISMATIC_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___PRISMATIC_H

#include <kinematic_model/geometry/object/joint/joint.h>

namespace kinematic_model {
namespace geometry {
namespace object {
namespace joint {

class prismatic_t
    : public joint_t
{
public:
    prismatic_t(const std::string& name, uint32_t state_index);

    bool set_axis_definition(double x, double y, double z);

    transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    uint32_t m_state_index;
    Eigen::Vector3d m_axis_definition;
};

}}}}

#endif