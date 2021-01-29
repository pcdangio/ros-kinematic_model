#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___JOINT_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___JOINT_H

#include <kinematic_model/geometry/object/object.hpp>
#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace object {

class joint_t
    : public object_t,
      public attachment::attachment_t
{
public:
    enum class type_t
    {
        REVOLUTE = 0,
        PRISMATIC = 1
    };

    joint_t(const std::string& name, type_t type, uint32_t state_index);

    type_t joint_type() const;

    bool set_axis_definition(double x, double y, double z);

    transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    type_t m_joint_type;
    Eigen::Vector3d m_axis_definition;

    uint32_t m_state_index;
};

}}}

#endif