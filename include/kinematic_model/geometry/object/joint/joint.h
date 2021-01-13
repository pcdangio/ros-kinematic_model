#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___JOINT_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT___JOINT_H

#include <kinematic_model/geometry/object/object.h>
#include <kinematic_model/geometry/transform.h>

namespace kinematic_model {
namespace geometry {
namespace object {
namespace joint {

class joint_t
    : public object_t
{
public:
    enum class type_t
    {
        FLOAT = 0,
        HINGE = 1,
        PRISMATIC = 2
    };

    type_t joint_type() const;

    virtual transform_t get_transform(const Eigen::VectorXd& state_vector) const = 0;

protected:
    joint_t(const std::string& name, type_t type);

private:
    type_t m_joint_type;
};

}}}}

#endif