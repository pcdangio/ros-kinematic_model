#include <kinematic_model/geometry/object/joint/joint.h>

using namespace kinematic_model::geometry::object::joint;

joint_t::joint_t(const std::string& name, type_t type)
    : joint_t::object_t(name, object_t::type_t::JOINT)
{
    joint_t::m_joint_type = type;
}

joint_t::type_t joint_t::joint_type() const
{
    return joint_t::m_joint_type;
}