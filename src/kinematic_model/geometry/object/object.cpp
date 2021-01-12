#include <kinematic_model/geometry/object/object.h>

using namespace kinematic_model::geometry::object;

object_t::object_t(const std::string& name, type_t type)
{
    object_t::m_name = name;
    object_t::m_object_type = type;
}

std::string object_t::name() const
{
    return object_t::m_name;
}
object_t::type_t object_t::object_type() const
{
    return object_t::m_object_type;
}