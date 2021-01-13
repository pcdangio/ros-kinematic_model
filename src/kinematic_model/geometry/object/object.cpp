#include <kinematic_model/geometry/object/object.hpp>

using namespace kinematic_model::geometry::object;

object_t::object_t(const std::string& name, type_t type)
{
    object_t::m_name = name;
    object_t::m_object_type = type;

    object_t::m_locked = false;
}

std::string object_t::name() const
{
    return object_t::m_name;
}
object_t::type_t object_t::object_type() const
{
    return object_t::m_object_type;
}

void object_t::lock()
{
    object_t::m_locked = true;
}
bool object_t::is_locked() const
{
    return object_t::m_locked;
}