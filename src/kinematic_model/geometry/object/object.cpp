#include <kinematic_model/geometry/object/object.hpp>

using namespace kinematic_model::geometry::object;

// CONSTRUCTORS
object_t::object_t(const std::string& name, type_t type)
    : m_name(name),
      m_object_type(type)
{
    object_t::m_locked = false;
}

// METHODS
void object_t::lock()
{
    object_t::m_locked = true;
}

// PROPERTIES
std::string object_t::name() const
{
    return object_t::m_name;
}
object_t::type_t object_t::object_type() const
{
    return object_t::m_object_type;
}
bool object_t::is_locked() const
{
    return object_t::m_locked;
}