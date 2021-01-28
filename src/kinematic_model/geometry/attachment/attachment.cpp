#include <kinematic_model/geometry/attachment/attachment.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
attachment_t::attachment_t(type_t type)
    : m_type(type)
{
}

// PROPERTIES
attachment_t::type_t attachment_t::type() const
{
    return attachment_t::m_type;
}