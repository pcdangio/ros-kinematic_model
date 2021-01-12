#include <kinematic_model/geometry/object/link.h>

using namespace kinematic_model::geometry::object;

link_t::link_t(const std::string& name)
    : link_t::object_t(name, object_t::type_t::LINK)
{

}