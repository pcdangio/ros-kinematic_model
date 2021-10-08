#include <kinematic_model/geometry/object/link.hpp>

using namespace kinematic_model::geometry::object;

/// FACTORY
std::shared_ptr<link_t> link_t::create(const std::string& name)
{
    return std::make_shared<link_t>(name);
}

/// CONSTRUCTORS
link_t::link_t(const std::string& name)
    : link_t::object_t(name, object_t::type_t::LINK)
{

}