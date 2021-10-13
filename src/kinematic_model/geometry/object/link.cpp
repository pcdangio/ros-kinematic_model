#include <kinematic_model/geometry/object/link.hpp>

using namespace kinematic_model::geometry::object;

/// FACTORY
std::shared_ptr<link_t> link_t::create(const std::string& name)
{
    // Create and return shared pointer.
    // NOTE: std::make_shared doesn't work with private constructors
    return std::shared_ptr<link_t>(new link_t(name));
}

/// CONSTRUCTORS
link_t::link_t(const std::string& name)
    : link_t::object_t(name, object_t::type_t::LINK)
{

}