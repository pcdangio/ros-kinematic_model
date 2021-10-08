#include <kinematic_model/geometry/object/frame.hpp>

using namespace kinematic_model::geometry::object;

// FACTORY
std::shared_ptr<frame_t> frame_t::create(const std::string& name)
{
    return std::make_shared<frame_t>(name);
}

// CONSTRUCTORS
frame_t::frame_t(const std::string& name)
    : frame_t::object_t(name, object_t::type_t::FRAME)
{

}