#include <kinematic_model/geometry/object/frame.hpp>

using namespace kinematic_model::geometry::object;

// CONSTRUCTORS
frame_t::frame_t(const std::string& name)
    : frame_t::object_t(name, object_t::type_t::FRAME)
{

}