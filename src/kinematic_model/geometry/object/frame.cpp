#include <kinematic_model/geometry/object/frame.h>

using namespace kinematic_model::geometry::object;

frame_t::frame_t(const std::string& name)
    : frame_t::object_t(name, object_t::type_t::FRAME)
{

}