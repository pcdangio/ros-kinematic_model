#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___FRAME_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___FRAME_H

#include <kinematic_model/geometry/object/object.hpp>

namespace kinematic_model {
namespace geometry {
namespace object {

class frame_t
    : public object_t
{
public:
    frame_t(const std::string& name);
};

}}}

#endif