#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___LINK_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___LINK_H

#include <kinematic_model/geometry/object/object.h>

namespace kinematic_model {
namespace geometry {
namespace object {

class link_t
    : public object_t
{
public:
    link_t(const std::string& name);
};

}}}

#endif