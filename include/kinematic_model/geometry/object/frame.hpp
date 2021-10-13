/// \file kinematic_model/geometry/object/frame.hpp
/// \brief Defines the kinematic_model::geometry::object::frame_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___FRAME_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___FRAME_H

#include <kinematic_model/geometry/object/object.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace object {

/// \brief A reference frame object attached to the kinematic chain.
class frame_t
    : public object_t
{
public:
    // FACTORY
    /// \brief Creates a new frame object.
    /// \param name The unique name of the frame.
    static std::shared_ptr<frame_t> create(const std::string& name);

private:
    // CONSTRUCTORS
    /// \brief Instantiates a new frame.
    /// \param name The unique name of the frame.
    frame_t(const std::string& name);
    frame_t(const frame_t&) = delete; 
};

}}}

#endif