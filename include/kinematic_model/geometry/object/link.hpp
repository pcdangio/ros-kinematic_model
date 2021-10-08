/// \file kinematic_model/geometry/object/link.hpp
/// \brief Defines the kinematic_model::geometry::object::link_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___LINK_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___LINK_H

#include <kinematic_model/geometry/object/object.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace object {

/// \brief A link object in a kinematic chain.
class link_t
    : public object_t
{
public:
    /// \brief Creates a new link object.
    /// \param name The unique name of the link.
    static std::shared_ptr<link_t> create(const std::string& name);

private:
    // CONSTRUCTORS
    /// \brief Instantiates a new link object.
    /// \param name The unique name of the link object.
    link_t(const std::string& name);
    link_t(const link_t&) = delete;
};

}}}

#endif