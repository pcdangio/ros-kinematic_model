/// \file kinematic_model/geometry/graph/connection.hpp
/// \brief Defines the kinematic_model::geometry::graph::connection_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___GRAPH___CONNECTION_H
#define KINEMATIC_MODEL___GEOMETRY___GRAPH___CONNECTION_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace graph {

/// \brief A connection in-between two vertices in a connected graph.
struct connection_t
{
    // ENUMERATIONS
    /// \brief An enumeration of connection directions.
    enum class direction_t
    {
        PARENT_CHILD = 0,   ///< Connection originates from a parent and terminates at a child.
        CHILD_PARENT = 1    ///< Connection originates from a child and terminates at a parent.
    };

    // CONSTRUCTORS
    /// \brief Instantiates a new connection.
    /// \param attachment The attachment that physically makes the connection.
    /// \param direction The direction of the connection.
    /// \note This constructor is primarily used for std emplace() methods.
    connection_t(const std::shared_ptr<attachment::attachment_t>& attachment, const direction_t& direction);

    // VARIABLES
    /// \brief The attachment that physically makes the connection.
    std::shared_ptr<attachment::attachment_t> attachment;
    /// \brief The direction of the connection.
    direction_t direction;
};

}}}

#endif