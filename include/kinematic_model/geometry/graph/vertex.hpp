/// \file kinematic_model/geometry/graph/vertex.hpp
/// \brief Defines the kinematic_model::geometry::graph::vertex_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___GRAPH___VERTEX_H
#define KINEMATIC_MODEL___GEOMETRY___GRAPH___VERTEX_H

#include <kinematic_model/geometry/graph/connection.hpp>

#include <vector>

namespace kinematic_model {
namespace geometry {
namespace graph {

/// \brief A type definition for a vertex ID.
typedef uint64_t vertex_id_t;

/// \brief A vertex in a connected graph.
struct vertex_t
{
    // CLASSES
    /// \brief A neighboring vertex connected to this vertex.
    struct neighbor_t
    {
        // CONSTRUCTORS
        /// \brief Instantiates a new neighbor.
        /// \param vertex The neighboring/connected vertex.
        /// \param attachment The attachment physically connecting the two objects/vertices.
        /// \param direction The direction of the connection.
        /// \note This constructor is mainly used for std emplace() methods.
        neighbor_t(vertex_t* const& vertex, const std::shared_ptr<attachment::attachment_t>& attachment, const connection_t::direction_t& direction);

        // VARIABLES
        /// \brief The neighboring, connected vertex.
        vertex_t* vertex;
        /// \brief The connection between this vertex and the neighboring vertex.
        connection_t connection;
    };

    // VARIABLES
    /// \brief The unique ID of the vertex.
    vertex_id_t id;
    /// \brief A list of vertices connected to this vertex.
    std::vector<neighbor_t> neighbors;
};

}}}

#endif