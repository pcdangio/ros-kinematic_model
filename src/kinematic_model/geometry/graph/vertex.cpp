#include <kinematic_model/geometry/graph/vertex.hpp>

using namespace kinematic_model::geometry::graph;

// CONSTRUCTORS
vertex_t::neighbor_t::neighbor_t(vertex_t* const& vertex, const std::shared_ptr<attachment::attachment_t>& attachment, const connection_t::direction_t& direction)
    : connection(attachment, direction)
{
    vertex_t::neighbor_t::vertex = vertex;
}