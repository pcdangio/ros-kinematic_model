#include <kinematic_model/geometry/graph/connection.hpp>

using namespace kinematic_model::geometry::graph;

// CONSTRUCTORS
connection_t::connection_t(const std::shared_ptr<attachment::attachment_t>& attachment, const direction_t& direction)
{
    connection_t::attachment = attachment;
    connection_t::direction = direction;
}