/// \file kinematic_model/geometry/graph/path.hpp
/// \brief Defines the kinematic_model::geometry::graph::path_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___GRAPH___PATH_H
#define KINEMATIC_MODEL___GEOMETRY___GRAPH___PATH_H

#include <kinematic_model/geometry/graph/connection.hpp>

#include <vector>

namespace kinematic_model {
namespace geometry {
namespace graph {

/// \brief A type definition for a geometric path through a kinematic chain.
typedef std::vector<connection_t> path_t;

}}}

#endif