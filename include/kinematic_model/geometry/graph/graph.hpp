/// \file kinematic_model/geometry/graph/graph.hpp
/// \brief Defines the kinematic_model::geometry::graph::graph_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___GRAPH___GRAPH_H
#define KINEMATIC_MODEL___GEOMETRY___GRAPH___GRAPH_H

#include <kinematic_model/geometry/design.hpp>
#include <kinematic_model/geometry/graph/vertex.hpp>
#include <kinematic_model/geometry/graph/connection.hpp>
#include <kinematic_model/geometry/graph/path.hpp>

#include <unordered_map>

namespace kinematic_model {
namespace geometry {
/// \brief Components for implementing a connected graph.
namespace graph {

/// \brief A connected graph.
class graph_t
{
public:
    // CONSTRUCTORS
    ~graph_t();

    /// METHODS
    /// \brief Builds the graph from a geometric model design.
    /// \param model_design The design instructions for the geometric model.
    void build(const design_t& model_design);
    /// \brief Clears the graph by removing all vertices.
    void clear();
    /// \brief Solves a path between two geometric objects in the kinematic graph.
    /// \param source_object The name of the object to start the path from.
    /// \param destination_object The name of the object to find a path to.
    /// \returns The solved path from source to destination.
    /// Returns NULLPTR if source and/or destination don't exist, or the path could not be found.
    std::shared_ptr<path_t> solve_path(const std::string& source_object, const std::string& destination_object) const;

private:
    // VARIABLES
    /// \brief Contains the graph vertices and aligns them to unique object names.
    std::unordered_map<std::string, vertex_t*> m_vertices;

    // METHODS
    /// \brief A recursive method implementing a depth-first search through the graph.
    /// \param vertex The current vertex to scan.
    /// \param destination The ID of the destination vertex being searched for.
    /// \param current_visited The list of vertex IDs that have been visited along the current path.
    /// \param current_path The current path from the source vertex to the current vertex.
    /// \param solved_path Stores the current minimum path from the source vertex to the destination.
    void solve_path(vertex_t* vertex, const vertex_id_t& destination, std::vector<vertex_id_t>& current_visited, path_t& current_path, std::shared_ptr<path_t>& solved_path) const;
};

}}}

#endif