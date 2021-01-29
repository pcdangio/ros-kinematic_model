#ifndef KINEMATIC_MODEL___GEOMETRY___GEOMETRY_H
#define KINEMATIC_MODEL___GEOMETRY___GEOMETRY_H

#include <kinematic_model/model_plugin.hpp>
#include <kinematic_model/geometry/graph/graph.hpp>

namespace kinematic_model {
namespace geometry {

class geometry_t
{
public:
    bool initialize(const std::shared_ptr<model_plugin_t>& model_plugin);

private:
    graph::graph_t m_graph;
};

}}

#endif