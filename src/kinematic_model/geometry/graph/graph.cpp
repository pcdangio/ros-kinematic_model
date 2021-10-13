#include <kinematic_model/geometry/graph/graph.hpp>

using namespace kinematic_model::geometry::graph;

// CONSTRUCTORS
graph_t::~graph_t()
{
    // Clear the graph to free resources.
    graph_t::clear();
}

// METHODS
void graph_t::build(const design_t& model_design)
{
    // Clear graph if it's already been populated.
    graph_t::clear();

    // Get the design's instructions.
    auto instructions = model_design.instructions();

    // Iterate through design to populate the graph.
    for(auto instruction = instructions.cbegin(); instruction != instructions.cend(); ++instruction)
    {
        // Create new vertex for object.
        vertex_t* vertex = new vertex_t;
        vertex->id = graph_t::m_vertices.size();

        // Add object's vertex to vertex map.
        std::string object_name = instruction->object->name();
        // If object is a joint, append _parent to it's primary vertex.
        if(instruction->object->object_type() == object::object_t::type_t::JOINT)
        {
            object_name += "_parent";
        }
        graph_t::m_vertices.emplace(object_name, vertex);

        // Check if object has a parent.
        if(instruction->parent)
        {
            // Get parent name.
            std::string parent_name = instruction->parent->name();

            // Connect vertex with it's parent.
            auto parent_vertex = graph_t::m_vertices.at(parent_name);
            parent_vertex->neighbors.emplace_back(vertex, instruction->attachment, connection_t::direction_t::PARENT_CHILD);
            vertex->neighbors.emplace_back(parent_vertex, instruction->attachment, connection_t::direction_t::CHILD_PARENT);
        }

        // If object is a joint, add a second vertex for the joint_child frame.
        if(instruction->object->object_type() == object::object_t::type_t::JOINT)
        {
            // Create vertex for joint_child frame.
            vertex_t* vertex_jc = new vertex_t;
            vertex_jc->id = graph_t::m_vertices.size();
            
            // Add joint_child vertex to vertex map.
            graph_t::m_vertices.emplace(instruction->object->name() + "_child", vertex_jc);

            // Connect joint_child with the joint_parent frame.
            // Get joint as attachment object.
            auto joint_attachment = std::dynamic_pointer_cast<attachment::attachment_t>(instruction->object);
            // Attach parent to child and child to parent.
            vertex->neighbors.emplace_back(vertex_jc, joint_attachment, connection_t::direction_t::PARENT_CHILD);
            vertex_jc->neighbors.emplace_back(vertex, joint_attachment, connection_t::direction_t::CHILD_PARENT);
        }
    }
}
void graph_t::clear()
{
    // Clear the path cache.
    graph_t::m_path_cache.clear();

    // Iterate through map and delete vertex pointers.
    for(auto vertex_entry = graph_t::m_vertices.begin(); vertex_entry != graph_t::m_vertices.end(); ++vertex_entry)
    {
        delete vertex_entry->second;
    }

    // Empty the vertex map.
    graph_t::m_vertices.clear();
}
std::shared_ptr<path_t> graph_t::solve_path(const std::string& source_object, const std::string& destination_object) const
{
    // Check the path cache to see if this path has already been solved.
    std::string cache_key = source_object + ":" + destination_object;
    auto path_cache_iterator = graph_t::m_path_cache.find(cache_key);
    if(path_cache_iterator != graph_t::m_path_cache.end())
    {
        // Return the cached solved path.
        return path_cache_iterator->second;
    }

    // If this point reached, path must be solved.

    // Get pointer to source vertex.
    auto source_vertex_iterator = graph_t::m_vertices.find(source_object);
    if(source_vertex_iterator == graph_t::m_vertices.end())
    {
        return nullptr;
    }
    auto source_vertex = source_vertex_iterator->second;

    // Get pointer to destination vertex.
    auto destination_vertex_iterator = graph_t::m_vertices.find(destination_object);
    if(destination_vertex_iterator == graph_t::m_vertices.end())
    {
        return nullptr;
    }
    auto destination_vertex = destination_vertex_iterator->second;

    // Create path vertices list.
    std::vector<vertex_id_t> current_visited;

    // Create the currently tracked path.
    path_t current_path;

    // Create output path.
    std::shared_ptr<path_t> solved_path = nullptr;

    // Start recursion from source vertex.
    current_visited.push_back(source_vertex->id);
    graph_t::solve_path(source_vertex, destination_vertex->id, current_visited, current_path, solved_path);

    // Add solved path to the cache.
    graph_t::m_path_cache[cache_key] = solved_path;

    // Return solved path.
    return solved_path;
}
void graph_t::solve_path(vertex_t* vertex, const vertex_id_t& destination, std::vector<vertex_id_t>& current_visited, path_t& current_path, std::shared_ptr<path_t>& solved_path) const
{
    // Check if current vertex is the destination vertex.
    if(vertex->id == destination)
    {
        // Update the solved path if necessary.
        // First check if solved path even exists yet.
        if(!solved_path)
        {
            // Solved path does not exist. Create and copy from current path.
            solved_path = std::make_shared<path_t>(current_path);
        }
        // Next, check if current path is shorter than solved path.
        else if(current_path.size() < solved_path->size())
        {
            // Current path is shorter; updated solved path.
            *solved_path = current_path;
        }
    }
    // Otherwise, continue recursing through this vertex's neighbors.
    else
    {
        for(auto neighbor = vertex->neighbors.cbegin(); neighbor != vertex->neighbors.cend(); ++neighbor)
        {
            // Verify that this neighbor is not in the current path.
            if(std::find(current_visited.cbegin(), current_visited.cend(), neighbor->vertex->id) == current_visited.cend())
            {
                // Neighbor is not in the current path. Recurse.
                // Add neighbor to current path.
                current_visited.emplace_back(neighbor->vertex->id);
                current_path.emplace_back(neighbor->connection);
                // Recurse.
                graph_t::solve_path(neighbor->vertex, destination, current_visited, current_path, solved_path);
                // Pop neighbor off current path.
                current_path.pop_back();
                current_visited.pop_back();
            }
        }
    }
}