#include <kinematic_model/model/design.hpp>

#include <ros/console.h>

using namespace kinematic_model::model;

bool design_t::add(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, const std::shared_ptr<geometry::attachment::attachment_t>& attachment)
{
    // Check that object exists.
    if(!object)
    {
        ROS_ERROR("failed to add object to model design (object is nullptr)");
        return false;
    }

    // Check if parent exists
    if(parent)
    {
        // Check that the parent exists in the design.
        bool parent_exists = false;
        for(auto instruction = design_t::m_instructions.cbegin(); instruction != design_t::m_instructions.cend(); ++instruction)
        {
            if(instruction->parent == parent)
            {
                parent_exists = true;
                break;
            }
        }

        // If this point reached and parent_exists is still false, parent does not exist.
        if(parent_exists)
        {
            ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (parent does not exist)");
            return false;
        }
    }

    // If a parent is given, ensure that an attachment is given.
    if(parent && !attachment)
    {
        ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (has parent but attachment is nullptr)");
        return false;
    }
    else if(!parent && attachment)
    {
        ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (has no parent and attachment is not nullptr)");
        return false;
    }

    // Add object to instructions.
    design_t::m_instructions.push_back({object, parent, attachment});
}

std::vector<kinematic_model::model::design_t::instruction_t> design_t::instructions() const
{
    return design_t::m_instructions;
}