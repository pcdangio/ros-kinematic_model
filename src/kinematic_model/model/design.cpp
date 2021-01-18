#include <kinematic_model/model/design.hpp>

#include <ros/console.h>

using namespace kinematic_model::model;

bool design_t::add_object(const std::shared_ptr<geometry::object::object_t>& object)
{
    return design_t::add_object(object, nullptr, nullptr);
}
bool design_t::add_object(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, const std::shared_ptr<geometry::attachment::attachment_t>& attachment)
{
    // Check that object exists.
    if(!object)
    {
        ROS_ERROR("failed to add object to model design (object is nullptr)");
        return false;
    }

    // Check that object with same name doesn't already exist.
    for(auto instruction = design_t::m_instructions.cbegin(); instruction != design_t::m_instructions.cend(); ++instruction)
    {
        if(instruction->object->name() == object->name())
        {
            ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (object with same name already exists)");
            return false;
        }
    }

    // If a parent is given, ensure that it exists in the design.
    if(parent)
    {
        bool parent_exists = false;
        for(auto instruction = design_t::m_instructions.cbegin(); instruction != design_t::m_instructions.cend(); ++instruction)
        {
            if(instruction->object == parent)
            {
                parent_exists = true;
                break;
            }
        }

        // If this point reached and parent_exists is still false, parent does not exist.
        if(!parent_exists)
        {
            ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (parent does not exist)");
            return false;
        }
    }

    // If a parent is given, ensure that an attachment is given.
    if(parent && !attachment)
    {
        ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (given parent with nullptr attachment)");
        return false;
    }
    else if(!parent && attachment)
    {
        ROS_ERROR_STREAM("failed to add object [" << object->name() << "] to model design (cannot use attachment on nullptr parent)");
        return false;
    }

    // Add object to instructions.
    design_t::m_instructions.push_back({object, parent, attachment});

    return true;
}

std::vector<kinematic_model::model::design_t::instruction_t> design_t::instructions() const
{
    return design_t::m_instructions;
}