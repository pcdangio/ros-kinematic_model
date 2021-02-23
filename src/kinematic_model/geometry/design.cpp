#include <kinematic_model/geometry/design.hpp>

#include <ros/console.h>

using namespace kinematic_model::geometry;

// FACTORY
std::shared_ptr<object::link_t> design_t::create_link(const std::string& name)
{
    return std::make_shared<object::link_t>(name);
}
std::shared_ptr<object::frame_t> design_t::create_frame(const std::string& name)
{
    return std::make_shared<object::frame_t>(name);
}
std::shared_ptr<object::joint_t> design_t::create_joint(const std::string& name, object::joint_t::type_t type, uint32_t state_index)
{
    return std::make_shared<object::joint_t>(name, type, state_index);
}

// ADDITION
bool design_t::add_object(const std::shared_ptr<object::object_t>& object)
{
    // Call base add_object() with empty parent/attachment.
    return design_t::add_object(object, nullptr, nullptr);
}
bool design_t::add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, double x, double y, double z, double roll, double pitch, double yaw)
{
    // Create a fixed attachment.

    // Create quaternion from euler rotations.
    Eigen::Quaterniond orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // Create attachment shared pointer.
    auto attachment = std::make_shared<attachment::fixed_t>(x, y, z, orientation.w(), orientation.x(), orientation.y(), orientation.z());

    // Call base add_object().
    return design_t::add_object(object, parent, attachment);
}
bool design_t::add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
{
    // Create dynamic attachment.
    auto attachment = std::make_shared<attachment::dynamic_t>(state_index_x, state_index_y, state_index_z, state_index_qw, state_index_qx, state_index_qy, state_index_qz);

    // Call base add_object.
    return design_t::add_object(object, parent, attachment);
}
bool design_t::add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, const std::shared_ptr<attachment::attachment_t>& attachment)
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

    // Lock the object so it can no longer be edited.
    object->lock();

    return true;
}

// ACCESS
std::vector<design_t::instruction_t> design_t::instructions() const
{
    return design_t::m_instructions;
}