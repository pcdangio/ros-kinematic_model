/// \file kinematic_model/geometry/design.hpp
/// \brief Defines the kinematic_model::geometry::design_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___DESIGN_H
#define KINEMATIC_MODEL___GEOMETRY___DESIGN_H

#include <kinematic_model/geometry/object/link.hpp>
#include <kinematic_model/geometry/object/frame.hpp>
#include <kinematic_model/geometry/object/joint.hpp>

#include <kinematic_model/geometry/attachment/fixed.hpp>
#include <kinematic_model/geometry/attachment/dynamic.hpp>

#include <vector>
#include <memory>

namespace kinematic_model {
/// \brief Geometric objects and tools for modeling the kinematics of a system.
namespace geometry {

/// \brief An tool for specifying the geometric design of a model.
class design_t
{
public:
    // OBJECTS
    /// \brief Represents a single line instruction in the design.
    struct instruction_t
    {
        /// \brief The object that is included in the design.
        std::shared_ptr<object::object_t> object;
        /// \brief The parent that the object is attached to.
        std::shared_ptr<object::object_t> parent;
        /// \brief The attachment connecting the object to the parent.
        std::shared_ptr<attachment::attachment_t> attachment;
    };

    // FACTORY
    /// \brief Creates a new link object to add to the design.
    /// \param name The name to give the new link.
    /// \returns A shared pointer to the new link object. 
    static std::shared_ptr<object::link_t> create_link(const std::string& name);
    /// \brief Creates a new frame object to add to the design.
    /// \param name The name to give the new frame.
    /// \returns A shared pointer to the new frame object. 
    static std::shared_ptr<object::frame_t> create_frame(const std::string& name);
    /// \brief Creates a new joint object to add to the design.
    /// \param name The name to give the new joint.
    /// \param type The joint type to assign the new joint.
    /// \param state_index The index of the variable in the model's state vector that represents this joint's position.
    /// \returns A shared pointer to the new joint object.
    static std::shared_ptr<object::joint_t> create_joint(const std::string& name, object::joint_t::type_t type, uint32_t state_index);

    // ADDITION
    /// \brief Adds an object to the design without a parent.
    /// \param object The object to add.
    /// \returns TRUE if the object was added successfully, otherwise FALSE.
    bool add_object(const std::shared_ptr<object::object_t>& object);
    /// \brief Adds an object to the design with a fixed attachment to a parent.
    /// \param object The object to add.
    /// \param parent The parent to attach the object to.
    /// \param x The fixed x position of the object's frame relative to the parent's frame.
    /// \param y The fixed y position of the object's frame relative to the parent's frame.
    /// \param z The fixed z position of the object's frame relative to the parent's frame.
    /// \param roll The fixed roll (x) angle, in radians, of the object's frame relative to the parent's frame.
    /// \param pitch The fixed pitch (y) angle, in radians, of the object's frame relative to the parent's frame.
    /// \param yaw The fixed yaw (z) angle, in radians, of the object's frame relative to the parent's frame.
    /// \returns TRUE if the object was added successfully, otherwise FALSE.
    bool add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, double x, double y, double z, double roll, double pitch, double yaw);
    /// \brief Adds an object to the design with a dynamic attachment to a parent.
    /// \param object The object to add.
    /// \param parent The parent to attach the object to.
    /// \param state_index_x The index of the model state variable that indicates the x position of the object's frame relative to the parent's frame.
    /// \param state_index_y The index of the model state variable that indicates the y position of the object's frame relative to the parent's frame.
    /// \param state_index_z The index of the model state variable that indicates the z position of the object's frame relative to the parent's frame.
    /// \param state_index_qw The index of the model state variable that indicates the qw quaternion rotation of the object's frame relative to the parent's frame.
    /// \param state_index_qx The index of the model state variable that indicates the qx quaternion rotation of the object's frame relative to the parent's frame.
    /// \param state_index_qy The index of the model state variable that indicates the qy quaternion rotation of the object's frame relative to the parent's frame.
    /// \param state_index_qz The index of the model state variable that indicates the qz quaternion rotation of the object's frame relative to the parent's frame.
    /// \returns TRUE if the object was added successfully, otherwise FALSE.
    bool add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);
    
    // ACCESS
    /// \brief Gets the current set of instructions that specify the model's design.
    /// \returns A vector of instructions.
    std::vector<instruction_t> instructions() const;

private:
    // VARIABLES
    /// \brief The instructions that specify the model's design.
    std::vector<instruction_t> m_instructions;

    // ADDITION
    /// \brief A base method for adding objects to the design.
    /// \param object The object to add.
    /// \param parent The parent to attach the object to.
    /// \param attachment The attachment to use between the parent and the object.
    bool add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, const std::shared_ptr<attachment::attachment_t>& attachment);
};

}}

#endif