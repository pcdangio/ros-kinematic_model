/// \file kinematic_model/geometry/design.hpp
/// \brief Defines the kinematic_model::geometry::design_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___DESIGN_H
#define KINEMATIC_MODEL___GEOMETRY___DESIGN_H

#include <kinematic_model/geometry/object/link.hpp>
#include <kinematic_model/geometry/object/frame.hpp>
#include <kinematic_model/geometry/object/joint.hpp>

#include <kinematic_model/geometry/attachment/attachment.hpp>

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

    /// \brief Creates a new attachment between objects with a fixed translation / fixed rotation (FTFR).
    /// \param x The fixed x translation component.
    /// \param y The fixed y translation component.
    /// \param z The fixed z translation component.
    /// \param qw The fixed w quaternion rotation component.
    /// \param qx The fixed x quaternion rotation component.
    /// \param qy The fixed y quaternion rotation component.
    /// \param qz The fixed z quaternion rotation component.
    /// \returns A shared pointer to the new attachment.
    static std::shared_ptr<attachment::attachment_t> create_attachment_ftfr(double_t x, double_t y, double_t z, double_t qw, double_t qx, double_t qy, double_t qz);
    /// \brief Creates a new attachment between objects with a fixed translation / fixed rotation (FTFR).
    /// \param x The fixed x translation component.
    /// \param y The fixed y translation component.
    /// \param z The fixed z translation component.
    /// \param roll The fixed roll rotation component.
    /// \param pitch The fixed pitch rotation component.
    /// \param yaw The fixed yaw rotation component.
    /// \returns A shared pointer to the new attachment.
    static std::shared_ptr<attachment::attachment_t> create_attachment_ftfr(double_t x, double_t y, double_t z, double_t roll, double_t pitch, double_t yaw);
    static std::shared_ptr<attachment::attachment_t> create_attachment_ftdr(double_t x, double_t y, double_t z, uint32_t qw, uint32_t qx, uint32_t qy, uint32_t qz);
    static std::shared_ptr<attachment::attachment_t> create_attachment_dtfr(uint32_t x, uint32_t y, uint32_t z, double_t qw, double_t qx, double_t qy, double_t qz);
    static std::shared_ptr<attachment::attachment_t> create_attachment_dtfr(uint32_t x, uint32_t y, uint32_t z, double_t roll, double_t pitch, double_t yaw);
    static std::shared_ptr<attachment::attachment_t> create_attachment_dtdr(uint32_t x, uint32_t y, uint32_t z, uint32_t qw, uint32_t qx, uint32_t qy, uint32_t qz);


    // ADDITION
    /// \brief Adds an object to the design without a parent.
    /// \param object The object to add.
    /// \returns TRUE if the object was added successfully, otherwise FALSE.
    bool add_object(const std::shared_ptr<object::object_t>& object);
    /// \brief Adds an object to the design by attaching it to a parent.
    /// \param object The object to add.
    /// \param parent The parent to attach the object to.
    /// \param attachment The attachment from the parent to the object.
    bool add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, const std::shared_ptr<attachment::attachment_t>& attachment);
    
    
    // ACCESS
    /// \brief Gets the current set of instructions that specify the model's design.
    /// \returns A vector of instructions.
    std::vector<instruction_t> instructions() const;

private:
    // VARIABLES
    /// \brief The instructions that specify the model's design.
    std::vector<instruction_t> m_instructions;
};

}}

#endif