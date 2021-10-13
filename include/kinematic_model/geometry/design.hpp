/// \file kinematic_model/geometry/design.hpp
/// \brief Defines the kinematic_model::geometry::design_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___DESIGN_H
#define KINEMATIC_MODEL___GEOMETRY___DESIGN_H

#include <kinematic_model/geometry/object/link.hpp>
#include <kinematic_model/geometry/object/frame.hpp>
#include <kinematic_model/geometry/object/joint.hpp>

#include <kinematic_model/geometry/attachment/ftfr.hpp>
#include <kinematic_model/geometry/attachment/ftdr.hpp>
#include <kinematic_model/geometry/attachment/dtfr.hpp>
#include <kinematic_model/geometry/attachment/dtdr.hpp>

#include <vector>
#include <memory>

namespace kinematic_model {
/// \brief Geometric objects and tools for modeling the kinematics of a system.
namespace geometry {

/// \brief An tool for specifying the geometric design of a model.
class design_t
{
public:
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
    
    // INSTRUCTIONS
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