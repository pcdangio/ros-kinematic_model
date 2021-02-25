/// \file kinematic_model/geometry/attachment/attachment.hpp
/// \brief Defines the kinematic_model::geometry::attachment::attachment_t class
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___ATTACHMENT_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___ATTACHMENT_H

#include <transform/transform.hpp>

namespace kinematic_model {
namespace geometry {
/// \brief Attachments representing physical connections between geometry objects.
namespace attachment {

/// \brief A base class representing a physical connection between two geometry objects.
class attachment_t
{
public:
    // ENUMERATIONS
    /// \brief A type specifier for an attachment.
    enum class type_t
    {
        FIXED = 0,      ///< The attachment is fixed and does not change over time.
        DYNAMIC = 1     ///< The attachment is dynamic and does change over time.
    };

    // METHODS
    /// \brief Gets the parent-to-child transform between the two attached objects.
    /// \param state_vector The current state vector to evaluate the transform with.
    /// \returns The parent-to-child transform.
    virtual transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const = 0;

    // PROPERTIES
    /// \brief Gets the attachment's type.
    /// \returns The attachment's type.
    type_t type() const;
    
protected:
    // CONSTRUCTORS
    /// \brief Instantiates a new attachment.
    /// \param type The type of attachment to create.
    attachment_t(type_t type);

private:
    // VARIABLES
    /// \brief The attachment's type.
    const type_t m_type;
};

}}}

#endif