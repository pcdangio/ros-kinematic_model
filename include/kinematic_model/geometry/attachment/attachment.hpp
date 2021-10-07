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
        FTFR = 0,   ///< A fixed translation, fixed rotation (FTFR) attachment.
        FTDR = 1,   ///< A fixed translation, dynamic rotation (FTDR) attachment.
        DTFR = 2,   ///< A dynamic translation, fixed rotation (DTFR) attachment.
        DTDR = 3    ///< A dynamic translation, dynamic rotation (DTDR) attachment.
    };

    // METHODS
    /// \brief Gets the child-to-parent transform between the two attached objects.
    /// \param state_vector The current state vector to evaluate the transform with.
    /// \returns The child-to-parent transform.
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