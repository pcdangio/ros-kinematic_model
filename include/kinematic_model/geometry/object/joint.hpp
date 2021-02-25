/// \file kinematic_model/geometry/object/joint.hpp
/// \brief Defines the kinematic_model::geometry::object::joint_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___JOINT_H

#include <kinematic_model/geometry/object/object.hpp>
#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace object {

/// \brief A joint between two geometric objects.
class joint_t
    : public object_t,
      public attachment::attachment_t
{
public:
    // ENUMERATIONS
    /// \brief An enumeration of joint types.
    enum class type_t
    {
        REVOLUTE = 0,   ///< A 1DOF joint that revolves around an axis.
        PRISMATIC = 1   ///< A 1DOF joint that translates along an axis.
    };

    // CONSTRUCTORS
    /// \brief Instantiates a new joint.
    /// \param name The unique name of the joint.
    /// \param type The joint's type.
    /// \param state_index The index of the joint's state variable in the state vector.
    joint_t(const std::string& name, type_t type, uint32_t state_index);

    // METHODS
    /// \brief Sets the definition of the axis for the joint's degree of freedom.
    /// \param x The x component of the axis's unit vector.
    /// \param y The y component of the axis's unit vector.
    /// \param z The z component of the axis's unit vector.
    /// \returns TRUE if the axis was updated, otherwise FALSE if the joint is locked against editing.
    /// \note The unit vector definging the axis should have a magnitude of 1.
    bool set_axis_definition(double x, double y, double z);
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

    // PROPERTIES
    /// \brief Gets the joint's type.
    /// \returns The joints type.
    type_t joint_type() const;

private:
    // VARIABLES
    /// \brief The joint's type.
    const type_t m_joint_type;
    /// \brief The unit axis that the joint's degree of freedom operates on.
    Eigen::Vector3d m_axis_definition;
    /// \brief The index of the joint's state variable in the state vector.
    const uint32_t m_state_index;
};

}}}

#endif