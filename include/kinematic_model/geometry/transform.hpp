/// \file kinematic_model/geometry/transform.hpp
/// \brief Defines the kinematic_model::geometry::transform_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___TRANSFORM_H
#define KINEMATIC_MODEL___GEOMETRY___TRANSFORM_H

#include <eigen3/Eigen/Dense>

namespace kinematic_model {
/// \brief Geometric objects and tools for modeling the kinematics of a system.
namespace geometry {

/// \brief Represents a 3D transformation between coordinate frames.
class transform_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new identity transformation_t object.
    transform_t();
    /// \brief Instantiates a new transformation_t object.
    /// \param translation The translation component of the transformation.
    /// \param rotation The rotation component of the transformation.
    transform_t(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
    /// \brief Instantiates a new transformation_t object with an identity rotation.
    /// \param translation The translation component of the transformation.
    transform_t(const Eigen::Vector3d& translation);
    /// \brief Instantiates a new transformation_t object with an identity translation.
    /// \param rotation The rotation component of the transformation.
    transform_t(const Eigen::Quaterniond& rotation);

    // MODIFIERS
    /// \brief Calculates the inverse of the transform.
    /// \returns The inverted transform.
    transform_t inverse() const;

    // APPLICATIONS
    /// \brief Performs an in-place chain on top of another transform.
    /// \param transform The transform to chain in place.
    void transform(transform_t& transform) const;

    // ACCESS
    /// \brief Gets the translation component of this transform.
    /// \returns A const reference to the translation component.
    const Eigen::Vector3d& translation() const;
    /// \brief Gets the rotation component of this transform.
    /// \returns A const reference to the rotation component.
    const Eigen::Quaterniond& rotation() const;
    
private:
    // VARIABLES
    /// \brief The transform's translation component.
    Eigen::Vector3d m_translation;
    /// \brief The transform's rotation component.
    Eigen::Quaterniond m_rotation;
};

}}

#endif