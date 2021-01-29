/// \file kinematic_model/geometry/attachment/fixed.hpp
/// \brief Defines the kinematic_model::geometry::attachment::fixed_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FIXED_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FIXED_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief A fixed attachment between two geometry objects.
class fixed_t
    : public attachment_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a fixed attachment using a 3D translation and rotation.
    /// \param x The x component of the translation.
    /// \param y The y component of the translation.
    /// \param z The z component of the translation.
    /// \param qw The w component of the quaternion rotation.
    /// \param qx The x component of the quaternion rotation.
    /// \param qy The y component of the quaternion rotation.
    /// \param qz The z component of the quaternion rotation.
    fixed_t(double x, double y, double z, double qw, double qx, double qy, double qz);
    
    // METHODS
    transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // VARIABLES
    /// \brief The fixed parent->child transform of the attachment.
    transform_t m_transform;
};

}}}

#endif