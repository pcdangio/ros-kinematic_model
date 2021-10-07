/// \file kinematic_model/geometry/attachment/ftfr.hpp
/// \brief Defines the kinematic_model::geometry::attachment::ftfr_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FTFR_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FTFR_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief An fixed translation, fixed rotation (FTFR) attachment between two objects.
class ftfr_t
    : public attachment_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new FTFR attachment object.
    /// \param x The x component of the translation.
    /// \param y The y component of the translation.
    /// \param z The z component of the translation.
    /// \param qw The w component of the quaternion rotation.
    /// \param qx The x component of the quaternion rotation.
    /// \param qy The y component of the quaternion rotation.
    /// \param qz The z component of the quaternion rotation.
    ftfr_t(double x, double y, double z, double qw, double qx, double qy, double qz);
    
    // METHODS
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // VARIABLES
    /// \brief The fixed child->parent transform of the attachment.
    transform::transform_t m_transform;
};

}}}

#endif