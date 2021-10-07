/// \file kinematic_model/geometry/attachment/ftdr.hpp
/// \brief Defines the kinematic_model::geometry::attachment::ftdr_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FTDR_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___FTDR_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief An fixed translation, dynamic rotation (FTDR) attachment between two objects.
class ftdr_t
    : public attachment_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new ftdr attachment object.
    /// \param x The x component of the translation.
    /// \param y The y component of the translation.
    /// \param z The z component of the translation.
    /// \param state_index_qw The state vector index of the quaternion rotation w component.
    /// \param state_index_qx The state vector index of the quaternion rotation x component.
    /// \param state_index_qy The state vector index of the quaternion rotation y component.
    /// \param state_index_qz The state vector index of the quaternion rotation z component.
    ftdr_t(double_t x, double_t y, double_t z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);
    
    // METHODS
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // VARIABLES
    /// \brief The fixed x component of the translation.
    double_t m_x;
    /// \brief The fixed y component of the translation.
    double_t m_y;
    /// \brief The fixed z component of the translation.
    double_t m_z;
    /// \brief The state vector index of the dynamic w component of the quaternion.
    uint32_t m_state_index_qw;
    /// \brief The state vector index of the dynamic x component of the quaternion.
    uint32_t m_state_index_qx;
    /// \brief The state vector index of the dynamic y component of the quaternion.
    uint32_t m_state_index_qy;
    /// \brief The state vector index of the dynamic z component of the quaternion.
    uint32_t m_state_index_qz;
};

}}}

#endif