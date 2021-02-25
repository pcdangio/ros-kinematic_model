/// \file kinematic_model/geometry/attachment/dynamic.hpp
/// \brief Defines the kinematic_model::geometry::attachment::dynamic_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DYNAMIC_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DYNAMIC_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief A 6DOF dynamic attachment between two geometry objects.
class dynamic_t
    : public attachment_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new dynamic attachment as a state-dependant translation and quaternion rotation.
    /// \param state_index_x The index of the x translation component within the state vector.
    /// \param state_index_y The index of the y translation component within the state vector.
    /// \param state_index_z The index of the z translation component within the state vector.
    /// \param state_index_qw The index of the w quaternion component within the state vector.
    /// \param state_index_qx The index of the x quaternion component within the state vector.
    /// \param state_index_qy The index of the y quaternion component within the state vector.
    /// \param state_index_qz The index of the z quaternion component within the state vector.
    dynamic_t(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);

    // METHODS
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // VARIABLES
    /// \brief The index of the x translation component within the state vector.
    uint32_t m_state_index_x;
    /// \brief The index of the y translation component within the state vector.
    uint32_t m_state_index_y;
    /// \brief The index of the z translation component within the state vector.
    uint32_t m_state_index_z;
    /// \brief The index of the w quaternion component within the state vector.
    uint32_t m_state_index_qw;
    /// \brief The index of the x quaternion component within the state vector.
    uint32_t m_state_index_qx;
    /// \brief The index of the y quaternion component within the state vector.
    uint32_t m_state_index_qy;
    /// \brief The index of the z quaternion component within the state vector.
    uint32_t m_state_index_qz;
};

}}}

#endif