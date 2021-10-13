/// \file kinematic_model/geometry/attachment/dtdr.hpp
/// \brief Defines the kinematic_model::geometry::attachment::dtdr_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DTDR_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DTDR_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief A dynamic translation, dynamic rotation (DTDR) attachment between two objects.
class dtdr_t
    : public attachment_t
{
public:
    // FACTORY
    /// \brief Creates a new dynamic translation, dynamic rotation (DTDR) attachment.
    /// \param state_index_x The state vector index of the translation x component.
    /// \param state_index_y The state vector index of the translation y component.
    /// \param state_index_z The state vector index of the translation z component.
    /// \param state_index_qw The state vector index of the quaternion rotation w component.
    /// \param state_index_qx The state vector index of the quaternion rotation x component.
    /// \param state_index_qy The state vector index of the quaternion rotation y component.
    /// \param state_index_qz The state vector index of the quaternion rotation z component.
    /// \returns A shared pointer to a DTDR attachment.
    static std::shared_ptr<dtdr_t> create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);

    // METHODS
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // CONSTRUCTORS
    /// \brief Instantiates an empty DTDR attachment.
    dtdr_t();
    dtdr_t(const dtdr_t&) = delete;

    // VARIABLES
    /// \brief The state vector index of the dynamic x component of the translation.
    uint32_t m_state_index_x;
    /// \brief The state vector index of the dynamic y component of the translation.
    uint32_t m_state_index_y;
    /// \brief The state vector index of the dynamic z component of the translation.
    uint32_t m_state_index_z;
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