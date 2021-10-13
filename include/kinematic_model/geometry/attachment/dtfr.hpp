/// \file kinematic_model/geometry/attachment/dtfr.hpp
/// \brief Defines the kinematic_model::geometry::attachment::dtfr_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DTFR_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___DTFR_H

#include <kinematic_model/geometry/attachment/attachment.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace attachment {

/// \brief A dynamic translation, fixed rotation (DTFR) attachment between two objects.
class dtfr_t
    : public attachment_t
{
public:
    // FACTORY
    /// \brief Creates a new dynamic translation, fixed rotation (DTFR) attachment.
    /// \param state_index_x The state vector index of the translation x component.
    /// \param state_index_y The state vector index of the translation y component.
    /// \param state_index_z The state vector index of the translation z component.
    /// \param qw The w component of the quaternion rotation.
    /// \param qx The x component of the quaternion rotation.
    /// \param qy The y component of the quaternion rotation.
    /// \param qz The z component of the quaternion rotation.
    /// \returns A shared pointer to the new DTFR attachment.
    static std::shared_ptr<dtfr_t> create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t qw, double_t qx, double_t qy, double_t qz);
    /// \brief Creates a new dynamic translation, fixed rotation (DTFR) attachment.
    /// \param state_index_x The state vector index of the translation x component.
    /// \param state_index_y The state vector index of the translation y component.
    /// \param state_index_z The state vector index of the translation z component.
    /// \param roll The roll component of the euler rotation.
    /// \param pitch The x component of the euler rotation.
    /// \param yaw The y component of the euler rotation.
    /// \returns A shared pointer to the new DTFR attachment.
    static std::shared_ptr<dtfr_t> create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t roll, double_t pitch, double_t yaw);
    
    // METHODS
    transform::transform_t get_transform(const Eigen::VectorXd& state_vector) const override;

private:
    // CONSTRUCTORS
    /// \brief Instantiates an empty DTFR attachment.
    dtfr_t();
    dtfr_t(const dtfr_t&) = delete;

    // VARIABLES
    /// \brief The state vector index of the dynamic x component of the translation.
    uint32_t m_state_index_x;
    /// \brief The state vector index of the dynamic y component of the translation.
    uint32_t m_state_index_y;
    /// \brief The state vector index of the dynamic z component of the translation.
    uint32_t m_state_index_z;
    /// \brief The fixed w component of the quaternion.
    double_t m_qw;
    /// \brief The fixed x component of the quaternion.
    double_t m_qx;
    /// \brief The fixed y component of the quaternion.
    double_t m_qy;
    /// \brief The fixed z component of the quaternion.
    double_t m_qz;
};

}}}

#endif