#include <kinematic_model/geometry/transform.h>

using namespace kinematic_model::geometry;

// CONSTRUCTORS
transform_t::transform_t()
{
    transform_t::m_translation.setZero();
    transform_t::m_rotation.setIdentity();
}
transform_t::transform_t(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
{
    transform_t::m_translation = translation;
    transform_t::m_rotation = rotation;
}
transform_t::transform_t(const Eigen::Vector3d& translation)
{
    transform_t::m_translation = translation;
    transform_t::m_rotation.setIdentity();
}
transform_t::transform_t(const Eigen::Quaterniond& rotation)
{
    transform_t::m_translation.setZero();
    transform_t::m_rotation = rotation;
}

// METHODS
void transform_t::transform(transform_t& transform) const
{
    // Apply this transform's rotation to the original transform.
    transform.m_rotation = transform_t::m_rotation * transform.m_rotation;
    transform.m_translation = transform_t::m_rotation * transform.m_translation;
    // Normalize the transformed rotation for numerical stability.
    transform.m_rotation.normalize();

    // Add this transform's translation to the now rotated original translation.
    transform.m_translation += transform_t::m_translation;
}
void transform_t::transform(pose_t& pose) const
{
    // Rotate the original pose.
    pose.m_orientation = transform_t::m_rotation * pose.m_orientation;
    pose.m_position = transform_t::m_rotation * pose.m_position;
    // Normalize the rotated orientation for numerical stability.
    pose.m_orientation.normalize();

    // Translate the rotated pose.
    pose.m_position += transform_t::m_translation;
}
transform_t transform_t::inverse() const
{
    // Create output transform.
    // NOTE: This method allows Eigen to do operations in place.
    transform_t inverted;

    // Invert the translation.
    inverted.m_translation = transform_t::m_translation * -1.0;

    // Invert the rotation.
    inverted.m_rotation = transform_t::m_rotation.inverse();

    return inverted;
}