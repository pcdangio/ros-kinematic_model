#include <kinematic_model/geometry/transform.hpp>

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

// MODIFIERS
void transform_t::reset()
{
    transform_t::m_translation.setZero();
    transform_t::m_rotation.setIdentity();
}
void transform_t::invert()
{
    // Invert the translation.
    transform_t::m_translation = transform_t::m_translation * -1.0;

    // Invert the rotation.
    transform_t::m_rotation = transform_t::m_rotation.inverse();
}

// APPLICATIONS
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

const Eigen::Vector3d& transform_t::translation() const
{
    return transform_t::m_translation;
}
const Eigen::Quaterniond& transform_t::rotation() const
{
    return transform_t::m_rotation;
}