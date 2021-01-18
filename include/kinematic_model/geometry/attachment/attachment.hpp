#ifndef KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___ATTACHMENT_H
#define KINEMATIC_MODEL___GEOMETRY___ATTACHMENT___ATTACHMENT_H

#include <kinematic_model/geometry/transform.hpp>

#include <memory>

namespace kinematic_model {
namespace geometry {
namespace attachment {

class attachment_t
{
public:
    enum class type_t
    {
        FIXED = 0,
        DYNAMIC = 1
    };

    virtual transform_t get_transform(const Eigen::VectorXd& state_vector) const = 0;

    type_t type() const;
    
protected:
    attachment_t(type_t type);

private:
    type_t m_type;
};

}}}

#endif