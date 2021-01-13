#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___OBJECT_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___OBJECT_H

#include <string>

namespace kinematic_model {
namespace geometry {
namespace object {

class object_t
{
public:
    enum class type_t
    {
        LINK = 0,
        FRAME = 1,
        JOINT = 2
    };

    std::string name() const;
    type_t object_type() const;

    void lock();

protected:
    object_t(const std::string& name, type_t type);

    bool is_locked() const;

private:
    std::string m_name;
    type_t m_object_type;

    bool m_locked;
};

}}}

#endif