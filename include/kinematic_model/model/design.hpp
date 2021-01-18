#ifndef KINEMATIC_MODEL___MODEL___DESIGN_H
#define KINEMATIC_MODEL___MODEL___DESIGN_H

#include <kinematic_model/geometry/object/objects.hpp>
#include <kinematic_model/geometry/attachment/attachments.hpp>

#include <vector>
#include <memory>

namespace kinematic_model {
namespace model {

class design_t
{
public:
    struct instruction_t
    {
        std::shared_ptr<geometry::object::object_t> object;
        std::shared_ptr<geometry::object::object_t> parent;
        std::shared_ptr<geometry::attachment::attachment_t> attachment;
    };

    std::vector<instruction_t> instructions() const;

    bool add_object(const std::shared_ptr<geometry::object::object_t>& object);
    bool add_object(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, const std::shared_ptr<geometry::attachment::attachment_t>& attachment);

private:
    std::vector<instruction_t> m_instructions;
};

}}

#endif