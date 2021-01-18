#ifndef KINEMATIC_MODEL___MODEL___DESIGN_H
#define KINEMATIC_MODEL___MODEL___DESIGN_H

#include <kinematic_model/geometry/object/link.hpp>
#include <kinematic_model/geometry/object/frame.hpp>
#include <kinematic_model/geometry/object/joint.hpp>

#include <kinematic_model/geometry/attachment/fixed.hpp>
#include <kinematic_model/geometry/attachment/dynamic.hpp>

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

    static std::shared_ptr<geometry::object::link_t> create_link(const std::string& name);
    static std::shared_ptr<geometry::object::frame_t> create_frame(const std::string& name);
    static std::shared_ptr<geometry::object::joint_t> create_joint(const std::string& name, geometry::object::joint_t::type_t type, uint32_t state_index);

    bool add_object(const std::shared_ptr<geometry::object::object_t>& object);
    bool add_object(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, double x, double y, double z, double roll, double pitch, double yaw);
    bool add_object(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz);
    bool add_object(const std::shared_ptr<geometry::object::object_t>& object, const std::shared_ptr<geometry::object::object_t>& parent, const std::shared_ptr<geometry::attachment::attachment_t>& attachment);

    std::vector<instruction_t> instructions() const;

private:
    std::vector<instruction_t> m_instructions;
};

}}

#endif