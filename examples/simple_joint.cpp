#include <kinematic_model/kinematic_model.hpp>

using namespace kinematic_model;

class simple_joint_t
    : public kinematic_model_t
{
public:
    simple_joint_t()
        : kinematic_model_t(1, 1)
    {
        // Initialize state estimation parameters.
        simple_joint_t::Q(0,0) = 0.0001;
        simple_joint_t::R(0,0) = 0.01;
    }
private:
    void build_geometry(geometry::design_t& design) const override
    {
        auto link_a = design.create_link("link_a");
        auto link_b = design.create_link("link_b");
        auto joint_ab = design.create_joint("joint_ab", geometry::object::joint_t::type_t::REVOLUTE, 0);

        design.add_object(link_a);
        design.add_object(link_b, link_a, joint_ab);
    }
    void state_transition(const Eigen::VectorXd& xp, const Eigen::VectorXd& q, Eigen::VectorXd& x) const
    {
        x(0) = xp(0) + q(0);
    }
    void observation(const Eigen::VectorXd& x, const Eigen::VectorXd& r, Eigen::VectorXd& z) const
    {
        z(0) = x(0) + r(0);
    }
};

REGISTER_PLUGIN(simple_joint_t)