/// \file kinematic_model/geometry/object/object.hpp
/// \brief Defines the kinematic_model::geometry::object::object_t class.
#ifndef KINEMATIC_MODEL___GEOMETRY___OBJECT___OBJECT_H
#define KINEMATIC_MODEL___GEOMETRY___OBJECT___OBJECT_H

#include <string>

namespace kinematic_model {
namespace geometry {
/// \brief Geometric objects that make up a kinematic system.
namespace object {

/// \brief A base geometric object.
class object_t
{
public:
    // ENMUERATIONS
    /// \brief An enumeration of object types.
    enum class type_t
    {
        LINK = 0,   ///< A physical link in a kinematic chain.
        FRAME = 1,  ///< A reference coordinate frame attached to the kinematic chain.
        JOINT = 2   ///< A joint connecting two links in the kinematic chain.
    };

    // CONSTRUCTORS
    /// \note Needed to make object_t polymorphic as viewed by compiler.
    virtual ~object_t(){};

    // METHODS
    /// \brief Locks the object to prevent it from further editing.
    void lock();
    
    // PROPERTIES
    /// \brief Gets the name of the object.
    /// \returns The name of the object.
    std::string name() const;
    /// \brief Gets the object's type.
    /// \returns The object's type.
    type_t object_type() const;
    /// \brief Indicates if the object is locked for editing.
    /// \returns TRUE if the object is locked and can no longer be edited, otherwise FALSE.
    bool is_locked() const;

protected:
    // CONSTRUCTORS
    /// \brief Instantiates a new object.
    /// \param name The object's unique name.
    /// \param type The object's type.
    object_t(const std::string& name, type_t type);    

private:
    // VARIABLES
    /// \brief The object's name.
    const std::string m_name;
    /// \brief The object's type.
    const type_t m_object_type;
    /// \brief Flag indicated if the object is locked for editing.
    bool m_locked;
};

}}}

#endif