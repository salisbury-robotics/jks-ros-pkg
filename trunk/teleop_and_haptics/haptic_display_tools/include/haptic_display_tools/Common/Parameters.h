#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>
#include <string>
#include <map>
#include <typeinfo>
#include <cml/mathlib/typedef.h>

// The Parameters class maintains a key-value mapping of an aggregate
// collection of variables of various types.  Supports bool, int, float,
// string, and 3-vectors of ints and floats.

class Parameters
{
    std::string m_title;    // title of this collection, if any

    std::map<std::string, std::string>      m_type;
    std::map<std::string, bool>             m_bools;
    std::map<std::string, int>              m_ints;
    std::map<std::string, float>            m_floats;
    std::map<std::string, std::string>      m_strings;
    std::map<std::string, cml::vector3i>    m_vector3is;
    std::map<std::string, cml::vector3f>    m_vector3fs;
    
public:
    Parameters(std::string title = "") : m_title(title) { }
    virtual ~Parameters()                               { }

    std::string title() { return m_title; }

    // Mutators to set values for the keys.
    // Warning: do not try to set values of different types for the same key --
    //          behaviour is undetermined!

    void setBool(const std::string &key, bool value)
        { m_type[key] = typeid(bool).name();            m_ints[key] = value; }

    void setInt(const std::string &key, int value)
        { m_type[key] = typeid(int).name();             m_ints[key] = value; }

    void setFloat(const std::string &key, float value)
        { m_type[key] = typeid(float).name();           m_floats[key] = value; }

    void setString(const std::string &key, std::string value)
        { m_type[key] = typeid(std::string).name();     m_strings[key] = value; }

    void setVector3i(const std::string &key, cml::vector3i value)
        { m_type[key] = typeid(cml::vector3i).name();   m_vector3is[key] = value; }

    void setVector3f(const std::string &key, cml::vector3f value)
        { m_type[key] = typeid(cml::vector3f).name();   m_vector3fs[key] = value; }

    // Accessors to retrieve set values
    // Warning: no error/type checking is performed, for sake of speed...

    bool            getBool(const std::string &key)     { return m_bools[key]; }
    int             getInt(const std::string &key)      { return m_ints[key]; }
    float           getFloat(const std::string &key)    { return m_floats[key]; }
    std::string     getString(const std::string &key)   { return m_strings[key]; }
    cml::vector3i   getVector3i(const std::string &key) { return m_vector3is[key]; }
    cml::vector3f   getVector3f(const std::string &key) { return m_vector3fs[key]; }
};

// insertion and extraction operators to read/write the parameter set
std::ostream &operator<<(std::ostream &stream, const Parameters &p);
std::istream &operator>>(std::istream &steram, Parameters &p);

#endif // PARAMETERS_H
