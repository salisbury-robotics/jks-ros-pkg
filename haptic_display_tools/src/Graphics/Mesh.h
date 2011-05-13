#ifndef MESH_H
#define MESH_H

#include "Material.h"

// --------------------------------------------------------------------------

class Mesh
{
protected:
    std::string     m_name;
    std::string     m_path;
    bool            m_visible;
    Material        m_material;

    cml::vector3f   m_bbox[2];    

public:
    Mesh(const std::string &name = "")
        : m_name(name), m_visible(true)     { }
    virtual ~Mesh()                         { }

    std::string name() const                { return m_name; }
    void setName(const std::string &name)   { m_name = name; }

    std::string path() const                { return m_path; }
    void setPath(const std::string &path)   { m_path = path; }

    bool visible() const                    { return m_visible; }
    void setVisible(bool visible)           { m_visible = visible; }

    Material material() const               { return m_material; }
    void setMaterial(const Material &m)     { m_material = m; }

    enum Corner { bbLower, bbUpper };
    cml::vector3f boundingBox(Corner c) const { return m_bbox[c]; }

    // virtual functions that subclasses should support
    virtual int numVertices()               { return 0; }
    virtual int numTriangles()              { return 0; }
    virtual void render() = 0;
};

// --------------------------------------------------------------------------

struct _GLMmodel;

class MeshGLM : public Mesh
{
    _GLMmodel   *m_model;
    unsigned int m_flags;

    void computeBoundingBox();

public:
    // some constants for the flags
    static const unsigned int k_useTexture = (1 << 2);
    static const unsigned int k_useMaterial = (1 << 4);

    // constructor and destructor
    MeshGLM(const std::string &name = "", const std::string &filename = "",
            unsigned int flags = 0);
    virtual ~MeshGLM();

    // loads a .obj mesh from given file
    bool loadFromFile(const std::string &filename);

    virtual int numVertices();
    virtual int numTriangles();
    virtual void render();

    // pointer to vertex data
    const float *vertexPointer();
};

// --------------------------------------------------------------------------
#endif // MESH_H
