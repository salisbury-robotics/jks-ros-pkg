#include "Mesh.h"
#include <GLM/glm.h>

using namespace std;
using namespace cml;

// --------------------------------------------------------------------------

MeshGLM::MeshGLM(const string &name, const string &filename, unsigned int flags)
    : Mesh(name), m_model(0)
{    
    if (!filename.empty())
        loadFromFile(filename);

    // initialize rendering flags
    m_flags = 0;
    if (flags & k_useTexture)   m_flags |= GLM_TEXTURE;
    if (flags & k_useMaterial)  m_flags |= GLM_MATERIAL;
}

MeshGLM::~MeshGLM()
{
    if (m_model) glmDelete(m_model);
}

bool MeshGLM::loadFromFile(const string &filename)
{
    m_model = glmReadOBJ(filename.c_str());
    m_path = filename;
    computeBoundingBox();

    // auto-detect materials and/or textures to guess rendering flags
    if (m_model) {
        if (m_model->nummaterials)  m_flags |= GLM_MATERIAL;
        if (m_model->numtextures && m_model->numtexcoords)
                                    m_flags |= GLM_TEXTURE;
    }

    return m_model != 0;
}

int MeshGLM::numVertices()
{
    return m_model->numvertices;
}

int MeshGLM::numTriangles()
{
    return m_model->numtriangles;
}

void MeshGLM::render()
{
    m_material.apply();

    if (m_model->normals)
        glmDraw(m_model, GLM_SMOOTH | m_flags);
    else
        glmDraw(m_model, GLM_FLAT | m_flags);
}

void MeshGLM::computeBoundingBox()
{
    if (m_model && m_model->vertices)
    {
        // note: 0th vertex is a dud?!  (array probabaly indexed from 1)
        vector3f lo(m_model->vertices + 3);
        vector3f hi(m_model->vertices + 3);

        for (int i = 2; i < m_model->numvertices; ++i) {
            lo.minimize(vector3f(m_model->vertices + 3*i));
            hi.maximize(vector3f(m_model->vertices + 3*i));
        }

        m_bbox[bbLower] = lo;
        m_bbox[bbUpper] = hi;
    }
}

const float *MeshGLM::vertexPointer()
{
    return m_model->vertices + 3;
}

// --------------------------------------------------------------------------
