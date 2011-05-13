#include "HapticInstrument.h"
#include "PointShellIsosurface.h"
#include "Graphics/OpenGL.h"

using namespace std;

// --------------------------------------------------------------------------

HapticInstrument::HapticInstrument()
{
}

HapticInstrument::~HapticInstrument()
{
    for (int i = 0; i < msSentinel; ++i) {
        for (int j = 0; j < isSentinel; ++j)
            clearMeshes(m_mesh[i][j]);
        clearMeshes(m_pointshell[i]);
    }
}

// --------------------------------------------------------------------------

void HapticInstrument::clearMeshes(std::vector<MeshGLM *> &meshes)
{
    for (vector<MeshGLM *>::iterator it = meshes.begin(); it != meshes.end(); ++it)
        delete *it;
    meshes.clear();
}

void HapticInstrument::drawMeshes(std::vector<MeshGLM *> &meshes, float scale)
{
    if (meshes.empty()) return;
    glPushMatrix();
    glScalef(scale, scale, scale);
    for (vector<MeshGLM *>::iterator it = meshes.begin(); it != meshes.end(); ++it)
        (*it)->render();
    glPopMatrix();
}

void HapticInstrument::addShell(PointShellIsosurface *psi, std::vector<MeshGLM *> &meshes, double scale)
{
    for (vector<MeshGLM *>::iterator it = meshes.begin(); it != meshes.end(); ++it)
        psi->setPointShell((*it)->vertexPointer(), (*it)->numVertices(), scale, false);
}

// --------------------------------------------------------------------------

void HapticInstrument::addMesh(const std::string &file, MeshType type,
                               ModelScale scale, InstrumentState state)
{
    MeshGLM *mesh = new MeshGLM;
    if (mesh->loadFromFile(file))
    {
        if (type == mtVisual)
            m_mesh[scale][state].push_back(mesh);
        else if (type == mtPointShell)
            m_pointshell[scale].push_back(mesh);
    }
}

void HapticInstrument::render(float fScale, float vScale, bool active)
{
    glPushAttrib(GL_ENABLE_BIT);

    // first render the fixed scale meshes
    drawMeshes(m_mesh[msFixed][isFixed], fScale);
    if (active) drawMeshes(m_mesh[msFixed][isActive], fScale);
    else        drawMeshes(m_mesh[msFixed][isPassive], fScale);

    // then render the variable scale meshes
    drawMeshes(m_mesh[msVariable][isFixed], vScale);
    if (active) drawMeshes(m_mesh[msVariable][isActive], vScale);
    else        drawMeshes(m_mesh[msVariable][isPassive], vScale);

    glPopAttrib();
}

void HapticInstrument::updatePointShell(PointShellIsosurface *psi, double fScale, double vScale)
{
    psi->clearPointShell();
    addShell(psi, m_pointshell[isFixed], fScale);
    addShell(psi, m_pointshell[msVariable], vScale);
}

// --------------------------------------------------------------------------
