#ifndef HAPTICINSTRUMENT_H
#define HAPTICINSTRUMENT_H

#include "Graphics/Mesh.h"
#include <vector>
#include <string>

class PointShellIsosurface;

class HapticInstrument
{
public:
    enum ModelScale         { msFixed, msVariable, msSentinel };
    enum InstrumentState    { isFixed, isPassive, isActive, isSentinel };
    enum MeshType           { mtVisual, mtPointShell, mtSentinel };

protected:
    std::vector<MeshGLM *>  m_mesh[msSentinel][isSentinel];
    std::vector<MeshGLM *>  m_pointshell[msSentinel];

    void clearMeshes(std::vector<MeshGLM *> &meshes);
    void drawMeshes(std::vector<MeshGLM *> &meshes, float scale = 1.f);
    void addShell(PointShellIsosurface *psi, std::vector<MeshGLM *> &meshes, double scale = 1.f);

public:
    HapticInstrument();
    virtual ~HapticInstrument();

    void addMesh(const std::string &file, MeshType type = mtVisual,
                 ModelScale scale = msFixed, InstrumentState state = isFixed);

    void render(float fScale = 1.f, float vScale = 1.f, bool active = false);

    void updatePointShell(PointShellIsosurface *psi, double fScale = 1.f, double vScale = 1.f);
};

#endif // HAPTICINSTRUMENT_H
