#ifndef MESHREPOSITORY_H
#define MESHREPOSITORY_H

#include "Common/Logger.h"
#include "Common/Singleton.h"
#include "Mesh.h"

#include <vector>

class MeshRepository : public Singleton<MeshRepository>, public Logger
{
    // vector of currently loaded meshes
    std::vector<Mesh*> m_meshes;

public:
    MeshRepository();

    // enumeration of different file formats that can be loaded
    enum Format { FormatUnknown, FormatOBJ, FormatSentinel };

    bool loadFromFile(const std::string &path, Format format = FormatUnknown);

    // methods for accessing stored meshes
    int getNumberMeshes() const     { return m_meshes.size(); }
    Mesh *getMesh(int index) const  { return m_meshes[index]; }

    // removes a number of meshes, beginning at index, return true if success
    bool removeMesh(int index, int n = 1);
};

#endif // MESHREPOSITORY_H
