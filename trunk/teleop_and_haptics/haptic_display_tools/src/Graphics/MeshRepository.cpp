#include "MeshRepository.h"

using namespace std;

// --------------------------------------------------------------------------
// singleton instance

template < >
MeshRepository *Singleton<MeshRepository>::m_instance = 0;

// --------------------------------------------------------------------------

MeshRepository::MeshRepository()
{
    Logger::setClassName("MeshRepository");
}

// --------------------------------------------------------------------------

bool MeshRepository::loadFromFile(const std::string &path, Format format)
{
    log() << "Attempting to load model from path " << path << endl;

    switch (format)
    {
    case FormatUnknown:
    case FormatOBJ:
        {
            MeshGLM *mesh = new MeshGLM();

            // use filename as the mesh name
            size_t slash = path.rfind('/');
            size_t period = path.rfind('.');
            if (slash == string::npos)  mesh->setName(path.substr(0, period));
            else                        mesh->setName(path.substr(slash+1, period-slash-1));

            // try to load the mesh file from the given path
            if (mesh->loadFromFile(path)) {
                m_meshes.push_back(mesh);
                return true;
            }
            else {
                log() << "[ERROR] Unable to load .obj model" << endl;
                delete mesh;
                return false;
            }
        }
    default:
        log() << "[ERROR] Unknown mesh file format!" << endl;
    }

    return false;
}

// --------------------------------------------------------------------------

bool MeshRepository::removeMesh(int index, int n)
{
    // do a bounds check
    if (index+n > m_meshes.size()) return false;

    // deallocate meshes, then remove them from the array
    for (int i = 0; i < n; ++i) delete m_meshes[index+i];
    copy(m_meshes.begin()+index+n, m_meshes.end(), m_meshes.begin()+index);
    for (int i = 0; i < n; ++i) m_meshes.pop_back();

    return true;
}

// --------------------------------------------------------------------------
