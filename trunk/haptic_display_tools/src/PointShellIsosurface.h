#ifndef POINTSHELLISOSURFACE_H
#define POINTSHELLISOSURFACE_H

#include "HapticIsosurface.h"
#include "IntervalCollisionDetector.h"
#include <vector>
#include <QMutex>

class PointShellIsosurface : public HapticIsosurface
{
    std::vector<cml::vector3d>      m_pointShell;
    cml::vector3d                   m_mass, m_moment;

    double                          m_featureSize;

    std::vector<cml::vector3d>      m_contactPoints;
    std::vector<cml::vector3d>      m_contactNormals;

    std::vector<CollisionContact>   m_contacts;

    QMutex m_mutex;

protected:
    // computes the constrained acceleration that minimizes and "energy"
    // according to Gauss' principal
    void constrainedAcceleration(const cml::vector3d &a, const cml::vector3d &alpha,
                                 cml::vector3d &ac, cml::vector3d &alphac);

#ifdef HAVE_CGAL
    void constrainedAcceleration3(const cml::vector3d &a, const cml::vector3d &alpha,
                                 cml::vector3d &ac, cml::vector3d &alphac);
    void constrainedAcceleration6(const cml::vector3d &a, const cml::vector3d &alpha,
                                 cml::vector3d &ac, cml::vector3d &alphac);
#endif

    // detects interpenetrations in the contact set and attempts to move the
    // proxy out of the surface
    bool resolvePenetrations(HapticDisplay *display);

    // computes the mass and moment vectors for the point shell
    void computeMassMatrix();

    // executes a cut on the volume mask based on the current force exerted
    void processCut(HapticDisplay *display, const cml::vector3d &p, const cml::vector3d &q);

public:
    PointShellIsosurface(Volume *volume, Volume *mask = 0, double isoValue = 0.5f);

    template <class T>
    void setPointShell(const T *points, int n, double scale = 1.0, bool clear = true)
    {
        if (clear) clearPointShell();
        const T *p = points;
        for (int i = 0; i < n; ++i, p += 3)
            m_pointShell.push_back(scale * cml::vector3d(p[0], p[1], p[2]));
        computeMassMatrix();
    }

    void clearPointShell()
    {
        m_pointShell.clear();
    }

    virtual void update(HapticDisplay *display);


    // get back a vector of points for debug output
    void getDebugPoints(std::vector<cml::vector3d> &points)
    {
        points = m_pointShell;
    }

    void getContactPoints(std::vector<cml::vector3d> &points)
    {
        m_mutex.lock();
        points = m_contactPoints;
        m_mutex.unlock();
    }

    friend class IsosurfaceOracle;
};

#endif // POINTSHELLISOSURFACE_H
