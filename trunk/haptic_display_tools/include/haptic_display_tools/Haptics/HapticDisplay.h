#ifndef HAPTICDISPLAY_H
#define HAPTICDISPLAY_H

#include <devices/CGenericHapticDevice.h>
#include <cml/mathlib/typedef.h>

// A HapticDisplay is a loose wrapper around a cGeneric3dofPointer object
// meant for use as part of a HapticScene.  Because the pointer requires a
// cWorld on construction, a HapticDisplay is best created through the
// HapticScene class.

class HapticDisplay
{
protected:
    cGenericHapticDevice *m_chai3dDevice;
    cml::matrix33d       m_permutation;

    // scene-unique identifier for this haptic display
    int                  m_displayID;

    // affine transform to take coordinates from tool frame to world frame
    cml::matrix44d       m_toWorld;
    cml::matrix44d       m_toDevice;

    // radius of the desired workspace (world units) and the device's workspace (m)
    double               m_workspaceRadius;
    double               m_deviceRadius;

    // force and torque output capabilities of the device
    double               m_deviceMaxForce;
    double               m_deviceMaxTorque;

    // cache of the last-read device position and orientation
    cml::vector3d        m_devicePosition;
    cml::matrix33d       m_deviceOrientation;

    // variables to track clutching for workspace management
    static const int     k_clutchButton = 1;
    bool                 m_clutching;
    bool                 m_clutchExternal;
    cml::vector3d        m_clutchTranslation, m_clutchStartPosition, m_clutchedDevicePosition;
    cml::matrix33d       m_clutchRotation, m_clutchStartOrientation, m_clutchedDeviceOrientation;

    // a proxy position for rendering forces, etc.
    cml::vector3d        m_proxyPosition;
    cml::matrix33d       m_proxyOrientation;
    double               m_linearStiffness;     // in physical N/m
    double               m_torsionalStiffness;  // in Nm/rad

    double               m_toolRadius;

    // a direct force that can be applied to the device, in world coordinates
    cml::vector3d        m_directForce;

    // the last rendered force and torque, that can be queried
    cml::vector3d        m_lastForce;
    cml::vector3d        m_lastTorque;

    void updateClutch();

public:
    HapticDisplay(cGenericHapticDevice *device, int ID = 0);
    virtual ~HapticDisplay();

    int identifier()                { return m_displayID; }
    double stiffness()              { return m_linearStiffness; }
    double torsionalStiffness()     { return m_torsionalStiffness; }
    double physicalToWorldScale()   { return m_deviceRadius / m_workspaceRadius; }
    
    void setWorkspaceRadius(double r)                   { m_workspaceRadius = r; }
    void setTransform(const cml::matrix44f_c &transform);
    void setProxyPosition(const cml::vector3d &p)       { m_proxyPosition = p; }
    void setProxyOrientation(const cml::matrix33d &m)   { m_proxyOrientation = m; }
    void setStiffness(double k)                         { m_linearStiffness = k; }
    void setTorsionalStiffness(double k)                { m_torsionalStiffness = k; }
    void setToolRadius(double r)                        { m_toolRadius = r; }
    void setClutch(bool b)                              { m_clutchExternal = b; }
    void setPermutation(const cml::matrix33d &m)        { m_permutation = m; }

    void setClutchOffsets(const cml::vector3d &t, const cml::matrix33d &r)
    {
      m_clutchTranslation = t;
      m_clutchRotation = r;
    }

    virtual void update();
    virtual void applyForces();
    virtual void clearForces();

    // returns the tool position and orientation in world coordinates
    virtual cml::vector3d   toolPosition();
    virtual cml::matrix33d  toolOrientation();
    virtual double          toolRadius()        { return m_toolRadius; }

    virtual cml::vector3d   proxyPosition()     { return m_proxyPosition; }
    virtual cml::matrix33d  proxyOrientation()  { return m_proxyOrientation; }

    // returns the unscaled, physical position of the device, in meters
    virtual cml::vector3d   devicePosition()    { return m_devicePosition; }
    virtual cml::matrix33d  deviceOrientation() { return m_deviceOrientation; }
    virtual double          deviceWorkspace()   { return m_deviceRadius; }

    // status of button i on the device (true for down)
    virtual bool            buttonState(int i)
                            { bool b; m_chai3dDevice->getUserSwitch(i, b); return b; }

    // accessor and mutator for direct force in world coordinates
    virtual cml::vector3d   directForce()       { return m_directForce; }
    virtual void            setDirectForce(const cml::vector3d &f)
                                                { m_directForce = f; }

    // for querying the last rendered force and torque
    virtual cml::vector3d   lastForce() const   { return m_lastForce; }
    virtual cml::vector3d   lastTorque() const  { return m_lastTorque; }

    // for querying how close to saturation the force/torque are
    virtual double          forceSaturation() const
                            { return m_lastForce.length() / m_deviceMaxForce; }
    virtual double          torqueSaturation() const
                            { return m_lastTorque.length() / m_deviceMaxTorque; }
};

#endif // HAPTICDISPLAY_H
