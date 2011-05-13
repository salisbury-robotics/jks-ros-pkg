#include "HapticDisplay.h"
#include <chai3d.h>
#include <cml/cml.h>

using namespace cml;

// --------------------------------------------------------------------------

HapticDisplay::HapticDisplay(cGenericHapticDevice *device, int ID)
{
    m_displayID = ID;
    m_chai3dDevice = device;
    cHapticDeviceInfo specs = device->getSpecifications();

    m_permutation.set(0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0,
                      1.0, 0.0, 0.0);

    // set up workspace radii and transform
    m_toWorld           = identity_4x4();
    m_workspaceRadius   = 1.0;
    m_deviceRadius      = specs.m_workspaceRadius;
    m_linearStiffness   = specs.m_maxForceStiffness * 0.5;
    m_torsionalStiffness= specs.m_maxTorqueStiffness * 0.5;

    m_deviceMaxForce    = specs.m_maxForce;
    m_deviceMaxTorque   = specs.m_maxTorque;

    // initialize clutching variables
    m_clutching = false;
    m_clutchExternal = false;
    m_clutchTranslation = zero_3D();
    m_clutchRotation = identity_3x3();
    
    // do an initial read from the tool (assume device already initialized)
//    update();
    m_devicePosition    = zero_3D();
    m_deviceOrientation = identity_3x3();

    // set initial state
    m_toolRadius        = 0.02;
    m_proxyPosition     = toolPosition();
    m_proxyOrientation  = toolOrientation();
    m_directForce       = zero_3D();

    m_lastForce.zero();
    m_lastTorque.zero();
}

HapticDisplay::~HapticDisplay()
{

}

// --------------------------------------------------------------------------

void HapticDisplay::setTransform(const cml::matrix44f_c &transform)
{
    // the device-to-world matrix is the inverse of the transform given
    m_toWorld = m_toDevice = transform;
    matrix_invert_RT_only(m_toWorld);
}

// --------------------------------------------------------------------------

void HapticDisplay::update()
{
    // read the device position
    cVector3d dp;
    m_chai3dDevice->getPosition(dp);
    m_devicePosition = m_permutation * vector3d(dp.x, dp.y, dp.z);

    // read the device orientation
    cMatrix3d dr;
    m_chai3dDevice->getRotation(dr);
    m_deviceOrientation = m_permutation * matrix33d( dr[0][0], dr[0][1], dr[0][2],
                                                     dr[1][0], dr[1][1], dr[1][2],
                                                     dr[2][0], dr[2][1], dr[2][2] );

    // update the clutching state
    updateClutch();
}

void HapticDisplay::updateClutch()
{
    // detect a rising edge on the clutch button
    if (!m_clutching && (buttonState(k_clutchButton) || m_clutchExternal))
    {
        m_clutching = true;
        m_clutchStartPosition = m_devicePosition;
        m_clutchStartOrientation = m_deviceOrientation;
    }
    // detect a falling edge on the clutch button
    else if (m_clutching && !(buttonState(k_clutchButton) || m_clutchExternal))
    {
        m_clutching = false;
        m_clutchTranslation += m_clutchStartPosition - m_devicePosition;
        m_clutchRotation *= (m_clutchStartOrientation * transpose(m_deviceOrientation));
    }

    // set the resulting clutched position and orientation of the device
    if (m_clutching)
    {
        m_clutchedDevicePosition = m_clutchTranslation + m_clutchStartPosition;
        m_clutchedDeviceOrientation = m_clutchRotation * m_clutchStartOrientation;
    }
    else
    {
        m_clutchedDevicePosition = m_clutchTranslation + m_devicePosition;
        m_clutchedDeviceOrientation = m_clutchRotation * m_deviceOrientation;
    }
}

// --------------------------------------------------------------------------


void HapticDisplay::applyForces()
{
    // then add a spring force from the proxy maintained within this class
    vector3d p = transform_point(m_toDevice, m_proxyPosition);
    p *= m_deviceRadius / m_workspaceRadius;    // convert to physical coords

    vector3d f = (p - m_clutchedDevicePosition) * m_linearStiffness;

    // compute torque based on difference in orientation (in tool frame)
    matrix33d R = transpose(toolOrientation()) * m_proxyOrientation;

    double theta;
    vector3d t;
    matrix_to_axis_angle(R, t, theta);
    t *= theta * m_torsionalStiffness;

    // transform the torque to device coordinates (does not take into account clutch!)
    t = m_clutchedDeviceOrientation * t;

    // add the direct force if there is one
    if (m_directForce != zero_3D())
        f += transform_vector(m_toDevice, m_directForce);

    // disable forces if we're clutching
    if (m_clutching) {
        f.zero();
        t.zero();
    }

    // remember the force and torque being rendered
    m_lastForce = f;
    m_lastTorque = t;

    // tell the tool to apply the computed forces and torques
    f = transpose(m_permutation) * f;
    cVector3d force(f[0], f[1], f[2]);
    
	// convert torque vector to cVector type and send
    t = transpose(m_permutation) * t;
    cVector3d torque(t[0], t[1], t[2]);

	cVector3d zero(0,0,0);
    m_chai3dDevice->setForceAndTorqueAndGripper(force, torque, 0.0);
}

void HapticDisplay::clearForces()
{
    cVector3d zero = CHAI_VECTOR_ZERO;
    m_chai3dDevice->setForce(zero);
}

// --------------------------------------------------------------------------

vector3d HapticDisplay::toolPosition()
{
    // compute a scale factor to transform device positions to desired radius
    double scale = m_workspaceRadius / m_deviceRadius;

    // return the device position transformed into the world workspace
    return transform_point(m_toWorld, scale * m_clutchedDevicePosition);
}

matrix33d HapticDisplay::toolOrientation()
{
    // extract the rotation component of the tool to world matrix
    matrix33d orient;
    matrix_linear_transform(orient, m_toWorld);

    // apply rotation and return
    return orient * m_clutchedDeviceOrientation;
}

// --------------------------------------------------------------------------
