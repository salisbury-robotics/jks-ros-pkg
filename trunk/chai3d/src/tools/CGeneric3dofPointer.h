//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-#YEAR# by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
    \author:    Federico Barbagli
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGeneric3dofPointerH
#define CGeneric3dofPointerH
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
#include "graphics/CColor.h"
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CMesh.h"
#include "forces/CProxyPointForceAlgo.h"
#include "forces/CPotentialFieldForceAlgo.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file       CGeneric3dofPointer.h
      \class      cGeneric3dofPointer
      \brief      cGeneric3dofPointer represents a haptic tool that 
                  can apply forces in three degrees of freedom and
                  maintains three or six degrees of device pose.

                  This class provides i/o with haptic devices and
                  a basic graphical representation of a tool.
*/
//===========================================================================
class cGeneric3dofPointer : public cGenericTool
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cGeneric3dofPointer.
    cGeneric3dofPointer(cWorld* a_world);

    //! Destructor of cGeneric3dofPointer.
    virtual ~cGeneric3dofPointer();

    // METHODS:

    // Graphics

    //! Render the object in OpenGL
    virtual void render(const int a_renderMode=0);

    //----------------------------------------------------------------------
    // Tool commands
    //----------------------------------------------------------------------

    //! Start communication with the device connected to the tool (0 indicates success)
    virtual int start();

    //! Stop communication with the device connected to the tool (0 indicates success)
    virtual int stop();

    //! Initialize the device connected to the tool (0 indicates success).
    virtual int initialize(const bool a_resetEncoders=false);

    //! Toggle forces on
    virtual int setForcesON();

    //! Toggle forces off
    virtual int setForcesOFF();

    //! Update position and orientation of the device.
    virtual void updatePose();

    //! Compute interaction forces with environment.
    virtual void computeInteractionForces();

    //! Apply latest computed forces to device.
    virtual void applyForces();

    //! get a pointer to the proxy force algorithm
    virtual cProxyPointForceAlgo* getProxy() { return m_proxyPointForceModel; }

    // Check if the tool is touching a particular object
    virtual bool isInContact(cGenericObject* a_object);

    //---------------------------------------------------------------------
    // Workspace settings
    //---------------------------------------------------------------------

    //! Set radius of pointer.
    virtual void setRadius(const double& a_radius);

    //! Set virtual workspace dimensions in which tool will be working.
    virtual void setWorkspaceRadius(const double& a_workspaceRadius);

    //! Read the radius of the workspace of the tool
    double getWorkspaceRadius() { return(m_workspaceRadius); }

    //! Set the scale factor between the workspace of the tool and one of the haptic device.
    double setWorkspaceScaleFactor(double a_scaleFactor);

    //! Read the scale factor between the workspace of the tool and one of the haptic device.
    double getWorkspaceScaleFactor() { return (m_workspaceScaleFactor); }


    // MEMBERS:

    //---------------------------------------------------------------------
    // Tool graphical models
    //---------------------------------------------------------------------
    //! Sphere representing the device
    cShapeSphere* m_deviceSphere;

    //! Sphere representing the proxy
    cShapeSphere* m_proxySphere;

    //! Mesh representing the device
    cMesh* m_deviceMesh;

    //! Mesh representing the proxy
    cMesh* m_proxyMesh;

    //! Color of line connecting proxy and device position together
    cColorf m_colorLine;

    //! Material prperties of proxy
    cMaterial m_materialProxy;

    //! Material properties of proxy when button is pressed
    cMaterial m_materialProxyButtonPressed;


    //---------------------------------------------------------------------
    // Tool force models
    //---------------------------------------------------------------------

    //! finger-proxy algorithm model to handle interactions with mesh objects.
    cProxyPointForceAlgo* m_proxyPointForceModel;

    //! potential fields model
    cPotentialFieldForceAlgo* m_potentialFieldsForceModel;

    //! The last force computed for application to this tool, in the world coordinate
    //! system.  (N)

    //! If you want to manually send forces to a device, you can modify this
    //! value before calling 'applyForces'.
    cVector3d m_lastComputedGlobalForce;

    //! The last force computed for application to this tool, in the device coordinate
    //! system.  (N)
    cVector3d m_lastComputedLocalForce;

    
    //---------------------------------------------------------------------
    // Tool position information and workspace
    //---------------------------------------------------------------------
    //! radius of the workspace which can be accessed by the tool
    double m_workspaceRadius;

    //! scale factor between the sizes of the tool workspace and the haptic device workspace.
    double m_workspaceScaleFactor;

    //! Position of device in device local coordinate system
    cVector3d m_deviceLocalPos;

    //! Position of device in world global coordinate system
    cVector3d m_deviceGlobalPos;

    //! Velocity of device in device local coordinate system
    cVector3d m_deviceLocalVel;

    //! Velocity of device in world global coordinate system
    cVector3d m_deviceGlobalVel;

    //! Orientation of wrist in local coordinates of device
    cMatrix3d m_deviceLocalRot;

    //! Orientation of wrist in global coordinates of device
    cMatrix3d m_deviceGlobalRot;

  protected:

    // MEMBERS:

    //! Radius of sphere representing position of pointer.
    double m_displayRadius;

    //! World in which tool is interacting
    cWorld* m_world;

    //! this flag records whether the user has enabled forces
    bool m_forceON;

    //! flag to avoid initial bumps in force (has the user sent a _small_ force yet?)
    bool m_forceStarted;

    //! Normally this class waits for a very small force before initializing forces
    //! to avoid initial "jerks" (a safety feature); you can bypass that requirement
    //! with this variable
    bool m_waitForSmallForce;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

