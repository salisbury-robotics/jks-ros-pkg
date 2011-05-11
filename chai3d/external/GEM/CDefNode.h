//===========================================================================
/*
    This file is part of the GEM dynamics engine.
    Copyright (C) 2006-2008 by Francois Conti, Stanford University.
    All rights reserved.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDefNodeH
#define CDefNodeH
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------

extern bool default_node_showFrame;

//===========================================================================
/*!
      \class      cDefNode
      \brief      cDefNode defines a dynamic node within the skeleton
*/
//===========================================================================
class cDefNode
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cDefNode.
    cDefNode();

    //! Destructor of cDefNode.
    ~cDefNode();

    // METHODS:
    //! set the mass of the node
    void setMass(double a_mass);

    //! Add force to node
    inline void addForce(const cVector3d &a_force)
    {
        m_force.add(a_force);
    }

    //! Add torque to node
    inline void addTorque(const cVector3d &a_torque)
    {
        m_torque.add(a_torque);
    }

    //! Set an external force to node
    inline void setExternalForce(const cVector3d &a_force)
    {
        m_externalForce = a_force;
    }

    //! set an external torque to node
    inline void setExternalTorque(const cVector3d &a_torque)
    {
        m_externalTorque = a_torque;
    }

    //! compute next position
    inline void computeNextPose(double a_timeInterval)
    {
        if (!m_fixed)
        {
            // Euler double integration for position
            cVector3d damping;
            m_vel.mulr(-m_kDampingPos * m_mass, damping);
            m_force.add(damping);
            m_acc = cDiv(m_mass, cAdd(m_force, m_externalForce));
            m_vel = cAdd(m_vel, cMul(a_timeInterval, m_acc));
            m_nextPos = cAdd(m_pos, cMul(a_timeInterval, m_vel));
        }
        else
        {
            m_nextPos = m_pos;
        }

        // Euler double integration for rotation
        cVector3d dampingAng;
        m_angVel.mulr(-m_kDampingRot * m_mass, dampingAng);
        m_torque.add(dampingAng);
        m_angAcc = cMul((1/m_inertia), m_torque);
        m_angVel = cAdd(m_angVel, cMul(a_timeInterval, m_angAcc));

        double normAngVel = m_angVel.length();
        if (normAngVel < 0.000001)
        {
            m_nextRot = m_rot;
        }
        else
        {
            m_nextRot = cRotate(m_rot, m_angVel, a_timeInterval * normAngVel);
        }
    }

    //! update pose with new computed values
    inline void applyNextPose()
    {
        m_pos = m_nextPos;
        m_rot = m_nextRot;
    }

    //! clear forces and torques
    inline void clearForces()
    {
        if (m_useGravity)
        {
            m_force = m_gravity;
            m_force.mul(m_mass);
        }
        else
        {
            m_force.zero();
        }
        m_torque.zero();
    }

    //! clear external forces and torques
    inline void clearExternalForces()
    {
        m_externalForce.zero();
        m_externalTorque.zero();
    }

    //! Render node in OpenGL.
    inline void render()
    {
        // set pose
        cMatrixGL mat;
        mat.set(m_pos, m_rot);
        mat.glMatrixPushMultiply();

        // draw node
        m_color.render();
        cDrawSphere(m_radius, 12, 12);

        // draw frame
        if (default_node_showFrame == true)
        {
            double frameScale = 3.0 * m_radius;
            cDrawFrame(frameScale);
        }

        // pos open gl matrix
        mat.glMatrixPop();

        // render external forces
        glColor4fv( (const float *)&m_color);
        cVector3d v = cAdd(m_pos, cMul(1.0/50.0, m_externalForce));
        glBegin(GL_LINES);
          glVertex3dv( (const double *)&m_pos);
          glVertex3dv( (const double *)&v);
        glEnd();
    }

    // GENERAL PROPERTIES:
    //! radius of mass node
    double m_radius;

    //! Color used to display nodes.
    cColorf m_color;

    // PHYSICAL PROPERTIES:
    //! Mass property
    double m_mass;

    //! Current force applied on node
    cVector3d m_force;

    //! Current torque applies on node
    cVector3d m_torque;

    //! instant acceleration at node
    cVector3d m_acc;

    //! instant angular acceleration at node
    cVector3d m_angAcc;

    //! instant velocity at node
    cVector3d m_vel;

    //! instant angular velocity at node;
    cVector3d m_angVel;

    //! linear damping
    double m_kDampingPos;

    //! angular damping
    double m_kDampingRot;

    //! inertia (to be completed)
    double m_inertia;

    //! if TRUE, then mass is fixed in space and can not move.
    bool m_fixed;

    //! if TRUE, then gravity is enabled
    bool m_useGravity;

    //! gravity field
    cVector3d m_gravity;

    //! position computed
    cVector3d m_pos;

    //! rotation computed
    cMatrix3d m_rot;

  public:
    //! next position computed
    cVector3d m_nextPos;

    //! next rotation computed
    cMatrix3d m_nextRot;

  private:
    // external force
    cVector3d m_externalForce;

    // external torque
    cVector3d m_externalTorque;
};

#endif


