//===========================================================================
/*
    This file is part of the GEM dynamics engine.
    Copyright (C) 2003-#YEAR# by Francois Conti, Stanford University.
    All rights reserved.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CDefNode.h"
//---------------------------------------------------------------------------

//===========================================================================
// Default Values:
//===========================================================================

// Physical properties:
double default_node_radius        = 0.05;
double default_node_kDampingPos   = 5.00;
double default_node_kDampingRot   = 0.1;
double default_node_mass          = 0.1;  // [kg]

// Graphic properties:
bool default_node_showFrame       = true;
cColorf default_node_color(1.0, 0.4, 0.4);

// Gravity field:
bool default_node_useGravity      = true;
cVector3d default_node_gravity(0.00, 0.00, -9.81);

//===========================================================================
/*!
     Constructor of cDefNode.

      \fn       cDefNode::cDefNode()
*/
//===========================================================================
cDefNode::cDefNode()
{
    m_pos.zero();
    m_rot.identity();
    m_radius        = default_node_radius;
    m_color         = default_node_color;
    m_externalForce.zero();
    m_force.zero();
    m_torque.zero();
    m_acc.zero();
    m_angAcc.zero();
    m_vel.zero();
    m_angVel.zero();
    m_kDampingPos   = default_node_kDampingPos;
    m_kDampingRot   = default_node_kDampingRot;
    m_gravity       = default_node_gravity;
    m_useGravity    = default_node_useGravity;
    m_fixed         = false;
    setMass(default_node_mass);
}


//===========================================================================
/*!
     Destructor of cDefNode.

      \fn       cDefNode::~cDefNode()
*/
//===========================================================================
cDefNode::~cDefNode()
{

}


//===========================================================================
/*!
     Define the mass of the node. The inertia is computed accordingly.

     \fn       void cDefNode::render()
*/
//===========================================================================
void cDefNode::setMass(double a_mass)
{
    m_mass = a_mass;
    m_inertia = (2.0 / 5.0) * m_mass * m_radius * m_radius ;
}


