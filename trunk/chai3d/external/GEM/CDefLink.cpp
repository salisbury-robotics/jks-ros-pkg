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
#include "CDefLink.h"
#include "CDefMesh.h"
//---------------------------------------------------------------------------

//===========================================================================
// Default Values:
//===========================================================================

// Physical properties:
double default_link_kSpringElongation = 100.0; // [N/m]
double default_link_kSpringFlexion    = 0.1;   // [Nm/RAD]
double default_link_kSpringTorsion    = 0.1;   // [Nm/RAD]

// Graphic properties:
cColorf default_link_color(0.2, 0.2, 1.0);

//---------------------------------------------------------------------------

//===========================================================================
/*!
     Constructor of cDefLink.

      \fn       cDefLink::cDefLink()
*/
//===========================================================================
cDefLink::cDefLink(cDefNode* a_node0, cDefNode* a_node1)
{
    // Set nodes:
    m_node0 = a_node0;
    m_node1 = a_node1;
	m_enabled = true;

    // set position references
    m_wLink01 = cSub(m_node1->m_pos, m_node0->m_pos);
    m_wLink10 = cSub(m_node0->m_pos, m_node1->m_pos);

	if (m_wLink01.length() == 0)
	{
		m_enabled = false;
	}

    m_wzLink01 = m_wLink01;
    m_wzLink10 = m_wLink10;

    m_nzLink01 = cMul(cTrans(m_node0->m_rot), m_wzLink01);
    m_nzLink10 = cMul(cTrans(m_node1->m_rot), m_wzLink10);

    // compute reference frames
    cVector3d v(1.0, 0.0, 0.0);
    double ang = cAngle(v, m_wLink01);
    if ((ang < 0.01) || (ang > 3.13)) { v.set(0.0, 1.0, 0.0); }

    cVector3d A0 = cNormalize( cCross(m_wLink01, v) );
    cVector3d A1 = A0;
    cVector3d B0_ = cNormalize( cCross(m_wLink01, A0) );

    m_A0 = cMul(cTrans(m_node0->m_rot), A0);
    m_B0 = cMul(cTrans(m_node0->m_rot), B0_);
    m_A1 = cMul(cTrans(m_node1->m_rot), A1);

    // set default color
    m_color = default_link_color;

    // compute initial length
    m_length0 = cDistance(m_node1->m_pos, m_node0->m_pos);
    m_length = m_length0;

    // set elongation spring constant [N/M]
    m_kSpringElongation = default_link_kSpringElongation;

    // set flexion angular spring constant [NM/RAD]
    m_kSpringFlexion = default_link_kSpringFlexion;

    // set torsion angular spring constant [NM/RAD]
    m_kSpringTorsion = default_link_kSpringTorsion;
}


//===========================================================================
/*!
     Destructor of cDefLink.

      \fn       cDefLink::~cDefLink()
*/
//===========================================================================
cDefLink::~cDefLink()
{

}

