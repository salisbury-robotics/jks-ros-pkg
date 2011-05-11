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
#ifndef CDefLinkH
#define CDefLinkH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CDefNode.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \class      cDefLink
      \brief      cDefLink between two nodes.
*/
//===========================================================================
class cDefLink
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cDefLink.
    cDefLink(cDefNode* a_node0, cDefNode* a_node1);

    //! Destructor of cDefLink.
    ~cDefLink();

    // METHODS:
    //===========================================================================
    /*!
         Render link in OpenGL.

         \fn       void cDefLink::render()
    */
    //===========================================================================
    inline void render()
    {
        // render link
        glColor4fv( (const float *)&m_color);
        glBegin(GL_LINES);
          glVertex3dv( (const double *)&m_node0->m_pos);
          glVertex3dv( (const double *)&m_node1->m_pos);
        glEnd();
    }


    //===========================================================================
    /*
         Compute forces applied on mass nodes

         \fn       void cDefLink::computeForces()
    */
    //===========================================================================
    inline void computeForces()
    {
        // update basic parameters of current link
        m_wLink01 = cSub(m_node1->m_pos, m_node0->m_pos);
        m_wLink10 = cMul(-1, m_wLink01);
        m_length = m_wLink01.length();

        m_wzLink01 = cMul((m_node0->m_rot), m_nzLink01);
        m_wzLink10 = cMul((m_node1->m_rot), m_nzLink10);

        //-------------------------------------------------------------
        // ELONGATION:
        //-------------------------------------------------------------
        // if distance too small, no forces are applied
        if (m_length < 0.000001) { return; }

        // elongation compute force
        double f = m_kSpringElongation * (m_length - m_length0);

        // apply force
        if (m_length > 0.000001)
        {
            cVector3d force = cMul(f/m_length, m_wLink01);
            m_node0->addForce(force);
            m_node1->addForce(cMul(-1, force));
        }

        //-------------------------------------------------------------
        // FLEXION:
        //-------------------------------------------------------------
        // compute flexion forces and torques
        cVector3d torqueDir0 = cCross(m_wzLink01, m_wLink01);
        cVector3d torqueDir1 = cCross(m_wzLink10, m_wLink10);
        double angle0 = cAngle(m_wzLink01, m_wLink01);
        double angle1 = cAngle(m_wzLink10, m_wLink10);
        double torqueMag0 = angle0 * m_kSpringFlexion;
        double torqueMag1 = angle1 * m_kSpringFlexion;

        if ((m_length > 0.000001) && (m_enabled))
        {
			if (torqueMag0 > 0.0001)
            {
                cVector3d torque0 = cMul(torqueMag0, cNormalize(torqueDir0));
                m_node0->addTorque(torque0);
                cVector3d force1 = cMul((torqueMag0/m_length), cNormalize(cCross(m_wLink01, torque0)));
                m_node1->addForce(force1);
                m_node0->addForce(cMul(-1,force1));
            }

            if (torqueMag1 > 0.0001)
            {
                cVector3d torque1 = cMul(torqueMag1, cNormalize(torqueDir1));
                m_node1->addTorque(torque1);
                cVector3d force0 = cMul((torqueMag1/m_length), cNormalize(cCross(m_wLink10, torque1)));
                m_node0->addForce(force0);
                m_node1->addForce(cMul(-1,force0));
            }
        }

        //-------------------------------------------------------------
        // TORSION:
        //-------------------------------------------------------------

		if (m_enabled)
		{
			// update frame vectors in world coordinates.
			m_node0->m_rot.mulr(m_A0, m_wA0);
			m_node0->m_rot.mulr(m_B0, m_wB0);
			m_node1->m_rot.mulr(m_A1, m_wA1);

			// compute torsional angle and torque
			cVector3d v0, v1;
			m_wA0.crossr(m_wLink01, v0);
			m_wA1.crossr(m_wLink01, v1);
			v0.normalize();
			v1.normalize();

			cVector3d torque;
			v0.crossr(v1, m_torsionAxisAngle);
			m_torsionAxisAngle.mulr(m_kSpringTorsion, torque);

			m_node0->addTorque(torque);
			m_node1->addTorque(-1.0 * torque);
		}
    }


    // GRAPHICAL PROPERTIES:
    //! Color used to display NN-links.
    cColorf m_color;

    // NODES:
    //! Node 0 of current link.
    cDefNode *m_node0;

    //! Node 1 of current link.
    cDefNode *m_node1;

    // SPRING PROPERTIES:

    //! elongation spring constant
    double m_kSpringElongation;

    //! flexion spring constant
    double m_kSpringFlexion;

    //! torsion spring constant
    double m_kSpringTorsion;

	//! is link enabled
	bool m_enabled;

    //! elongation damper constant
    //double m_kDamperElongation;

    //! flexion damper constant
    //double m_kDamperFlexion;

    //! torsion damper constant
    //double m_kDamperTorsion;

  public:

    //! initial link vector between node 0 and node 1 in node0 reference frame and world coordinates
    cVector3d m_nzLink01;
    cVector3d m_wzLink01;

    //! initial link vector between node 1 and node 0 in node1 reference frame and world coordinates
    cVector3d m_nzLink10;
    cVector3d m_wzLink10;

    //! current link between node 0 and node1 in world coordinates
    cVector3d m_wLink01;
    cVector3d m_wLink10;

    //! torsional angle
    cVector3d m_torsionAxisAngle;

    //! reference vector frame A on sphere 0 (local and world coordinates)
    cVector3d m_A0;
    cVector3d m_wA0;

    //! reference vector frame B on sphere 0 (local and world coordinates)
    cVector3d m_B0;
    cVector3d m_wB0;

    //! reference vector frame A on sphere 1 (local and world coordinates)
    cVector3d m_A1;
    cVector3d m_wA1;

    // PHYSICAL PROPERTIES:
    //! initial length of length
    double m_length0;

    //! current length of length
    double m_length;
};

#endif

