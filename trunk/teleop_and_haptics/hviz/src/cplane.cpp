#include "cplane.h"

cPlane::cPlane(cWorld * a_world) :
    cMesh(a_world)
{

//  tPlane = new cMesh(a_world);
  cPlane *tPlane = this;

  cVector3d planeVertex[4];
  double groundSize = 0.5;
  int vertices0 = tPlane->newVertex(-groundSize, -groundSize, 0.0);
  int vertices1 = tPlane->newVertex( groundSize, -groundSize, 0.0);
  int vertices2 = tPlane->newVertex( groundSize,  groundSize, 0.0);
  int vertices3 = tPlane->newVertex(-groundSize,  groundSize, 0.0);

  tPlane->m_vertices[0].m_color = cColorf(0.1, 0.1, 0.8);
  tPlane->m_vertices[1].m_color = cColorf(0.1, 0.1, 0.8);
  tPlane->m_vertices[2].m_color = cColorf(0.1, 0.1, 0.8);
  tPlane->m_vertices[3].m_color = cColorf(0.1, 0.1, 0.8);

  // compose surface with 2 triangles
  tPlane->newTriangle(vertices0, vertices1, vertices2);
  tPlane->newTriangle(vertices0, vertices2, vertices3);

  // compute surface normals
  tPlane->computeAllNormals();
  tPlane->setAsGhost(true);
  tPlane->m_normal = cVector3d(0,0,1);

  // define some material properties and apply to mesh
  cMaterial *matPlane = new cMaterial;
  matPlane->m_ambient.set(0.1, 0.1, 0.3);
  matPlane->m_diffuse.set(0.2, 0.2, 0.8);
  matPlane->m_specular.set(0.0, 0.0, 0.8);
  tPlane->setMaterial(*matPlane);
  tPlane->setTransparencyRenderMode(true, false);

  // enable and set transparency level of ground
  tPlane->setTransparencyLevel(0.5);
  tPlane->setUseTransparency(true);
  tPlane->setUseMaterial(true);
  tPlane->setUseCulling(false);

}

