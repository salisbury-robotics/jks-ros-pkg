//===========================================================================
/*
    CS277 - Experimental Haptics
    Winter 2011, Stanford University

    This class encapsulates the visual and haptic rendering for a point cloud.
    It inherits the regular CHAI3D cMesh class.

    \author    Adam Leeper
    \date      January 2011
*/
//===========================================================================

#include "PointCloudObject.h"

#include <vector>
#include <algorithm>
#include <ctime>
//#include <unistd.h>

//#include "kdtree.h"

PointCloudObject::PointCloudObject(cWorld *a_world)
    : cMesh(a_world)
    , m_projectedSphere(0.05)
    , m_active_radius(0)
    //, tree(new pcl::octree::OctreePointCloud<PointT> (0.005))
    , m_shape(0)
    , last_tree     ( new pcl::KdTreeFLANN <PointT> ()       )
    , last_points   ( new pcl::PointCloud  <PointT> ()       )
    , last_normals  ( new pcl::PointCloud  <pcl::Normal> ()  )
//    , m_cloud_points (new pcl::PointCloud<PointT> ())
//    , m_cloud_normals (new pcl::PointCloud<pcl::Normal> ())
    , scaleZ(1.0)
    , m_tool(0)
{
    // because we are haptically rendering this object as an implicit surface
    // rather than a set of polygons, we will not need a collision detector
    setCollisionDetector(0);

//    for(unsigned int count = 0; count < NUM_CLOUDS; count++)
//      tree[count] = boost::shared_ptr(new pcl::KdTreeFLANN<PointT> ());


    // Stuff I added...
    m_interactionInside = false;
    m_showTangent = true;
    m_useFriction = true;
    m_isReady = false;
    m_normalsFlip = false;
    m_refresh_cloud = false;

    //----------------------------
    // Tangent plane

    tPlane = new cPlane(a_world);
    this->addChild(tPlane);

    // --------------------------------------------
    
    // create a sphere that tracks the position of the proxy
    m_projectedSphere.m_material.m_ambient.set(0.3, 0.3, 0.0);
    m_projectedSphere.m_material.m_diffuse.set(0.8, 0.8, 0.0);
    m_projectedSphere.m_material.m_specular.set(1.0, 1.0, 1.0);
    addChild(&m_projectedSphere);
}


//! Contains code for graphically rendering this object in OpenGL.
void PointCloudObject::render(const int a_renderMode)
{
    // update the position and visibility of the proxy sphere
    m_projectedSphere.setShowEnabled(m_interactionInside);
    m_projectedSphere.setPos(m_interactionProjectedPoint);
    
    // get the base class to render the mesh
    renderCloud(a_renderMode);
}

bool PointCloudObject::evaluateCloud(const std::vector< std::vector<int> > &all_indices,
                                     const cVector3d &position, double &func, cVector3d &grad, int basis_type)
{
    func = 0;
    grad.zero();
    const cVector3d &t = position;

    // if( m_neighbors.size() == 0 ) return false; // indicates failure
    if( all_indices.size() == 0 ) return false; // indicates failure

    // variables for Wyvill
    double a=0, b=0, c=0, R=0, R2=0, R4=0, R6=0, a1=0, b1=0, c1=0, a2=0, b2=0, c2=0;

    // for surfels
    double denom = 0;
    cVector3d a_x(0,0,0);
    cVector3d n_x(0,0,0);

//    R = m_active_radius;
//    R2 = R*R;
//    R4 = R2*R2;
//    R6 = R4*R2;
//
//    if(basis_type == hviz::Haptics_WYVILL)
//    {
//      a = -4.0/9.0; b  = 17.0/9.0; c = -22.0/9.0;
//      a1 = a/R6; b1 = b/R4; c1 = c/R2;
//      a2 = 6*a1; b2 = 4*b1; c2 = 2*c1;
//    }
//    else if(basis_type == hviz::Haptics_WENDLAND)
//    {
//      //Do nothing?
//    }

    std::list< pcl::KdTree<PointT>::Ptr >::iterator          it_tree;
    std::list< pcl::PointCloud<PointT>::Ptr >::iterator      it_points;
    std::list< pcl::PointCloud<pcl::Normal>::Ptr >::iterator it_normals;
    int index = 0;
    for ( it_tree=trees.begin(), it_points=m_cloud_points.begin(), it_normals=m_cloud_normals.begin() ;
          it_tree != trees.end();
          it_tree++, it_points++, it_normals++, index++ )
    {
      const std::vector<int> &indices = all_indices[index];
      for(unsigned int i = 0; i < indices.size(); i++)
      {
          int index = indices[i];
          PointT pt = (*it_points)->points[index];
          pcl::Normal nv;
          //if(m_cloud_normals->points.size() == m_cloud_points->points.size())
          nv = (*it_normals)->points[index];
          cVector3d v = cVector3d(pt.x, pt.y, pt.z);
          cVector3d n = cVector3d(nv.normal[0], nv.normal[1], nv.normal[2]);

          if(m_configPtr->auto_threshold)
            R = pt.data[3];
          else
            R = m_active_radius;
          R2 = R*R;
          R4 = R2*R2;
          R6 = R4*R2;

          if(basis_type == hviz::Haptics_WYVILL)
          {
            a = -4.0/9.0; b  = 17.0/9.0; c = -22.0/9.0;
            a1 = a/R6; b1 = b/R4; c1 = c/R2;
            a2 = 6*a1; b2 = 4*b1; c2 = 2*c1;
          }


          double f_val = 0;
          cVector3d f_grad(0,0,0);

          cVector3d pos = (t-v);
          pos.z *= scaleZ;
          double r = pos.length();
          if(r > R)  // must skip points outside valid bounds.
            continue;
          double r2 = r*r;
          double r3 = r*r2;
          double r4 = r2*r2;
          double r5 = r3*r2;
          double r6 = r3*r3;

          pos.normalize();
          if(basis_type == hviz::Haptics_WYVILL)
          {
            f_val = (a1*r6 + b1*r4 + c1*r2 + 1);
            f_grad = (a2*r5 + b2*r3 + c2*r)*pos;
          }
          else if(basis_type == hviz::Haptics_WENDLAND)
          {
            double r_scaled = r/R;
            // TODO still need to address the scaling...
            f_val = pow((1-r_scaled),4)*(4*r_scaled + 1);
            f_grad = (-4.0/R*pow(1.0-r_scaled,3)*(4.0*r_scaled+1.0)+4.0/R*pow(1-r_scaled,4))*pos;
          }

          if(this->m_cloudType == hviz::Haptics_METABALLS)
          {
            func -= f_val;
            grad -= f_grad;
          }
          else if(   m_cloudType == hviz::Haptics_SURFELS_CLOSED
                  || m_cloudType == hviz::Haptics_SURFELS_OPEN )
          {
            a_x += f_val * v;
            n_x += f_val * n;
            denom += f_val;
          }
      }
    }

    if(this->m_cloudType == hviz::Haptics_METABALLS)
    {
      func += m_configPtr->meta_thresh * (m_configPtr->num_clouds - 1);
    }
    else if(   m_cloudType == hviz::Haptics_SURFELS_CLOSED
            || m_cloudType == hviz::Haptics_SURFELS_OPEN )
    {
      a_x /= denom;
      n_x /= denom;

      func = n_x.dot(t - a_x);
      grad = cNormalize(n_x);
    }

    return true; // it worked
}

//! Contains seed to surface point algorithm.
bool PointCloudObject::seed2surface(const std::vector< std::vector<int> > &indices,
                                    cVector3d &p, cVector3d &grad, double epsilon, int max_steps)
{
  int steps = 0;
  cVector3d dp;
    while(true){
        double funcL = 0; 

        if(!evaluateCloud(indices, p, funcL, grad, m_basisType)) return false;

        dp =  cMul(-funcL/cDot(grad, grad), grad);
        p  = p + dp; 
        if(dp.length() < epsilon) return true;
        if( ++steps > max_steps) 
        {
          printf("Exceeded number of steps!\n");
          return false;
        }
    }
}

int PointCloudObject::computeNeighborIndices( PointT position, float radius,
                                              std::vector< std::vector <int> >    &k_indices,
                                              std::vector< std::vector <float> >  &k_sqr_distances)
{
  int NN = 0;
  std::list< pcl::KdTree<PointT>::Ptr >::iterator          it_tree;
  std::list< pcl::PointCloud<PointT>::Ptr >::iterator      it_points;
  std::list< pcl::PointCloud<pcl::Normal>::Ptr >::iterator it_normals;
  int index = 0;
  for ( it_tree=trees.begin(), it_points=m_cloud_points.begin(), it_normals=m_cloud_normals.begin() ;
        it_tree != trees.end();
        it_tree++, it_points++, it_normals++, index++ )
  {
    NN += (*it_tree)->radiusSearch ( position,
                                        radius,
                                        k_indices[index],
                                        k_sqr_distances[index]
                                      );
  }

  return NN;
}

//===========================================================================
/*!
    This method should contain the core of the implicit surface rendering
    algorithm implementation.  The member variables m_interactionProjectedPoint
    and m_interactionInside should both be set by this method.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void PointCloudObject::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // -----------------------------------------------------------------------------

    //boost::recursive_mutex::scoped_lock lock(mutex_);
    if(m_ghostStatus || !m_isReady){
        m_interactionProjectedPoint = a_toolPos;
        m_interactionInside = false;
        return;
    }

    // Storage for function value and gradient
    double func = 0;
    cVector3d grad(0,0,1);

    // Other useful storage
    cVector3d seed(0,0,0);
    cVector3d prevPos = tPlane->getPos();
    double epsilon = 0.00001;
    const cVector3d *pt;

//    printf("Surfels: %d Tool: %.3f %.3f %.3f Value % 10.4f,  %d vtx within % 4.2f of tool. \r",
//           m_useSurfels, a_toolPos.x, a_toolPos.y, a_toolPos.z,
//           func, m_neighbors.size(), m_configPtr->basis_radius);
    if(m_configPtr->auto_threshold)
    {
      PointT position;
      position.x = m_interactionProjectedPoint.x;
      position.y = m_interactionProjectedPoint.y;
      position.z = m_interactionProjectedPoint.z;
      std::vector< int >   	k_indices(1, 0);
      std::vector< float >  k_sqr_distances(1, 0);
      int N = trees.front()->nearestKSearch ( position, 1, k_indices, k_sqr_distances);
      if(N)
      {
        int index = k_indices[0];
        m_active_radius = m_cloud_points.front()->points[index].data[3]*m_configPtr->radius_multiple;
        //printf("Active radius is %.3f \n", m_active_radius);
      }
    }
    else
      m_active_radius = m_configPtr->basis_radius;





  // -------------------------------------------------------------------
  // If we are not in contact we check for penetration into the surface
  if( !m_interactionInside )
  {
    pt = &a_toolPos;
    PointT position;
    position.x = pt->x;
    position.y = pt->y;
    position.z = pt->z;

    std::vector< std::vector <int> >    k_indices(trees.size());
    std::vector< std::vector <float> >  k_sqr_distances(trees.size());
    computeNeighborIndices(position, m_active_radius, k_indices, k_sqr_distances);
    evaluateCloud(k_indices, *pt, func, grad, m_basisType);
    //ROS_INFO("Value is %f\n", func);

    if( func < 0 ) {
      //ROS_INFO("Initial contact! Value is %f\n", func);
      m_interactionInside = true;
      seed = a_toolPos;

      // seed2surface is defined above :)
      seed2surface(k_indices, seed, grad, epsilon); //Step point to surface and store new point and gradient
      m_interactionProjectedPoint = seed; // Not actually necessary (will happen again later)

      //Using the tangent plane as our storage for position and surface normal.
      tPlane->m_normal = grad;
      tPlane->m_normal.normalize();
      tPlane->setPos(seed);
    }
    else{
      tPlane->setShowEnabled(false); // disable tangent plane
      tPlane->setPos(a_toolPos);      // have it follow the tool... not really needed
      m_interactionProjectedPoint = a_toolPos;
      m_interactionInside = false;
      return;
    }
  }

  if( m_interactionInside ){
    // Project tool position onto the tangent plane
    seed = cProjectPointOnPlane(a_toolPos, tPlane->getPos(), tPlane->m_normal);

    if(m_useFriction)
    {
      double mu = m_material.getDynamicFriction();
      cVector3d Fn(0,0,0), Ft(0,0,0), F(0,0,0);
      F = a_toolPos - prevPos;
      F.decompose(tPlane->m_normal,Fn, Ft);
      cVector3d FxFn = F.crossAndReturn(Fn);
      double magFxFn = FxFn.length();
      double FdotFn = F.dot(Fn);
      if( FdotFn  < 0.0000000001 ) FdotFn = 0.0000000001;

      //ROS_INFO("mu: %f, frac: %f", mu, magFxFn / FdotFn);
      if( (magFxFn / FdotFn) > mu )
      {
        cVector3d diff = prevPos - seed;
        if(diff.length() > 0.0000000001)
          seed = seed + Fn.length()*mu*cNormalize(diff);
      }
      else
      {
        seed = prevPos;
      }
    }

    // limit proxy movement...
    cVector3d prev2seed = seed - prevPos;
    if(prev2seed.length() > m_active_radius/2){
        prev2seed.normalize();
        prev2seed *= m_active_radius/2;
        seed = prevPos + prev2seed;
    }

    //tree->neighbors(cVertex( seed.x, seed.y, seed.z), m_configPtr->basis_radius, m_neighbors);
    PointT position;
    position.x = seed.x;
    position.y = seed.y;
    position.z = seed.z;

    std::vector< std::vector <int> >    k_indices(trees.size());
    std::vector< std::vector <float> >  k_sqr_distances(trees.size());
    int NN = computeNeighborIndices(position, m_active_radius, k_indices, k_sqr_distances);

    //printf("my tree found %d neighbors, pcl tree found %d neighbors\n", m_neighbors.size(), k_indices.size());
    if(NN == 0)
    {
      //tree->neighbors(cVertex( prevPos.x, prevPos.y, prevPos.z), m_configPtr->basis_radius, m_neighbors);
      position.x = prevPos.x;
      position.y = prevPos.y;
      position.z = prevPos.z;
      NN = computeNeighborIndices(position, m_active_radius, k_indices, k_sqr_distances);
      if(NN == 0)
      {
        m_interactionInside = false;
        return;
      }
    }

//    m_neighbors.clear();
//    m_neighbors.reserve(NN);
//    for(int i = 0; i < NN; i++)
//    {
//      cVertex v;
//
//      PointT p = m_cloud_points->points[k_indices[i]];
//      pcl::Normal n = m_cloud_normals->points[k_indices[i]];
//      v.m_localPos = cVector3d(p.x, p.y, p.z);
//      v.m_normal = cVector3d(n.normal[0], n.normal[1], n.normal[2]);
//      m_neighbors.push_back(v);
//    }

    if(!seed2surface(k_indices, seed, grad, epsilon)){
        //printf("Lost the surface... \n");
        m_interactionInside = false;
    }
    else
    {
        //printf("done!  ");

        m_interactionProjectedPoint = seed;
        m_interactionInside = true;

        // ----------------------------------
        // Update tangent plane visualization
        tPlane->setShowEnabled(m_showTangent);
        tPlane->setPos(seed);
        grad.normalize();
        tPlane->m_normal = 0.4*grad + 0.6*tPlane->m_normal;
        tPlane->m_normal.normalize();
        cVector3d rotX = cNormalize(tPlane->m_normal.crossAndReturn(cVector3d(0,0,1)));
        cVector3d rotY = tPlane->m_normal.crossAndReturn(rotX);
        cMatrix3d rot;
        rot.setCol(rotX, rotY, tPlane->m_normal);
        tPlane->setRot(rot);
        // -----------------------------------

        //printf(" Finished \n");

        if( cDot(tPlane->m_normal, a_toolPos - prevPos) > 0)
        {   
            //printf("Lost contact! \n");
            m_interactionInside = false;
            m_interactionProjectedPoint = a_toolPos;
          }
    }
 }

  if(m_tool)
    m_tool->setRadius(0.01 * !m_interactionInside);
            
    // m_interactionInside should be set to true when the tool is in contact
    // with the object.
    // -----------------------------------------------------------------------------
      
}
