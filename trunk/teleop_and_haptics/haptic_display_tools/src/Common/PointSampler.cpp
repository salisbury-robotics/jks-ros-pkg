#include "PointSampler.h"
#include <algorithm>
#include <ros/ros.h>
#include <limits>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace cml;

typedef cml::vector3d vectorT;

PointSampler::PointSampler() :
    m_active_radius(1.4),
    m_radius_multiple(2.5),
    m_basis_type(PointSampler::WYVILL),
    m_cloud_type(PointSampler::METABALLS),
    m_cloud_list_size(1)
{

}

int PointSampler::computeNeighborIndices( const PointT &position, float radius,
                                              std::vector< std::vector <int> >    &k_indices,
                                              std::vector< std::vector <float> >  &k_sqr_distances)
{
  int NN = 0;
  std::list< pcl::search::KdTree<PointT>::Ptr >::iterator  it_tree;
  std::list< pcl::PointCloud<PointT>::Ptr >::iterator      it_points;
  std::list< pcl::PointCloud<pcl::Normal>::Ptr >::iterator it_normals;
  int index = 0;
  for ( it_tree=trees.begin(), it_points=m_cloud_points.begin(), it_normals=m_cloud_normals.begin() ;
        it_tree != trees.end();
        it_tree++, it_points++, it_normals++, index++ )
  {
    NN += (*it_tree)->radiusSearch (    position,
                                        radius,
                                        k_indices[index],
                                        k_sqr_distances[index]
                                      );
  }

  return NN;
}

bool PointSampler::sampleCloud(const vectorT &p, float &intensity, vectorT &gradient)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  intensity = 0.f;
  gradient.zero();

  PointT position;
  position.x = p[0]; position.y = p[1]; position.z = p[2];

  std::vector< std::vector <int> >    all_indices(trees.size());
  std::vector< std::vector <float> >  k_sqr_distances(trees.size());
  int NN = computeNeighborIndices(position, m_active_radius, all_indices, k_sqr_distances);

  if(NN == 0)
  {
    //ROS_INFO("Found no neighbors in %d clouds within distance %.3f m", trees.size(), m_active_radius);
    return false;
  }
  //ROS_INFO("Found %d neighbors in %zd clouds within distance %.3f m", NN, trees.size(), m_active_radius);

  // variables for Wyvill
  double a=0, b=0, c=0, R=0, R2=0, R4=0, R6=0, a1=0, b1=0, c1=0, a2=0, b2=0, c2=0;

  // for surfels
  double denom = 0;
  vectorT a_x(0,0,0);
  vectorT n_x(0,0,0);

  std::list< pcl::search::KdTree<PointT>::Ptr >::iterator          it_tree;
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
        vectorT v(pt.x, pt.y, pt.z);
        vectorT n(nv.normal[0], nv.normal[1], nv.normal[2]);

         //ROS_INFO("point (%.3f %.3f, %.3f)", v[0], v[1], v[2]);

//        if(m_configPtr->auto_threshold)
//          R = pt.data[3];
//        else
        R = m_active_radius;
        R2 = R*R;
        R4 = R2*R2;
        R6 = R4*R2;

        if(m_basis_type == PointSampler::WYVILL)
        {
          a = -4.0/9.0; b  = 17.0/9.0; c = -22.0/9.0;
          a1 = a/R6; b1 = b/R4; c1 = c/R2;
          a2 = 6*a1; b2 = 4*b1; c2 = 2*c1;
        }
        else
        {
          ROS_ERROR("This should not be called!");
        }

        double f_val = 0;
        vectorT f_grad(0,0,0);

        vectorT pos = (p-v);
        //pos.z *= scaleZ;
        double r = pos.length();
        if(r > R)  // must skip points outside valid bounds.
        {
          //ROS_WARN("Skipping a point (%.3f %.3f, %.3f)", v[0], v[1], v[2]);
          continue;
        }
        double r2 = r*r;
        double r3 = r*r2;
        double r4 = r2*r2;
        double r5 = r3*r2;
        double r6 = r3*r3;

        pos.normalize();
        if(m_basis_type == PointSampler::WYVILL)
        {
          f_val = (a1*r6 + b1*r4 + c1*r2 + 1);
          f_grad = (a2*r5 + b2*r3 + c2*r)*pos;
        }
        else if(m_basis_type == PointSampler::WENDLAND)
        {
          ROS_ERROR("This should not be called!");
          double r_scaled = r/R;
          // TODO still need to address the scaling...
          f_val = pow((1-r_scaled),4)*(4*r_scaled + 1);
          f_grad = (-4.0/R*pow(1.0-r_scaled,3)*(4.0*r_scaled+1.0)+4.0/R*pow(1-r_scaled,4))*pos;
        }

        if(this->m_cloud_type == PointSampler::METABALLS)
        {
//          func -= f_val;
//          grad -= f_grad;

          // TODO:  The whole library should be overhauled to follow the "gradient points out"
          //        convention of implicit functions.
          intensity += f_val;
          gradient += f_grad;
        }
        else if(   m_cloud_type == PointSampler::CLOSED_SURFELS
                || m_cloud_type == PointSampler::OPEN_SURFELS )
        {
          ROS_ERROR("This should not be called!");
          a_x += f_val * v;
          n_x += f_val * n;
          denom += f_val;
        }
    }
  }

//  if(this->m_cloudType == hviz::Haptics_METABALLS)
//  {
//    func += m_configPtr->meta_thresh * (m_configPtr->num_clouds - 1);
//  }
  if(   m_cloud_type == PointSampler::CLOSED_SURFELS
        || m_cloud_type == PointSampler::OPEN_SURFELS )
  {
    ROS_ERROR("This should not be called!");
    a_x /= denom;
    n_x /= denom;

    intensity = -1.0*cml::dot(n_x, p - a_x);
    gradient = -n_x.normalize();
  }

  return true; // it worked
}

float PointSampler::intensityAt(const cml::vector3d &p)
{
  float intensity;
  vectorT gradient;
  sampleCloud(p, intensity, gradient);
  //ROS_INFO("returning with intensity % 6.2f", intensity);
  return intensity;

//  float radius = 0.2;
//  return std::max<float>(5*(radius - p.length()), 0);
}

vector3f PointSampler::gradientAt(const cml::vector3d &p)
{
  float intensity;
  vectorT gradient;
  sampleCloud(p, intensity, gradient);
//  ROS_INFO("returning with gradient % .2f % .2f % .2f, magnitude %.2f",
//           gradient[0], gradient[1], gradient[2], gradient.length());
  return gradient;

//  return -2*p;

  /* Numerical method, if we care ... though for this it would be *much* slower.
    vector3d dx(m_gradientDelta, 0.0, 0.0);
    vector3d dy(0.0, m_gradientDelta, 0.0);
    vector3d dz(0.0, 0.0, m_gradientDelta);

    vector3f g;
    g[0] = intensityAt(p + dx) - intensityAt(p - dx);
    g[1] = intensityAt(p + dy) - intensityAt(p - dy);
    g[2] = intensityAt(p + dz) - intensityAt(p - dz);

    return 0.5 * g / m_gradientDelta;
    */

}

//! This function may not be thread safe. Only call at the right time!
void PointSampler::applyLastCloud()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  ROS_INFO_NAMED("haptics", "Before push Num stored clouds: %zd %zd %zd",
                    trees.size(), m_cloud_points.size(), m_cloud_normals.size() );
  trees.push_front(last_tree);
  m_cloud_points.push_front(last_points);
  m_cloud_normals.push_front(last_normals);

  ROS_INFO_NAMED("haptics", "After push stored clouds: %zd %zd %zd",
                    trees.size(), m_cloud_points.size(), m_cloud_normals.size() );

  while( trees.size() > m_cloud_list_size )
  {
    trees.pop_back();
    m_cloud_points.pop_back();
    m_cloud_normals.pop_back();
    ROS_INFO_NAMED("haptics", "After pop stored clouds: %zd %zd %zd",
                      trees.size(), m_cloud_points.size(), m_cloud_normals.size() );
  }
}


// This could be useful at some point.
//bool PointSampler::loadPCD(const std::string file)
//{
//
//
//}

//! Queues up a cloud to apply to the internal cloud list.
bool PointSampler::createFromCloud(const pcl::PointCloud<PointT> &cloud)
{
//   // We may want this in the future... though perhaps it is better to
//   // throw away older clouds in favor of newer ones.
//   if(m_refresh_cloud)
//     return false;

  boost::recursive_mutex::scoped_lock lock(mutex_);
  ROS_INFO_NAMED("haptics", "Loading cloud with %zd points!", cloud.size());

  ros::WallTime begin = ros::WallTime::now();

  // Create a KD-Tree
  last_points.reset( new pcl::PointCloud<PointT> ());
  *last_points = cloud;

  ROS_INFO_NAMED("haptics", "Building kdtree...");
  last_tree.reset(new pcl::search::KdTree<PointT> ());
  last_tree->setInputCloud (last_points);
  //tree->addPointsFromInputCloud();

  ROS_INFO_NAMED("time", "Initializing kdtree took %.3lf ms",
                  (ros::WallTime::now() - begin).toSec()*1000.0);

  last_normals.reset(new pcl::PointCloud<pcl::Normal> ());
  if( true )
  {
    begin = ros::WallTime::now();
    ROS_INFO_NAMED("haptics", "Estimating normals...");
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne_omp;

    // Set parameters
    ne_omp.setInputCloud (last_points);
    ne_omp.setSearchMethod(last_tree);
    ne_omp.setKSearch(9);
    // Reconstruct
    ROS_INFO_NAMED("haptics", "computing...");
    ne_omp.compute(*last_normals);
    ROS_INFO_NAMED("haptics", "...done!");
    ROS_INFO_NAMED("time", "OMP Normal estimation took %.3lf ms",
                    (ros::WallTime::now() - begin).toSec()*1000.0);
  }

//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud (m_cloud_points);
//  //  viewer.showCloud (m_cloud_normals);
//  viewer.showCloud();
//  while (!viewer.wasStopped ())
//  {
//  }

  computeCloudSpecs();

  //ROS_INFO("All done!\n");

  return true;
}

void PointSampler::computeCloudSpecs()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  ros::Time begin = ros::Time::now();
  ROS_INFO_NAMED("haptics", "Computing cloud specs ... %f", begin.toSec());
  double average_distance = 0;
  int NN = 3;
  if(last_points->size() != 0){
    std::vector< int >   	k_indices(NN+1, 0);
    std::vector< float >  k_sqr_distances(NN+1, 0);
    for(size_t index = 0; index < last_points->size(); index++)
    {
      //printf("Index: %d, size %d\n", index, last_points->size());
      int N = last_tree->nearestKSearch ( index, NN+1, k_indices, k_sqr_distances);
      if( N == 0 ) continue;
      float distance = 0;
      for(int d = 1; d < N; d++)
      {
        //printf("For seed %d, neighbor %d distance %f\n", index, k_indices[d], sqrt(k_sqr_distances[d]));
        distance += sqrt(k_sqr_distances[d]);
      }
      distance = 1.0/(N-1)*distance;

      // TODO fix this so it uses intensity, then preserve the color channel
      last_points->points[index].data[3] = distance * m_radius_multiple;
      average_distance += distance;
    }
    average_distance /= last_points->size();
  }

  m_active_radius = m_radius_multiple*average_distance;
  ROS_INFO_NAMED("info",
                  "Average spacing is %f , multiplier is %.1f, setting to % 5.3f.",
      average_distance, m_radius_multiple, m_active_radius);

//  if(m_configPtr->auto_threshold){
//    m_active_radius = m_configPtr->radius_multiple*average_distance;
//    m_configPtr->basis_radius= m_active_radius;
//    //m_configPtr->meta_thresh = THRESH_MULT*m_configPtr->basis_radius;
//    //m_configPtr->meta_thresh = 0.5;
//    ROS_INFO("Average separation in cloud is %f , setting basis_radius to % 5.3f and threshold to % 5.2f ",
//    average_distance, m_configPtr->basis_radius, m_configPtr->meta_thresh);
//  }
//  else
//  {
//    m_active_radius = m_configPtr->basis_radius;
//  }

  ros::Time end = ros::Time::now();
  ros::Duration elapsed = end-begin;
  //ROS_INFO("Begin: %f, end: %f, elapsed: %f", begin.toSec(), end.toSec(), elapsed.toSec());
  ROS_INFO_NAMED("time", "Computing cloud specs took %.3lf ms", elapsed.toSec()*1000.0);
}


//! Convenience function to create part of a spherical shell.
void PointSampler::createSphericalShell(double radius, double angle_step,
                                        double min_lat, double max_lat,
                                        double min_lon, double max_lon,
                                        double noise_size)
{
    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian
    srand(time(0));

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    for( double theta = min_lat; theta < max_lat; theta +=angle_step )
    {
        for( double phi = min_lon; phi < max_lon; phi+= angle_step  )
        {
            double x = radius*cos(phi)*cos(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double y = radius*sin(phi)*cos(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double z = radius*sin(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud.push_back(point);
        }
    }
    ROS_INFO("Creating a spherical cloud with %zd points!\n", cloud.size());
    this->createFromCloud(cloud);
}

//! Convenience function to create an open-top box (5-sides).
void PointSampler::createOpenTopBox(double edge_size, double linear_step,
                                        double noise_size)
{
    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian
    srand(time(0));

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    for( int face = 0; face < 5; face++)
    {
        for( double u = -edge_size/2; u < edge_size/2; u += linear_step )
        {
            for( double v = -edge_size/2; v < edge_size/2; v += linear_step )
            {
                double x = (face == 0) * edge_size/2 + (face == 1) * -edge_size/2 + u * (face == 2 || face == 3 || face == 4 || face == 5);
                double y = (face == 2) * edge_size/2 + (face == 3) * -edge_size/2 + u * (face == 0 || face == 1) + v * (face == 4 || face == 5);
                double z = (face == 4) * -edge_size/2 + (face == 5) * edge_size/2 + v * (face == 0 || face == 1 || face == 2 || face == 3);

                x += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
                y += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
                z += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;

                PointT point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud.push_back(point);
            }
        }
    }
    ROS_INFO("Creating an open-top box with %zd points!\n", cloud.size());
    this->createFromCloud(cloud);
}

//! Convenience function to create an open-top box (5-sides).
void PointSampler::createPlane()
{
    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian
    srand(time(0));

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    for(int i = 1; i <= 3; i++)
    {
      for(int j = -4; j <= 4; j++)
      {
        PointT point;
        point.x = i*0.02;
        point.y = j*0.02;
        point.z = 0;
        cloud.push_back(point);
      }

    }

    ROS_INFO("Creating a plane with %zd points!\n", cloud.size());
    this->createFromCloud(cloud);
}

//! Clears all cloud data stored in this sampler.
void PointSampler::deleteAllClouds()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  // This might make everything crash... we'll see.
  trees.clear();
  m_cloud_points.clear();
  m_cloud_normals.clear();
}

//! Returns a point cloud from the internal list.
void PointSampler::getPointCloud(const int &index, pcl::PointCloud<PointT>::Ptr &cloud)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if(m_cloud_points.size())
    *cloud = *m_cloud_points.front();
}

//! Returns a normal cloud from the internal list.
void PointSampler::getNormalCloud(const int &index, pcl::PointCloud<pcl::Normal>::Ptr &cloud)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if(m_cloud_normals.size())
    *cloud = *m_cloud_normals.front();
}
