#ifndef _POINTSAMPLER_H_
#define _POINTSAMPLER_H_

#include "Sampler.h"

#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/recursive_mutex.hpp>

#include <list>



// The PointSampler performs trilinear interpolation to sample a volume in
// 0-1 cube space with correct physical proportions (ie. the maximum physical
// dimension of the volume is scaled to fit in the range 0,1).  Index
// coordinates are double-precision values, and the returned samples are
// single-precision floats.  Templatization for other variations may be
// developed later if needed.

// TODO template on the point type.
typedef pcl::PointXYZRGB PointT;

class PointSampler : public Sampler
{
protected:

  //! Gets all points within a given radius of a given position. Result is stored in given vectors.
  int computeNeighborIndices( const PointT &position, float radius,
                                                std::vector< std::vector <int> >    &k_indices,
                                                std::vector< std::vector <float> >  &k_sqr_distances);

  //! Computes the intensity and gradient of the cloud at a given positon.
  bool sampleCloud(const cml::vector3d &p, float &intensity, cml::vector3d &gradient);

  //! Calculate average point spacing, etc.
  void computeCloudSpecs();



  float m_active_radius;
  float m_radius_multiple;

  //std::list< pcl::octree::OctreePointCloud<PointT>::Ptr > trees;

  //! A list of search objects.
  std::list< pcl::KdTree<PointT>::Ptr >           trees;

  //! A list of clouds containing points.
  std::list< pcl::PointCloud<PointT>::Ptr >       m_cloud_points;

  //! A list of clouds containing normals.
  std::list< pcl::PointCloud<pcl::Normal>::Ptr >  m_cloud_normals;

  //! A search object waiting to be added to the internal list.
  pcl::KdTree<PointT>::Ptr            last_tree;

  //! A cloud of points waiting to be added to the internal list.
  pcl::PointCloud<PointT>::Ptr        last_points;

  //! A cloud of normals waiting to be added to the internal list.
  pcl::PointCloud<pcl::Normal>::Ptr   last_normals;

  //! A mutex for locking stuff to prevent nasty fiery explosions of death.
  boost::recursive_mutex mutex_;


public:

  //! Default constructor
  PointSampler();

  //! enum definition for available radial basis functions
  enum BasisType {WYVILL, WENDLAND};

  //! enum definition for available radial basis functions
  enum CloudType {METABALLS, CLOSED_SURFELS, OPEN_SURFELS};

  //! enum instance to store the active basis function type
  BasisType m_basis_type;

  //! enum instance to store the active basis function type
  CloudType m_cloud_type;

  //! Number of clouds to use for smoothing
  size_t m_cloud_list_size;

  //! Applies the most recently constructed cloud to the internal cloud list.
  void applyLastCloud();

  //! Constructs a cloud for haptic rendering from the input cloud.
  bool createFromCloud(const pcl::PointCloud<PointT> &cloud);

  //! Convenience function to create a spherical shell.
  void createSphericalShell(double radius, double angle_step,
                                          double min_lat, double max_lat,
                                          double min_lon, double max_lon,
                                          double noise_size);

  //! Convenience function to create an open-top box (5-sides).
  void createOpenTopBox(double edge_size, double linear_step,
                                          double noise_size);

  //! Creates a plane that matches the VolumeTestData in sss. For debugging only.
  void createPlane();

  //! Clears all cloud data stored in this sampler.
  void deleteAllClouds();

  //! Gets a cloud by index... (does this work?)
  void getPointCloud(const int &index, pcl::PointCloud<PointT>::Ptr &cloud);

  //! Gets a normal cloud by index... (does this work?)
  void getNormalCloud(const int &index, pcl::PointCloud<pcl::Normal>::Ptr &cloud);

  //! Get the metaball radius
  float getActiveRadius() {return m_active_radius; }

  //! Get the metaball radius
  void setActiveRadius(float r) { m_active_radius = r; }

  //! Get the metaball radius
  float getRadiusMultiplier() {return m_radius_multiple; }

  //! Get the metaball radius
  void setRadiusMultiplier(float m) { m_radius_multiple = m; }

  //! Returns the function value (intensity) at position p
  virtual float           intensityAt(const cml::vector3d &p);

  //! Returns the function gradient at position p
  virtual cml::vector3f   gradientAt(const cml::vector3d &p);
};

#endif // _POINTSAMPLER_H
