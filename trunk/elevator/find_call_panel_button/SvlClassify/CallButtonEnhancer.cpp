#include "CallButtonEnhancer.h"


#define MAX_PR_DEV 0.01
#define CLUSTER_OVERLAP 0.1

int getNumberOfClusters(svlObject2dFrame &objectList);


CallButtonEnhancer::CallButtonEnhancer(){
}

CallButtonEnhancer::~CallButtonEnhancer() {
}

svlObject2dFrame CallButtonEnhancer::process(IplImage* image,svlObject2dFrame &objects, int maxWidth) 
{
  vector<pair<int,svlObject2d> > temp_clusters, clusters;

  cout << "Initiating CallButtonEnhancer processing... " << endl;

  maxObjWidth = maxWidth;
  svlObject2dFrame processed_objects;
  svlObject2dFrame temp_objects;

  for (int i=0; i<objects.size(); i++) 
    {
      temp_objects.push_back(objects[i]);
    }
  /*  processed_objects = pruneList(temp_objects);
  
      temp_objects.clear();
      for (int i=0; i<processed_objects.size(); i++) 
      {
      temp_objects.push_back(processed_objects[i]);
      }
  */
  //  temp_clusters = clusterCandidateObjects(temp_objects);
  clusters = clusterCandidateObjects(temp_objects);

  //  clusters =  getBestCandidates(temp_clusters);

  //  clusters = clusterButtonCenters(temp_clusters,temp_objects);
  
  processed_objects.clear();
  for (int i=0; i<clusters.size(); i++) {
    if(clusters[i].second.x + clusters[i].second.w < image->width && clusters[i].second.y + clusters[i].second.h < image->height){
      clusters[i].second.pr = clusters[i].first;
      processed_objects.push_back(clusters[i].second);
    }
  }
  return processed_objects;
}


svlObject2dFrame CallButtonEnhancer::pruneList(svlObject2dFrame &objects) 
{

  double maxPrDev = MAX_PR_DEV;
  double maxPr = 0;

  // Find max probability of any object.
  for (unsigned k = 0; k < objects.size(); k++) {
    if (objects[k].pr > maxPr) {
      maxPr = objects[k].pr;
    }
  }

  // Remove objects under threshold probability.
  svlObject2dFrame objectList;
  for (unsigned k = 0; k < objects.size(); k++) {
    if (objects[k].pr >= (maxPr-MAX_PR_DEV) && objects[k].w < maxObjWidth) {
      objectList.push_back(objects[k]);
    }
  }
  cout << objectList.size() << " in reduced list " << endl;

  return objectList;	
}


vector<pair<int,svlObject2d> > CallButtonEnhancer::clusterButtonCenters(vector<pair<int,svlObject2d> >& detect_centroids, svlObject2dFrame &objects){


  int numClusters= detect_centroids.size();
  int numObjects = objects.size();


  // Find cluster centroids using Kmeans.
  svlKMeansT<svlPoint2d> kmeans;
	
  // Initialize point and centroid svlPoint2d vectors
  vector<svlPoint2d> points;
  vector<svlPoint2d> centroids;
  double h_mean=0;
  double w_mean=0;

  bool add = false;
  for (int j=0; j<numObjects; j++) {
    add = false;
    for(int i =0; i< detect_centroids.size();i++){
      if(abs((objects[j].x + objects[j].w* 0.4 ) - (detect_centroids[i].second.x + detect_centroids[i].second.w* 0.4 )) < detect_centroids[i].second.w * 0.4
	 &&  abs((objects[j].y + objects[j].h* 0.4 ) - (detect_centroids[i].second.y + detect_centroids[i].second.w* 0.4 )) < detect_centroids[i].second.h* 0.4 ){
	add =true;
	break;

      }
    }
    if(add){
      points.push_back(svlPoint2d(objects[j].x,objects[j].y));
      w_mean += objects[j].w;
      h_mean += objects[j].h;
    }
  }

  h_mean = h_mean/numObjects;
  w_mean = w_mean/numObjects;
  double l_mean = (h_mean+w_mean)/2;

  for (int i=0; i<numClusters; i++) {
    centroids.push_back(svlPoint2d(detect_centroids[i].second.x,detect_centroids[i].second.y));
  }

  vector<int> cluster_idx;
  kmeans.do_kmeans(points,centroids,&cluster_idx,numClusters,-1);
  cout << "kmeans successful" << endl;
  vector<pair<int,svlObject2d> > buttons;
  for (int i=0; i<numClusters; i++) {
    pair<int,svlObject2d> p;
    p = make_pair(0,svlObject2d(centroids[i].x,centroids[i].y,l_mean,l_mean,0));
    buttons.push_back(p);
  }

  return buttons;
}

vector<pair<int,svlObject2d> > CallButtonEnhancer::clusterCandidateObjects(svlObject2dFrame &objects) 
{
  svlObject2dFrame detect_centroids;;
  int numObjects = objects.size();
  for (int i=0; i<numObjects; i++) {
    detect_centroids.push_back(objects[i]);
  }

  double numRemoved = removeOverlappingObjects(detect_centroids, CLUSTER_OVERLAP);
  cout << "Removed overlapping objects: " << numRemoved << " objects." << endl;
  int numClusters = detect_centroids.size();
  cout << "Number remaining: " << detect_centroids.size() << endl;
  cout << "Number of clusters to use for Kmeans: " << numClusters << endl;
  cout << "Number of objects passed to Kmeans: " << numObjects << endl;
	
  // Find cluster centroids using Kmeans.
  svlKMeansT<svlPoint2d> kmeans;
	
  // Initialize point and centroid svlPoint2d vectors
  vector<svlPoint2d> points;
  vector<svlPoint2d> centroids;
  double h_mean=0;
  double w_mean=0;

  for (int i=0; i<numObjects; i++) {
    cout << "OBJECT " << i << "x = " << objects[i].x << " y = " << objects[i].y << " w = " << objects[i].w <<endl;
    points.push_back(svlPoint2d(objects[i].x,objects[i].y));
    w_mean += objects[i].w;
    h_mean += objects[i].h;
  }
  h_mean = h_mean/numObjects;
  w_mean = w_mean/numObjects;
  double l_mean = (h_mean+w_mean)/2;

  for (int i=0; i<numClusters; i++) {
    centroids.push_back(svlPoint2d(detect_centroids[i].x,detect_centroids[i].y));
  }

  vector<int> cluster_idx;
  kmeans.do_kmeans(points,centroids,&cluster_idx,numClusters,-1);
  cout << "kmeans successful" << endl;

  vector<pair<int,svlObject2d> > clusterCentroids;

	
  for (int i=0; i<numClusters; i++) {
    pair<int,svlObject2d> p;
    p = make_pair(0,svlObject2d(centroids[i].x,centroids[i].y,l_mean,l_mean,0));
    clusterCentroids.push_back(p);
  }
  for (int i=0; i<numClusters; i++) {
    cout << "CENTROID " << i << "x = " << clusterCentroids[i].second.x << " y = " << clusterCentroids[i].second.y << " w = " << clusterCentroids[i].second.h <<endl;
    for (int j=0; j<numObjects; j++) {
      if(abs((objects[j].x + objects[j].w* 0.4 ) - (clusterCentroids[i].second.x + clusterCentroids[i].second.w* 0.4 )) < clusterCentroids[i].second.w * 0.4
	 &&  abs((objects[j].y + objects[j].h* 0.4 ) - (clusterCentroids[i].second.y + clusterCentroids[i].second.w* 0.4 )) < clusterCentroids[i].second.h* 0.4 ){
	clusterCentroids[i].first++;
      }
      if(abs((objects[j].x + objects[j].w* 0.4 ) - (clusterCentroids[i].second.x + clusterCentroids[i].second.w* 0.4 )) > clusterCentroids[i].second.w * 0.4
	 &&  abs((objects[j].y + objects[j].h* 0.4 ) - (clusterCentroids[i].second.y + clusterCentroids[i].second.h* 0.4 )) < clusterCentroids[i].second.h* 0.4 ){
	clusterCentroids[i].first-=2;
      }
    }
  }
  bool isAligned = false;
  for (int i=0; i<numClusters; i++) {
    isAligned=false;
    for (int j=0; j<numClusters; j++) {
      if(i!=j && abs(clusterCentroids[i].second.x - clusterCentroids[j].second.x) < clusterCentroids[i].second.w*0.4 && 
	 abs(clusterCentroids[i].second.y - clusterCentroids[j].second.y) < clusterCentroids[i].second.h*1.9 &&
	 abs(clusterCentroids[i].second.y - clusterCentroids[j].second.y) >= clusterCentroids[i].second.h*0.4){
	isAligned = true;
	break;
      }
    }
    if(isAligned){
      clusterCentroids[i].first+=5;
    }
  }


  sortByFirst(clusterCentroids);


  /*
    for (int i=0; i<objects.size(); i++) {
    clusterCentroids[cluster_idx[i]].second.x += objects[i].x;
    clusterCentroids[cluster_idx[i]].second.y += objects[i].y;
    clusterCentroids[cluster_idx[i]].second.w += objects[i].w;
    clusterCentroids[cluster_idx[i]].second.h += objects[i].h;
    clusterCentroids[cluster_idx[i]].second.pr += objects[i].pr;
    clusterCentroids[cluster_idx[i]].first--;
    //cout << objects[i].x << " " << objects[i].y << " " << cluster_idx[i] << endl; 	
    }
	
    for (int i=0; i<numClusters; i++) {
    clusterCentroids[i].second.x /= -clusterCentroids[i].first; 
    clusterCentroids[i].second.y /= -clusterCentroids[i].first;
    clusterCentroids[i].second.w /= -clusterCentroids[i].first; 
    clusterCentroids[i].second.h /= -clusterCentroids[i].first; 
    clusterCentroids[i].second.pr /= -clusterCentroids[i].first; 

    }

    // sort cluster centroids in decreasing order by number of points in cluster.	
  
	
    // Change cluster count to positive numbers.
    for (int i=0; i<numClusters; i++) {
    clusterCentroids[i].first = -clusterCentroids[i].first;
    //cout << clusterCentroids[i].x << " " << clusterCentroids[i].y << " " 
    //	<< clusterCentroids[i].w << " " << clusterCentroids[i].h << " " 
    //	<< clusterCentroids[i].pr << " " << clusterCentroids[i].n << endl;
    }
  */
  return clusterCentroids;
}


vector<pair<int,svlObject2d> > CallButtonEnhancer::getBestCandidates(vector<pair<int,svlObject2d> > &clusters) {

  // first element should contain upper button and second should contain lower button
  vector<pair<int,svlObject2d> > candidates;
  if(clusters.size() <1){
    return candidates;
  }
  if (clusters.size() > 1) {
    cout << "Checking for two buttons....." << endl;
    double w = (clusters[0].second.w + clusters[1].second.w)/2;
    double h = (clusters[0].second.h + clusters[1].second.h)/2;
    // Check if there are two call buttons -- assume they are vertically aligned and "close" together
    if (fabs(clusters[0].second.x-clusters[1].second.x) < w/3 &&
	(fabs(clusters[0].second.y-clusters[1].second.y)-h) < h) {	// might have to adjust this (button proximity)
      if (fabs(clusters[0].second.y-clusters[1].second.y) > h/2) {
	cout << "Two buttons found." << endl;
	if ((clusters[0].second.y-clusters[1].second.y) > 0) {
	  candidates.push_back(clusters[1]);
	  candidates.push_back(clusters[0]);
	} else if ((clusters[0].second.y-clusters[1].second.y) < 0) {
	  candidates.push_back(clusters[0]);
	  candidates.push_back(clusters[1]);
	}
      } else {
	// only one button found
	candidates.push_back(clusters[0]);
      } 
    } else {
      // only one button found
      candidates.push_back(clusters[0]);
    }
  } else {
    // only one button found
    candidates.push_back(clusters[0]);
  }

  return candidates;
}


//*****************************PRIVATE METHODS*****************************//

