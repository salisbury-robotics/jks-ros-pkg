#ifndef CALLBUTTONENHANCER_H_
#define CALLBUTTONENHANCER_H_

#include "../definitions.h"
#include "svlKMeansT.h"
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>


class CallButtonEnhancer
{
 public:
  CallButtonEnhancer();
  ~CallButtonEnhancer();
	
  // Process and return final list of elevator call buttons in image.
  svlObject2dFrame process(IplImage* image, svlObject2dFrame &objects, int maxWidth);

 private:
  static const int maxNumOfButtons = 2;
  int maxObjWidth;
  vector<pair<int,svlObject2d> > clusterV;		
  vector<pair<int,svlObject2d> > clusterButtonCenters(vector<pair<int,svlObject2d> >& detect_centroids, svlObject2dFrame& objects);
  svlObject2dFrame pruneList(svlObject2dFrame &objects);
  vector<pair<int,svlObject2d> > clusterCandidateObjects(svlObject2dFrame &objects);
  vector<pair<int,svlObject2d> > getBestCandidates(vector<pair<int,svlObject2d> > &clusters);
		
};

class sortByFirstComp
{
 public:
  template <typename NUM, typename T>
    bool operator()(const pair<NUM, T> &l, const pair<NUM, T> &r)
  {
    return l.first > r.first;
  }
};

//Given a vector of pairs where the first element of each pair is a number, sorts the vector into ascending order of those numbers)
//(So instead of writing lots of comparison functions for different ways you
//want to sort, you can just dump the objects into a list with an associated
//score and change the score when you want to sort in a different way)
template <typename NUM, typename T>
void sortByFirst(vector<pair<NUM, T> > & v)
{
  sortByFirstComp temp;
  sort(v.begin(),v.end(),temp);
}

#endif /*CALLBUTTONENHANCER_H_*/
