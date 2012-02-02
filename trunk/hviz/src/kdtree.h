#ifndef _KDTREE_H
#define _KDTREE_H

#include "chai3d.h"

#include <vector>
#include <algorithm>
#include <ctime>
#include <queue>
//#include <unistd.h>


// Bounding box for a set of points
struct bbox{
    double x0, x1, y0, y1, z0, z1;
    double dx, dy, dz;
    bbox() : x0(1000), x1(-1000), y0(1000), y1(-1000), z0(1000), z1(-1000), dx(0), dy(0), dz(0) {}

    // computes bounding box from a bunch of points
    void compute(const vector<cVertex*> &vtx);

    // squared distance between a point and this box, 0 if inside.
    double distance(const cVertex *v);
};

struct kdnode {
    bool leaf;      // true if this is a leaf node (has one point)
    cVertex *vtx;    // the vertex on this leaf
    bbox bound;     // bounding box for set of points in children

    kdnode *first, *second;  // two children of this node.
    
    kdnode() : leaf(false), first(0), second(0) {}
    ~kdnode() { if(first) delete first; if(second) delete second; }

    // intersect a point with this node (returns squared distance)
    double intersect(const cVertex *v);

    // recursively builds a kd-tree from a given cloud of vertices
    void construct(vector<cVertex*> &vec);
};

//simple kdtree class to hold the tree and handle queries
struct kdtree{
    kdnode *root;
    std::priority_queue<double> kNN_pq;


    // constructs a kdtree from points
    kdtree(vector<cVertex> &vec);
    ~kdtree() {delete root;}

    //recursively get neighbors
    void getNeighbors(kdnode *node, const cVertex &vx, const double &radius2, vector<cVertex*> &neighbors);

    // get neighbors
    void neighbors(const cVertex &vx, const double &radius, vector<cVertex*> &neighbors);

    //recursively get neighbors
    void getkNN(kdnode *node, const cVertex &vx, const int &k, vector<cVertex*> &neighbors);

    // get neighbors
    void kNN(const cVertex &vx, const int &k, vector<cVertex*> &neighbors);

    // recursive search method returns squared distance to nearest point
    double search(kdnode *node, const cVertex &vx);

    //squared distance to nearest
    double nearest(const cVertex &vx);


};

#endif
