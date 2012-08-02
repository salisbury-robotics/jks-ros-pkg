#include <vector>
#include <algorithm>
#include <ctime>
//#include <unistd.h>

#include "kdtree.h"

bool on_x(const cVertex *a, const cVertex *b){    return a->m_localPos.x < b->m_localPos.x;  }
bool on_y(const cVertex *a, const cVertex *b){    return a->m_localPos.y < b->m_localPos.y;  }
bool on_z(const cVertex *a, const cVertex *b){    return a->m_localPos.z < b->m_localPos.z;  }

double vDist2(const cVertex *a, const cVertex *b)
{
    double dx = a->m_localPos.x-b->m_localPos.x, dy = a->m_localPos.y-b->m_localPos.y, dz = a->m_localPos.z-b->m_localPos.z;
    return dx*dx + dy*dy + dz*dz;
}

double pDist2(const cVector3d &a, const cVector3d &b)
{
    double dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
    return dx*dx + dy*dy + dz*dz;
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

// Bounding box for a set of points

    // computes bounding box from a bunch of points
    void bbox::compute(const vector<cVertex*> &vtx)
    {
        for( unsigned int i =0; i < vtx.size(); i++)
        {
            const cVector3d &v = vtx[i]->m_localPos;
            x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
            y0 = std::min(y0, v.y); y1 = std::max(y1, v.y);
            z0 = std::min(z0, v.z); z1 = std::max(z1, v.z);
        }
        dx = x1 - x0; 
        dy = y1 - y0;
        dz = z1 - z0;
    }

    // squared distance between a point and this box, 0 if inside.
    double bbox::distance(const cVertex *v)
    {
        const cVector3d &p = v->m_localPos;
        if(p.x < x0){
            if(p.y < y0)
            {
                if( p.z < z0 )      return pDist2( cVector3d(x0, y0, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x0, y0, z1), p);
                else                return pDist2( cVector3d(x0, y0, p.z), p);
            }
            else if(p.y > y1){
                if( p.z < z0 )      return pDist2( cVector3d(x0, y1, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x0, y1, z1), p);
                else                return pDist2( cVector3d(x0, y1, p.z), p);
            }
            else {
                if( p.z < z0 )      return pDist2( cVector3d(x0, p.y, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x0, p.y, z1), p);
                else                return pDist2( cVector3d(x0, p.y, p.z), p);
            }
        }
        else if(p.x > x1){
            if(p.y < y0)
            {
                if( p.z < z0 )      return pDist2( cVector3d(x1, y0, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x1, y0, z1), p);
                else                return pDist2( cVector3d(x1, y0, p.z), p);
            }
            else if(p.y > y1){
                if( p.z < z0 )      return pDist2( cVector3d(x1, y1, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x1, y1, z1), p);
                else                return pDist2( cVector3d(x1, y1, p.z), p);
            }
            else {
                if( p.z < z0 )      return pDist2( cVector3d(x1, p.y, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(x1, p.y, z1), p);
                else                return pDist2( cVector3d(x1, p.y, p.z), p);
            }
        }
        else{ 
            if(p.y < y0)
            {
                if( p.z < z0 )      return pDist2( cVector3d(p.x, y0, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(p.x, y0, z1), p);
                else                return pDist2( cVector3d(p.x, y0, p.z), p);
            }
            else if(p.y > y1){
                if( p.z < z0 )      return pDist2( cVector3d(p.x, y1, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(p.x, y1, z1), p);
                else                return pDist2( cVector3d(p.x, y1, p.z), p);
            }
            else {
                if( p.z < z0 )      return pDist2( cVector3d(p.x, p.y, z0), p);
                else if(p.z > z1 )  return pDist2( cVector3d(p.x, p.y, z1), p);
                else                return 0;
            }
        }
    }

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    // intersect a point with this node (returns squared distance)
    double kdnode::intersect(const cVertex *v){
        return bound.distance(v);
    }

    // recursively builds a kd-tree from a given cloud of vertices
    void kdnode::construct(vector<cVertex*> &vec){
        // compute a bounding box for points at this node
        bound.compute(vec);

        // if we're down to one point then we're a leaf node
        if( vec.size() == 1 ){
            leaf = true;
            vtx = vec[0];
        }
        else {
            if( bound.dx >= bound.dy && bound.dx >= bound.dz)
                std::sort(vec.begin(), vec.end(), on_x);
            else if(bound.dy >= bound.dx && bound.dy >= bound.dz)
                std::sort(vec.begin(), vec.end(), on_y);
            else
                std::sort(vec.begin(), vec.end(), on_z);

            int half = vec.size()/2;
            vector<cVertex*> vleft(vec.begin(), vec.begin() + half);
            vector<cVertex*> vright(vec.begin()+half, vec.end());
            first = new kdnode();
            first->construct(vleft);
            second = new kdnode();
            second->construct(vright);
        }
    }

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

//simple kdtree class to hold the tree and handle queries
   // constructs a kdtree from points
    kdtree::kdtree(vector<cVertex> &vec)
    {
        vector<cVertex*> v;
        v.reserve(vec.size());
        for(unsigned int i = 0; i < vec.size(); i++)
        {
            v.push_back(&(vec[i]));
        }
        
        root = new kdnode();
        printf("Constructing a kdtree with %d points! \n", (unsigned int)v.size());
        root->construct(v);
    }
    //kdtree::~kdtree() {delete root;}


    //recursively get neighbors
    void kdtree::getNeighbors(kdnode *node, const cVertex &vx, const double &radius2, vector<cVertex*> &neighbors)
    {
        if(node->leaf){
            double dist = vDist2( &vx, node->vtx);
            if(dist <= radius2)
                neighbors.push_back(node->vtx);
            return;
        }

        double bfirst = node->first->intersect(&vx);
        double bsecond = node->second->intersect(&vx);

        if( bfirst < radius2 )
            getNeighbors(node->first, vx, radius2, neighbors);
        
        if( bsecond < radius2 )
            getNeighbors(node->second, vx, radius2, neighbors);
    }

    // get neighbors
    void kdtree::neighbors(const cVertex &vx, const double &radius, vector<cVertex*> &neighbors){
        neighbors.clear();
        neighbors.reserve(10);
        getNeighbors(root, vx, radius*radius, neighbors);
    }

        //recursively get neighbors
    void kdtree::getkNN(kdnode *node, const cVertex &vx, const int &k, vector<cVertex*> &neighbors)
    {
        //if(node->leaf){
        //    double dist = vDist2( vx, node->vtx);
        //    if(dist <= radius2)
        //    {
        //        neighbors.push_back(node->vtx);
        //        
        //    }
        //    return;
        //}

        //double bfirst = node->first->intersect(vx);
        //double bsecond = node->second->intersect(vx);

        //if( bfirst < radius2 )
        //    getNeighbors(node->first, vx, radius2, neighbors);
        //
        //if( bsecond < radius2 )
        //    getNeighbors(node->second, vx, radius2, neighbors);
    }

    // get neighbors
    void kdtree::kNN(const cVertex &vx, const int &k, vector<cVertex*> &neighbors){
        neighbors.clear();
        neighbors.reserve(10);
        getNeighbors(root, vx, k, neighbors);
    }

    // recursive search method returns squared distance to nearest point
    double kdtree::search(kdnode *node, const cVertex &vx)
    {
        if(node->leaf){
            double distance = vDist2(&vx, node->vtx);
            if( distance < 0.00001 ) return 1000.0;
            return distance;
        }

        double bfirst = node->first->intersect(&vx);
        double bsecond = node->second->intersect(&vx);

        //choose side with closest bounding box to search first
        if(bfirst < bsecond){
            double best = search(node->first, vx);
            if( bsecond < best )
                best = std::min(best, search(node->second, vx));
            return best;
        }
        else {
            double best = search(node->second, vx);
            if( bfirst < best )
                best = std::min(best, search(node->first, vx));
            return best;
        }
    }

    // distance to nearest
    double kdtree::nearest(const cVertex &vx){ 
        return sqrt(search(root, vx));
    }
