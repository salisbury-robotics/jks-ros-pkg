#ifndef WILHELMSENPROJECTION_H
#define WILHELMSENPROJECTION_H

#include <vector>

// Select the matrix library to use here. Eigen is used if USE_EIGEN is defined.
// Otherwise, CML is used. Eigen's linear equation solver has much better
// stability than matrix inversion using CML.
//#define USE_EIGEN

// includes for matrix library
#ifdef USE_EIGEN
#include <Eigen/Core>
#else
#include <cml/mathlib/typedef.h>
#include <algorithm>
#endif

// This class is an implementation of the convex cone projection algorithm
// described by Don R. Wilhelmsen in "A nearest point algorithm for convex
// polyhedral cones and applications to positive linear approximation,"
// Mathematics of Computation 30(133), 1976.

template <class T>          // class is templated for double and long double types
class WilhelmsenProjection
{
    // a debugging variable that holds the set of active constraints
    static std::vector<int> constraints;

public:
    // This implementation works (only) on 6-dimensional vectors
#ifdef USE_EIGEN
    typedef Eigen::Matrix<T, 6, 1>                              vector6;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1>                 vectorn;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>    matrixn;
    template <class V> static T dot(const V &a, const V &b)     { return a.dot(b); }
    template <class V> static T norm(const V &v)                { return v.norm(); }
    template <class V> static T norm2(const V &v)               { return v.squaredNorm(); }
    template <class V> static T smallest(const V &v)            { return v.minCoeff(); }
    static vector6 zero6()                                      { return vector6::Zero(); }
#else
    typedef cml::vector< T, cml::fixed<6> >                     vector6;
    typedef cml::vector< T, cml::dynamic<> >                    vectorn;
    typedef cml::matrix< T, cml::dynamic<> >                    matrixn;
    template <class V> static T dot(const V &a, const V &b)     { return cml::dot(a, b); }
    template <class V> static T norm(const V &v)                { return v.length(); }
    template <class V> static T norm2(const V &v)               { return v.length_squared(); }
    template <class V> static T smallest(const V &v)            { return *std::min_element(&v[0], &v[v.size()]); }
    static vector6 zero6()                                      { vector6 v; return v.zero(); }
#endif

    // epsilon values to (hopefully) robustify zero-checking
    static const T eps, eps2;

    // Computes the Euclidean projection of a point q onto a subspace S, the
    // span of the basis vectors given. Returns lambda, the set of barycentric
    // coordinates for the projected point, in terms of the basis of S.
    static vectorn projectSubspace(const std::vector<vector6> &S,
                                   const vector6 &q);

    // Computes the Euclidean projection of a point q onto a convex cone K,
    // given by a set of generating vectors. Returns a point p in K that is
    // closest to q in terms of Euclidean distance.
    static vector6 projectCone(const std::vector<vector6> &K,
                               const vector6 &q);

    // Conditions a set of generating vectors by normalizing them and removing
    // any near-parallel vectors.
    static void conditionGenerators(std::vector<vector6> &K);

    // prints the active constraint set, for debugging
    static void printConstraints();
};

#endif // WILHELMSENPROJECTION_H
