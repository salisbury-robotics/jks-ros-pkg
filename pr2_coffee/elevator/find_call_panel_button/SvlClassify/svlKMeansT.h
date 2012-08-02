#ifndef SVLKMEANST_H
#define SVLKMEANST_H
/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2009, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlKMeansT.h
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
**
** DESCRIPTION:
**   Templated simple k-means class. Works for any datatype T that has a
**   norm2() as well as binary minus (as in "(a-b).norm2"), so svlPoint2d,
**   svlPoint3d, svlPointNd.
*****************************************************************************/

#include <iostream>
#include <vector>
#include <time.h>

using namespace std;

template<class V>
class svlKMeansT {
public:
	svlKMeansT();
	virtual ~svlKMeansT();

	void do_kmeans(const vector<V> &P, vector<V> &cent, vector<int> *ix, int k, int max_iter, int num_changes = 0);
};

template<class V>
svlKMeansT<V>::svlKMeansT() {
}

template<class V>
svlKMeansT<V>::~svlKMeansT() {
}

template<class V>
void svlKMeansT<V>::do_kmeans(const vector<V> &P, vector<V> &cent, vector<int> *ix, int k, int max_iter, int num_changes) {
	// Points P and centroids cent. If cent already contains values, these are kept and used
	// as initialization centroids. Any missing values (up to k) are drawn as random points
	// from P.
	// Pass max_iter = -1 for unlimited iterations (until threshold convergence criterion is met).
	// num_changes loops until <= num_changes points switch assignments in a loop. Default 0.
	// Input num_changes = -1 to nullify this condition.
	srand((unsigned int)time(NULL));

	if ( cent.size() < (unsigned int)k ) {
		// Initialize missing centroids with random data points.
		for ( unsigned int i=cent.size(); i<(unsigned int)k; ++i ) {
			cent.push_back( V( P[rand()%P.size()] ) );
		}
	}

	vector<typename vector<V>::iterator> asgs(P.size(),cent.begin());
	typename vector<typename vector<V>::iterator>::iterator asg_it;
	int *counts = new int[k];
	double d, d1;

	int n_changes = num_changes+1;
	for ( int iter=0; ( max_iter < 0 || iter<max_iter )
				   && ( num_changes < 0 || n_changes > num_changes ); ++iter ) {
		// Calculate new minimum distances.
		asg_it = asgs.begin();
		n_changes = 0;
		for ( typename vector<V>::const_iterator p=P.begin(); p!=P.end(); ++p ) {
			// Get current assignment point-to-centroid distance.
			d = (*p - **asg_it).norm2();
			for ( typename vector<V>::iterator c=cent.begin(); c!=cent.end(); ++c ) {
				if ( c == *asg_it ) continue;
				if ( (d1=(*p-*c).norm2()) < d ) {
					d = d1;
					++n_changes;
					*asg_it = c; // Save new centroid for p.
				}
			}
			++asg_it;
		}
		
		// Calculate new centroids.
		for ( typename vector<V>::iterator c=cent.begin(); c!=cent.end(); ++c )
			*c = 0.0; // Zeralize centroids.
		memset(counts,0,sizeof(int)*cent.size()); // Zeralize counters.
		asg_it = asgs.begin();
		for ( typename vector<V>::const_iterator p=P.begin(); p!=P.end(); ++p ) {
			**asg_it += *p; // Add in all points assigned to cluster.
			++counts[ *asg_it - cent.begin() ];
			++asg_it;
		}
		for ( unsigned int i=0; i<cent.size(); ++i )
			if ( counts[i] > 0 ) // No div by 0.
				cent[i] /= (double)counts[i];
	}

	// Convert cluster assignments to assignment indices to fill ix.
	if ( ix != NULL ) {
		ix->resize(P.size());
		vector<int>::iterator it = ix->begin();
		for ( asg_it=asgs.begin(); asg_it!=asgs.end(); ++asg_it ) {
			*it = *asg_it - cent.begin();
			++it;
		}
	}
	delete[] counts;
}

#endif
