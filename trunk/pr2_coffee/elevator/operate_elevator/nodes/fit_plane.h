#pragma once

#include <iostream>
#include <vector>
#include <time.h>

#include "opencv/cv.h"
#include "opencv/cxcore.h"

using namespace std;

class FitPlane {
public:
	FitPlane();
	~FitPlane();

	void fit_plane(const vector<vector<double> > &points, vector<double> &normal, 
                 vector<double> &centroid);
};

FitPlane::FitPlane() {
}

FitPlane::~FitPlane() {
}

void FitPlane::fit_plane(const vector<vector<double> > &points, 
                         vector<double> &normal, vector<double> &centroid)
{
	vector<double> temp (3,0);
	normal = temp;
	centroid = temp;

	size_t numPts = points.size();

	// write vector of points into CvMat X, Y, Z
	CvMat* X = cvCreateMat(numPts, 1, CV_32FC1);
	CvMat* Y = cvCreateMat(numPts, 1, CV_32FC1);
	CvMat* Z = cvCreateMat(numPts, 1, CV_32FC1);
	for (size_t i=0; i<numPts; i++) {
		//X->data.fl[i] = points[i][0];
		//Y->data.fl[i] = points[i][1];
		//Z->data.fl[i] = points[i][2];
		CV_MAT_ELEM(*X, float, i, 0) = points[i][0];
		CV_MAT_ELEM(*Y, float, i, 0) = points[i][1];
		CV_MAT_ELEM(*Z, float, i, 0) = points[i][2];
	}

	// allocate temporary matrices
   CvMat* C = cvCreateMat(3,3,CV_32FC1); // covariance matrix
   CvMat* U = cvCreateMat(3,3,CV_32FC1); // left orthogonal matrix
   CvMat* S = cvCreateMat(3,3,CV_32FC1); // singular values
   assert((C != NULL) && (U != NULL) && (S != NULL));

   float muX, muY, muZ;
   float numPoints = numPts;

   // estimate point normal by SVD
   cvZero(C);
   muX = 0.0; muY = 0.0; muZ = 0.0;
   for (size_t i = 0; i < numPts; i++) {

       float ptX = CV_MAT_ELEM(*X, float, i, 0);
       float ptY = CV_MAT_ELEM(*Y, float, i, 0);
       float ptZ = CV_MAT_ELEM(*Z, float, i, 0);

       muX += ptX; muY += ptY; muZ += ptZ;
       CV_MAT_ELEM(*C, float, 0, 0) += ptX * ptX;
       CV_MAT_ELEM(*C, float, 0, 1) += ptX * ptY;
       CV_MAT_ELEM(*C, float, 0, 2) += ptX * ptZ;
       CV_MAT_ELEM(*C, float, 1, 1) += ptY * ptY;
       CV_MAT_ELEM(*C, float, 1, 2) += ptY * ptZ;
       CV_MAT_ELEM(*C, float, 2, 2) += ptZ * ptZ;
   }

   CV_MAT_ELEM(*C, float, 0, 0) -= (muX * muX) / numPoints;
   CV_MAT_ELEM(*C, float, 0, 1) -= (muX * muY) / numPoints;
   CV_MAT_ELEM(*C, float, 0, 2) -= (muX * muZ) / numPoints;
   CV_MAT_ELEM(*C, float, 1, 1) -= (muY * muY) / numPoints;
   CV_MAT_ELEM(*C, float, 1, 2) -= (muY * muZ) / numPoints;
   CV_MAT_ELEM(*C, float, 2, 2) -= (muZ * muZ) / numPoints;
   CV_MAT_ELEM(*C, float, 1, 0) = CV_MAT_ELEM(*C, float, 0, 1);
   CV_MAT_ELEM(*C, float, 2, 0) = CV_MAT_ELEM(*C, float, 0, 2);
   CV_MAT_ELEM(*C, float, 2, 1) = CV_MAT_ELEM(*C, float, 2, 1);

   cvSVD(C, S, U, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);
   float residual = CV_MAT_ELEM(*S, float, 2, 2);

   // normal
   float dp = CV_MAT_ELEM(*U, float, 2, 0) * muX +
       CV_MAT_ELEM(*U, float, 2, 1) * muY +
       CV_MAT_ELEM(*U, float, 2, 2) * muZ;
   if (dp <= 0.0) {
       normal[0] = CV_MAT_ELEM(*U, float, 2, 0);
       normal[1] = CV_MAT_ELEM(*U, float, 2, 1);
       normal[2] = CV_MAT_ELEM(*U, float, 2, 2);
   } else {
       normal[0] = -CV_MAT_ELEM(*U, float, 2, 0);
       normal[1] = -CV_MAT_ELEM(*U, float, 2, 1);
       normal[2] = -CV_MAT_ELEM(*U, float, 2, 2);
   }

   // centroid
	centroid[0] = muX/numPoints;
	centroid[1] = muY/numPoints;
	centroid[2] = muZ/numPoints;

   // release matrices
   cvReleaseMat(&S);
   cvReleaseMat(&U);
   cvReleaseMat(&C);
	cvReleaseMat(&X);
   cvReleaseMat(&Y);
   cvReleaseMat(&Z);

	return;
}
