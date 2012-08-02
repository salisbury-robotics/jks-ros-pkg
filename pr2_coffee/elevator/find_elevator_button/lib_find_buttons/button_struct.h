#ifndef BUTTONSTRUCT_H
#define BUTTONSTRUCT_H

#include <stdlib.h>
#include <string>
#include <vector>
using namespace std;

	struct button_struct
	{
	  double x;
	  double y;	
	  bool isButton;
	  int labelInd;
	  string button_image_file;
	  string label_image_file;
	  vector<string> button_binarizations;
	  vector<string> label_binarizations;
	  string label;
	};

	struct grid_param_struct {
		double gx;
		double gy;
 	 	double dx;
  		double dy;
  		int nrows;
  		int ncols;
  		int gridNum;
  		int nDetections;
	};

#endif
