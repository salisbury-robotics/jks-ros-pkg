#ifndef CONTROL_H
#define CONTROL_H

#include <vector>
using namespace std;
vector<double> get_Joint_Pos(void);
vector<double> get_Joint_Pos_Actual(void);
void set_Joint_Pos(vector<double>);
vector<double> get_Joint_Vel(void);


#endif // CONTROL_H
