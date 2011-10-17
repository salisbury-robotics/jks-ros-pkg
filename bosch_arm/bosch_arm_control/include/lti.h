#ifndef LTISYS_H
#define LTISYS_H
#include<iostream>
using namespace std;
class LTISys
{
public:
  double *a;
  double *b;
  double *x;
  double *y;
  int order_a;
  int order_b;
  LTISys()
  {
    a=NULL;
    b=NULL;
    order_a=0;
    order_b=0;
  }
 ~LTISys()
 {
    if (a!=NULL)
      delete[] a;
    if (b!=NULL)
      delete[] b;
    delete[] x;
    delete[] y;
 }

  LTISys(double c);
  LTISys(const LTISys &rhs);
  LTISys(const double* tb, int ob);
  LTISys(const double* ta, int oa, const double* tb, int ob,double gain=1);
  LTISys & operator=(const LTISys &rhs);
  const LTISys num() const;
  const LTISys den() const;
  const LTISys operator*(const LTISys &other) const;
  const LTISys inv() const;
  const LTISys operator/(const LTISys &other) const;
  const LTISys operator+(const LTISys &other) const;
  const LTISys operator-() const;
  const LTISys operator-(const LTISys &other) const;
  const LTISys tustin(double ts) const;
  void initialize(double x0);
  double filter(double xn);
  friend ostream& operator<<(ostream &os,const LTISys &obj);
};
#endif