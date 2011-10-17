#include "lti.h"

LTISys::LTISys(double c)
{
  a=new double[1];
  b=new double[1];
  x=new double[0];
  y=new double[0];
  order_a=0;
  order_b=0;
  a[0]=1;
  b[0]=c;
}

LTISys::LTISys(const double* tb, int ob)
{
  int ib=ob;
  while (ib>=0&&tb[ib]==0)
    ib--;
  //numerator is zero
  if (ib==-1)
  {
    order_b=0;
    b=new double[1];
    x=0;
    b[0]=0;
  }
  else
  {
    order_b=ib;
    b=new double[ib+1];
    x=new double[ib];
    for (int i=0;i<=ib;i++)
      b[i]=tb[i];
    for(int i=0;i<ib;i++)
      x[i]=0;
  }
  order_a=0;
  a=new double[1];
  y=new double[0];
  a[0]=1;
}

LTISys::LTISys(const double* tb, int ob, const double* ta, int oa, double gain)
{

  int ia=oa;
  while (ia>=0&&ta[ia]==0)
    ia--;
  //denumerator is zero
  if (ia==-1)
  {
    order_a=0;
    a=new double[1];
    y=0;
    a[0]=0;
  }
  else
  {
    order_a=ia;
    a=new double[ia+1];
    y=new double[ia];
    for (int i=0;i<=ia;i++)
      a[i]=ta[i];
    for(int i=0;i<ia;i++)
      y[i]=0;
  }
  
  int ib=ob;
  while (ib>=0&&tb[ib]==0)
    ib--;
  //numerator is zero
  if (ib==-1)
  {
    order_b=0;
    b=new double[1];
    x=0;
    b[0]=0;
  }
  else
  {
    order_b=ib;
    b=new double[ib+1];
    x=new double[ib];
    for (int i=0;i<=ib;i++)
      b[i]=tb[i];
    for(int i=0;i<ib;i++)
      x[i]=0;
  }
  for(int i=0;i<=ib;i++)
    b[i]*=gain;
}

LTISys::LTISys(const LTISys &rhs)
{
  order_a=rhs.order_a;
  order_b=rhs.order_b;
  a=new double[order_a+1];
  b=new double[order_b+1];
  x=new double[order_b];
  y=new double[order_a];
  for (int i=0;i<=order_a;i++)
    a[i]=rhs.a[i];
  for (int i=0;i<=order_b;i++)
    b[i]=rhs.b[i];
  for(int i=0;i<order_a;i++)
    y[i]=0;
  for(int i=0;i<order_b;i++)
    x[i]=0;
}

LTISys & LTISys::operator=(const LTISys &rhs)
{
  if (this==&rhs)
    return *this;
  
  order_a=rhs.order_a;
  order_b=rhs.order_b;
  delete[] a;
  delete[] b;
  delete[] x;
  delete[] y;
  a=new double[order_a+1];
  b=new double[order_b+1];
  
  if(order_a==0)
    y=0;
  else
    y=new double[order_a];
  
  if(order_b==0)
    x=0;
  else
    x=new double[order_b];
  
  for (int i=0;i<=order_a;i++)
    a[i]=rhs.a[i];
  for (int i=0;i<=order_b;i++)
    b[i]=rhs.b[i];
  for(int i=0;i<order_a;i++)
    y[i]=rhs.y[i];
  for(int i=0;i<order_b;i++)
    x[i]=rhs.x[i];
  return *this;
}

const LTISys LTISys::num() const
{
  return LTISys(b,order_b);
}

const LTISys LTISys::den() const
{
  return LTISys(a,order_a);
}

const LTISys LTISys::operator*(const LTISys &other) const
{
  int oa=order_a+other.order_a;
  int ob=order_b+other.order_b;
  double *ta=new double[oa+1];
  double *tb=new double[ob+1];
  for (int i=0;i<oa+1;i++)
  {
    ta[i]=0;
    for (int j=0;j<=i&&j<=order_a;j++)
    {
      if ((i-j)<=other.order_a)
        ta[i]+=a[j]*other.a[i-j];
    }
    //cout<<result.a[i]<<endl;
  }
  for (int i=0;i<ob+1;i++)
  {
    tb[i]=0;
    for (int j=0;j<=i&&j<=order_b;j++)
    {
      if ((i-j)<=other.order_b)
        tb[i]+=b[j]*other.b[i-j];
    }
  }
  LTISys result(tb,ob,ta,oa);
  delete[] ta;
  delete[] tb;
  return result;
}

const LTISys LTISys::inv() const
{
  return LTISys(a,order_a,b,order_b);
}

const LTISys LTISys::operator/(const LTISys &other) const
{
  LTISys result=(*this)*other.inv();
  return result;
}

const LTISys LTISys::operator+(const LTISys &other) const
{
  LTISys ra=this->den()*other.den();
  LTISys rb1=this->num()*other.den();
  LTISys rb2=this->den()*other.num();

  int oa,ob;
  oa=ra.order_b;
  ob=(rb1.order_b>rb2.order_b)?rb1.order_b:rb2.order_b;

  double *ta=new double[oa+1];
  double *tb=new double[ob+1];

  for (int i=0;i<ob+1;i++)
  {
    tb[i]=0;
    if(i<=rb1.order_b)
      tb[i]+=rb1.b[i];
    if(i<=rb2.order_b)
      tb[i]+=rb2.b[i];
  }
  for (int i=0;i<oa+1;i++)
    ta[i]=ra.b[i];

  LTISys result(tb,ob,ta,oa);
  delete[] ta;
  delete[] tb;
  return result;
}

const LTISys LTISys::operator-() const
{
  LTISys result=*this;
  for (int i=0;i<order_b;i++)
    result.b[i]=-result.b[i];
  return result;
}

const LTISys LTISys::operator-(const LTISys &other) const
{
  return (*this)+(-other);
}


ostream& operator<<(ostream &os,const LTISys &obj)
  {
    for(int i=obj.order_b;i>=0;i--)
      os<<obj.b[i]<<',';
    os<<endl;
    for(int i=obj.order_a;i>=0;i--)
      os<<obj.a[i]<<',';
    return os;
  }
const LTISys LTISys::tustin(double ts) const
{
  double ssa[]={ts/2,ts/2};
  double ssb[]={-1,1};
  LTISys s(ssb,1,ssa,1);
  LTISys sa(ssa,1);
  LTISys sb(ssb,1);

  LTISys ra=LTISys(a[order_a]);
  for (int i=order_a-1;i>=0;i--)
    ra=s*ra+LTISys(a[i]);
  LTISys rb=LTISys(b[order_b]);
  for (int i=order_b-1;i>=0;i--)
    rb=s*rb+LTISys(b[i]);

  int n=order_a-order_b;
  if (n>0)
    for (int i=0;i<n;i++)
      rb=rb*sa;
  else
    for (int i=0;i<-n;i++)
      ra=ra*sa;
  LTISys r=rb.num()/ra.num();
  //LTISys *dbgr=&r;
  double c=r.a[r.order_a];
  for(int i=0;i<=r.order_a;i++)
    r.a[i]/=c;
  for(int i=0;i<=r.order_b;i++)
    r.b[i]/=c;
  return r;
}

void LTISys::initialize(double x0)
{
  for(int i=0;i<order_b;i++)
    x[i]=x0;
  double sa=0;
  double sb=0;
  for(int i=0;i<=order_a;i++)
    sa+=a[i];
  for(int i=0;i<=order_b;i++)
    sb+=b[i];
  double dc_gain=sb/sa;
  for(int i=0;i<order_a;i++)
    y[i]=x0*dc_gain;
}

double LTISys::filter(double xn)
{
  double yn=xn*b[order_b];
  //cout<<xn<<','<<b[1]<<',';
  for(int i=order_b-1;i>=0;i--)
    yn+=x[order_b-1-i]*b[i];
  //cout<<x[0]<<','<<b[0]<<',';
  for(int i=order_a-1;i>=0;i--)
    yn-=y[order_b-1-i]*a[i];
  yn/=a[order_b];
  //cout<<y[0]<<','<<a[0]<<','<<yn<<endl;
  for(int i=order_b-1;i>0;i--)
    x[i]=x[i-1];
  x[0]=xn;
  for(int i=order_a-1;i>0;i--)
    y[i]=y[i-1];
  y[0]=yn;
  
  return yn;
}