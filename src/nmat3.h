
// Lightweight templated matrix3 library
// Dec 2019
// Ivan DeWolf
//
// this was written with the goal of brevity and ease of use.
// all code is in this .h file
// there is no error checking, 
// this was not optimized for execution speed or mimimizing RAM usage.
// this was not written to handle every possible requirement.
// this is not documented, or written for ease of comprehension.

#ifndef __nmat3_h__
#define __nmat3_h__
#include <iostream>
#include <cmath>
#include "nvec3.h"
#include "nquat.h"
namespace ntv {
template <typename T>
class nmat3 // nano matrix 3, 3x3 matrices for geometric transforms
{
 public:
  // ------------------------------------------------- constructors
  nmat3(){this->identity();}
  nmat3(nmat3& m){for(int i=0; i<9; ++i)n[i]=m[i];}
  nmat3(T zz,T zo,T zt,T oz,T oo,T ot,T tz,T to,T tt) {n(0,0)=zz;n(0,1)=zo;n(0,2)=zt;n(1,0)=oz;n(1,1)=oo;n(1,2)=ot;n(2,0)=tz;n(2,1)=to;n(2,2)=tt;}
  // ----------------------------------------------- element access
  T& operator [] (int i){return n[i];}
  const T& operator[](int i)const{return n[i];}
  T& operator() (int r, int c){return n[(r*3) + c];}
  const T& operator()(int r, int c)const{return n[(r * 3) + c];}
  T get(int r, int c)const{return n[(r*3) + c];}// get() is const
  void set(const nvec3<T>& a, const nvec3<T>& b, const nvec3<T>& c){
    n[0]=a[0]; n[1]=b[0]; n[2]=c[0];
    n[3]=a[1]; n[4]=b[1]; n[5]=c[1];
    n[6]=a[2]; n[7]=b[2]; n[8]=c[2];}
  void setRow(int i,const nvec3<T>& v){n[(3*i)+0]=v[0]; n[(3*i)+1]=v[1]; n[(3*i)+2]=v[2];}
  nvec3<T> getRow(int i)const{return nvec3<T>(n[(3*i)+0], n[(3*i)+1], n[(3*i)+2]);}
  // -------------------------------------------- matrix3 scalar operators
  nmat3& operator *= (const T& s){for(int i=0; i<9; ++i)n[i]*=s;return *this;}
  nmat3& operator /= (const T& s){for(int i=0; i<9; ++i)n[i]/=s;return *this;}
  friend nmat3 operator*(const nmat3& m,T s){nmat3 out;for(int i=0; i<9; ++i)out[i]=m[i]*s;return out;}
  friend nmat3 operator/(const nmat3& m,T s){nmat3 out;for(int i=0; i<9; ++i)out[i]=m[i]/s;return out;}
  // ------------------------------------------- matrix3 vector operators
   nvec3<T> operator*(const nvec3<T>& v)const{nvec3<T> rowX = getRow(0) * v[0]; nvec3<T> rowY = getRow(1) * v[1]; nvec3<T> rowZ = getRow(2) * v[2]; return rowX + rowY + rowZ;}
  // ----------------------------------------- matrix3 quaternion operators
  void fromNquat(nquat<T>& q){T x2 = q[1] * q[1], y2 = q[2] * q[2], z2 = q[3] * q[3];T sx = q[0] * q[1], sy = q[0] * q[2], sz = q[0] * q[3];T xz = q[1] * q[3], yz = q[2] * q[3], xy = q[1] * q[2];
    //nmat3<T> m3;
    //m3[0] = 3;
    //(2,2)=4;
    n[0]=7;
    // *this = nmat3(1-2*(y2+z2),2*(xy+sz),2*(xz-sy),2*(xy-sz),1-2*(x2+z2),2*(sx+yz),2*(sy+xz),2*(yz-sx),1-2*(x2+y2));
    /*n[0]=1-2*(y2+z2); n[1]=2*(xy+sz); n[2]=2*(xz-sy);
    n[3]=2*(xy-sz); n[4]=1-2*(x2+z2); n[5]=2*(sx+yz);
    n[6]=2*(sy+xz); n[6]=3;n[7]=2*(yz-sx);n[8]=1-2*(x2+y2);*/}
  nquat<T> toNquat() const {const T trace = get(0, 0) + get(1, 1) + get(2, 2);
    if(trace>0){T s=sqrt(trace + 1)*2;T oneOverS=1/s;return nquat(0.25*s,(n[5]-n[7])*oneOverS,(n[6]-n[2])*oneOverS,(n[1]-n[3])*oneOverS);}
    if(n[0]>n[4]&&n[0]>n[8]){T s=sqrt(n[0]-n[4]-n[8]+1)*2;T oneOverS=1/s;return nquat((n[5]-n[7])*oneOverS,0.25*s,(n[3]+n[1])*oneOverS,(n[6]+n[2])*oneOverS);}
    if(n[4]>n[8]){T s=sqrt(n[4]-n[0]-n[8]+1)*2;T oneOverS=1/s;return nquat((n[6]-n[2])*oneOverS,(n[3]+n[1])*oneOverS,0.25*s,(n[5]+n[7])*oneOverS);}
    T s=sqrt(n[8]-n[0]-n[4]+1)*2;T oneOverS=1/s; return nquat((n[1]-n[3])*oneOverS,(n[6]+n[2])*oneOverS,(n[5]+n[7])*oneOverS,0.25*s);}
  // ----------------------------------------- matrix3 matrix3 operators
  nmat3& operator += (const nmat3& m){for(int i=0; i<9; ++i)n[i]+=m[i];return *this;}
  nmat3& operator -= (const nmat3& m){for(int i=0; i<9; ++i)n[i]-=m[i];return *this;}
  nmat3 operator+(const nmat3& m) const {nmat3 out;for(int i=0; i<9; ++i)out[i]=n[i]+m[i];return out;}
  nmat3 operator-(const nmat3& m) const {nmat3 out;for(int i=0; i<9; ++i)out[i]=n[i]-m[i];return out;}
  friend nmat3 operator*(const nmat3& mA, const nmat3& mB){nmat3 out;for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)out(i,j)=mA.get(i,k)*mB.get(k,j);return out;}
  // -------------------------------------------- equality functions
  bool operator == (const nmat3& m)const{for(int i=0; i<9; ++i)if(n[i]!=m[i])return false;return true;}
  bool operator != (const nmat3& m)const{return !(*this==m);}
  // ---------------------------------------------- mutators
  void identity(){n[0]=1;n[1]=0;n[2]=0;n[3]=0;n[4]=1;n[5]=0;n[6]=0;n[7]=0;n[8]=1;}
  void transpose(){nvec3<T> A,B,C;A.set(n[0],n[3],n[6]);B.set(n[1],n[4],n[7]);C.set(n[2],n[5],n[8]);set(A,B,C);}
  // --------------------------------------------- utility functions
  friend std::ostream& operator << (std::ostream& s, const nmat3<T>& m){
    s<<"\n("<<m.n[0]<<", "<<m.n[1]<<", "<<m.n[2]<<")";
    s<<"\n("<<m.n[3]<<", "<<m.n[4]<<", "<<m.n[5]<<")";
    s<<"\n("<<m.n[6]<<", "<<m.n[7]<<", "<<m.n[8]<<")";
    return s;}
  void rotate(const nvec3<T>& Axis, T angle) { // assumes your axis is normalized
    T c = cos(angle); T s = sin(angle); T t = 1.0 - c; nvec3<T> A,B,C;
    A.set(t*Axis[0]*Axis[0]+c,t*Axis[0]*Axis[1]+s*Axis[2],t*Axis[0]*Axis[2]-s*Axis[1]);
    B.set(t*Axis[0]*Axis[1]-s*Axis[2],t*Axis[1]*Axis[1]+c,t*Axis[1]*Axis[2]+s*Axis[0]);
    C.set(t*Axis[0]*Axis[2]+s*Axis[1],t*Axis[1]*Axis[2]-s*Axis[0],t*Axis[2]*Axis[2]+c);
    set(A,B,C);}
  T determinant2d(int a, int b, int c, int d)const{return n[a] * n[d] - n[c] * n[b];}
  T determinant()const{T ab=determinant2d(3,6,4,7);T ac=determinant2d(3,6,5,8);T bc=determinant2d(4,7,5,8);
    T a=n[0]*bc;T b=n[1]*ac;T c=n[2]*ab;return a-b+c;}
  nmat3<T> inverse()const{T det=determinant();if(det==0)return nmat3<T>();T invDet=1/det;
    nmat3<T> out;out.n[0]=invDet*(n[4]*n[8]-n[5]*n[7]);out.n[1]=invDet*(n[2]*n[7]-n[1]*n[8]);out.n[2]=invDet*(n[1]*n[5]-n[2]*n[4]);
    out.n[3]=invDet*(n[5]*n[6]-n[3]*n[8]);out.n[4]=invDet*(n[0]*n[8]-n[2]*n[6]);out.n[5]=invDet*(n[2]*n[3]-n[0]*n[5]);}
  /*void lookat(nvec3<T> look) {nvec3<T> lookN = look.normalized();nquat<T> q;q.RotateFromTo(nvec3<T>(0,0,1), lookN);
    set((q*nvec3<T>(1,0,0)).normalized(),(q*nvec3<T>(0,1,0)).normalized(),lookN);}*/
  void lookat(nvec3<T> look) {
    nvec3<T> lookN = look.normalized();nquat<T> q;q.RotateFromTo(nvec3<T>(0,0,1), lookN);fromNquat(q);
  }
  void lookat(nvec3<T> look, nvec3<T> up){nvec3<T> f = look.normalized();nvec3<T> u = up.normalized();
    if (f.dot(u) > 0.9999) u = (abs(f[2])<0.9999) ? nvec3<T>(T(0),T(0),T(1)):nvec3<T>(T(1),T(0),T(0));
    nvec3<T> r = u.cross(f);if (r.dot(r) > 0) r /= sqrt(r.dot(r));else r = nvec3<T>(T(1), T(0), T(0));nvec3<T> u2 = f.cross(r);set(r, u2, f);}
private:
  T n[9];
};
} //end ntv namespace
#endif

//Copyright 2019 Ivan DeWolf
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
