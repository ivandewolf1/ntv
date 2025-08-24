
// Lightweight templated vector library
// Dec 2019
// Ivan DeWolf
//
// this was written with the goal of brevity and ease of use.
// all code is in this .h file
// there is no error checking, 
// this was not optimized for execution speed or mimimizing RAM usage.
// this was not written to handle every possible requirement.
// this is not documented, or written for ease of comprehension.

#ifndef __nvec3_h__
#define __nvec3_h__
#include <iostream>
#include <cmath>
namespace ntv {
template <typename T>
class nvec3
{
 public:
  // ----------------------------------------------------------------------- constructors
  nvec3(){n[0]=n[1]=n[2]=0;}
  nvec3(const T& x, const T& y, const T& z){n[0] = x; n[1] = y; n[2] = z;}
  nvec3(const T& s){n[0]=n[1]=n[2]=s;}
  nvec3(const nvec3& v){n[0]=v.get(0);n[1]=v.get(1);n[2]=v.get(2);}
  // ----------------------------------------------------------------- assignment operators
  nvec3& operator += (const nvec3& v){n[0]+=v.get(0); n[1]+=v.get(1); n[2]+=v.get(2); return *this;}
  nvec3& operator -= (const nvec3& v){n[0]-=v.get(0); n[1]-=v.get(1); n[2]-=v.get(2); return *this;}
  nvec3& operator *= (const nvec3& v){n[0]*=v.get(0); n[1]*=v.get(1); n[2]*=v.get(2); return *this;}
  nvec3& operator /= (const nvec3& v){n[0]/=v.get(0); n[1]/=v.get(1); n[2]/=v.get(2); return *this;}
  nvec3& operator *= (T& s){n[0]*=s; n[1]*=s; n[2]*=s; return *this;}
  nvec3& operator /= (T& s){auto s_inv = 1.0/s; n[0]*=s_inv; n[1]*=s_inv; n[2]*=s_inv; return *this; }
  // ----------------------------------------------------------------------- accessors
  T& operator[](int i) { return n[i]; }
  const T& operator[](int i) const { return n[i]; }
  T get(int i)const{return n[i];}
  void set(T Xin, T Yin, T Zin){n[0]=Xin; n[1]=Yin; n[2]=Zin;}
  // ----------------------------------------------------------------------- vector to vector operators
  nvec3 operator-()const{return nvec3(-n[0],-n[1],-n[2]);}//unary
  nvec3 operator+(const nvec3& v)const{return nvec3(n[0] + v.get(0), n[1] + v.get(1), n[2] + v.get(2));}
  nvec3 operator-(const nvec3& v)const{return nvec3(n[0] - v.get(0), n[1] - v.get(1), n[2] - v.get(2));}
  nvec3 operator*(const nvec3& v)const{return nvec3(n[0] * v.get(0), n[1] * v.get(1), n[2] * v.get(2));}
  nvec3 operator/(const nvec3& v)const{return nvec3(n[0] / v.get(0), n[1] / v.get(1), n[2] / v.get(2));}
  // ----------------------------------------------------------------------- vector to scalar operators
  nvec3 operator+(const T& s)const{return nvec3(n[0] + s, n[1] + s, n[2] + s);}
  friend nvec3 operator+(const T& s, const nvec3& v){return nvec3(v.get(0)+s, v.get(1)+s, v.get(2)+s);}
  nvec3 operator-(const T& s)const{return nvec3(n[0] - s, n[1] - s, n[2] - s);}
  friend nvec3 operator-(const T& s, const nvec3& v){return nvec3(s-v.get(0), s-v.get(1), s-v.get(2));}
  nvec3 operator*(const T& s)const{return nvec3(n[0] * s, n[1] * s, n[2] * s);}
  friend nvec3 operator * (const T& s, const nvec3& v){return nvec3(v.get(0)*s, v.get(1)*s, v.get(2)*s);}
  nvec3 operator / (const T& s)const{double s_inv = 1.0/s;return nvec3(n[0]*s_inv, n[1]*s_inv, n[2]*s_inv);}
  friend nvec3 operator / (const T& s, const nvec3& v){return nvec3(s/v.get(0), s/v.get(1), s/v.get(2));}
  // ----------------------------------------------------------------------- equality operators
  bool operator == (const nvec3& v)const{return (n[0]==v.get(0)) && (n[1]==v.get(1)) && (n[2]==v.get(2));}
  bool operator != (const nvec3& v)const{return !(*this==v);}
  // ----------------------------------------------------------------------- utility functions
  // print
  friend std::ostream& operator << (std::ostream& s, const nvec3& v){return s<<"("<<v.get(0)<<", "<<v.get(1)<<", "<<v.get(2)<<")";}
  // mathematical
  T length2()const{return n[0]*n[0] + n[1]*n[1] + n[2]*n[2];}
  T length()const{return sqrt(length2());}
  nvec3& normalize(){(*this) /= length();return *this;}
  nvec3 normalized()const{T L = length(); return nvec3(n[0]/L,n[1]/L,n[2]/L);}
  T dot(const nvec3& v)const{return (n[0]*v.get(0) + n[1]*v.get(1) + n[2]*v.get(2));}
  nvec3 cross(const nvec3& v)const{return nvec3(n[1]*v.get(2) - n[2]*v.get(1),n[2]*v.get(0) - n[0]*v.get(2),n[0]*v.get(1) - n[1]*v.get(0));}
  // linear interpolation
  nvec3 lerp(nvec3& v, const T& t)const{return (*this) + t*(v-(*this));}
 private:
  T n[3];
};
} //end ntv namespace
#endif

//Copyright 2019 Ivan DeWolf
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
