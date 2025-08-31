
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
  nvec3() noexcept : n{T(0), T(0), T(0)} {}
  nvec3(T x, T y, T z) noexcept : n{x, y, z} {}
  nvec3(const T s) noexcept : n{s, s, s} {}
  // nvec3(const nvec3& v){n[0]=v[0];n[1]=v[1];n[2]=v[2];}
  nvec3(const nvec3&) = default;
  nvec3(nvec3&&) noexcept = default;
 // ----------------------------------------------------------------------- accessors
 T& operator[](int i)noexcept{ return n[i]; }
 const T& operator[](int i) const noexcept{ return n[i]; }
 void set(T Xin, T Yin, T Zin){n[0]=Xin; n[1]=Yin; n[2]=Zin;}
 // ----------------------------------------------------------------- compound operators
  nvec3& operator += (const nvec3& v)noexcept{n[0]+=v[0]; n[1]+=v[1]; n[2]+=v[2]; return *this;}
  nvec3& operator -= (const nvec3& v)noexcept{n[0]-=v[0]; n[1]-=v[1]; n[2]-=v[2]; return *this;}
  nvec3& operator *= (const nvec3& v)noexcept{n[0]*=v[0]; n[1]*=v[1]; n[2]*=v[2]; return *this;}
  nvec3& operator /= (const nvec3& v)noexcept{n[0]/=v[0]; n[1]/=v[1]; n[2]/=v[2]; return *this;}
  nvec3& operator *= (T& s)noexcept{n[0]*=s; n[1]*=s; n[2]*=s; return *this;}
  nvec3& operator /= (T& s)noexcept{auto s_inv = 1.0/s; n[0]*=s_inv; n[1]*=s_inv; n[2]*=s_inv; return *this; }
   // ----------------------------------------------------------------------- vector to vector operators
  nvec3 operator-()const noexcept{return nvec3(-n[0],-n[1],-n[2]);}//unary
  nvec3 operator+(const nvec3& v)const{return nvec3(n[0] + v[0], n[1] + v[1], n[2] + v[2]);}
  nvec3 operator-(const nvec3& v)const{return nvec3(n[0] - v[0], n[1] - v[1], n[2] - v[2]);}
  nvec3 operator*(const nvec3& v)const{return nvec3(n[0] * v[0], n[1] * v[1], n[2] * v[2]);}
  nvec3 operator/(const nvec3& v)const{return nvec3(n[0] / v[0], n[1] / v[1], n[2] / v[2]);}
  nvec3& operator=(const nvec3<T>& v){n[0]=v[0];n[1]=v[1];n[2]=v[2];return *this;}
  // ----------------------------------------------------------------------- vector to scalar operators
  nvec3 operator+(const T& s)const{return nvec3(n[0] + s, n[1] + s, n[2] + s);}
  friend nvec3 operator+(const T& s, const nvec3& v){return nvec3(v[0]+s, v[1]+s, v[2]+s);}
  nvec3 operator-(const T& s)const{return nvec3(n[0] - s, n[1] - s, n[2] - s);}
  friend nvec3 operator-(const T& s, const nvec3& v){return nvec3(s-v[0], s-v[1], s-v[2]);}
  nvec3 operator*(const T& s)const{return nvec3(n[0] * s, n[1] * s, n[2] * s);}
  friend nvec3 operator * (const T& s, const nvec3& v){return nvec3(v[0]*s, v[1]*s, v[2]*s);}
  nvec3 operator / (const T& s)const{double s_inv = 1.0/s;return nvec3(n[0]*s_inv, n[1]*s_inv, n[2]*s_inv);}
  friend nvec3 operator / (const T& s, const nvec3& v){return nvec3(s/v[0], s/v[1], s/v[2]);}
  // ----------------------------------------------------------------------- equality operators
  bool operator == (const nvec3& v)const{return (n[0]==v[0]) && (n[1]==v[1]) && (n[2]==v[2]);}
  bool operator != (const nvec3& v)const{return !(*this==v);}
  // ----------------------------------------------------------------------- utility functions
  // print
  friend std::ostream& operator << (std::ostream& s, const nvec3& v){return s<<"("<<v[0]<<", "<<v[1]<<", "<<v[2]<<")";}
  // math
  T length2()const noexcept{return n[0]*n[0] + n[1]*n[1] + n[2]*n[2];}
  T length()const noexcept{return sqrt(length2());}
  nvec3& normalize()noexcept{(*this) /= length();return *this;}
  nvec3 normalized()const noexcept{T L = length(); return nvec3(n[0]/L,n[1]/L,n[2]/L);}
  T dot(const nvec3& v)const noexcept{return (n[0]*v[0] + n[1]*v[1] + n[2]*v[2]);}
  nvec3 cross(const nvec3& v)const noexcept{return nvec3(n[1]*v[2] - n[2]*v[1],n[2]*v[0] - n[0]*v[2],n[0]*v[1] - n[1]*v[0]);}
  // linear interpolation
  nvec3 lerp(nvec3& v, const T& t)const noexcept{return (*this) + t*(v-(*this));}
  bool almostEqual(const nvec3& v,T eps)const noexcept{
   return std::fabs(n[0]-v.n[0]) <= eps && std::fabs(n[1]-v.n[1]) <= eps && std::fabs(n[2]-v.n[2]) <= eps;}

 private:
  T n[3];
};
} //end ntv namespace
#endif

//Copyright 2019 Ivan DeWolf
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
