
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

#ifndef __tmat4_h__
#define __tmat4_h__
#include <iostream>
#include <math.h>
#include "tvec3.h"
namespace lwtv {
template <typename T>
class tmat4
{
public:
  // ######################################## constructors
  tmat4(){this->identity();}
  tmat4(const T& s){for(int i=0; i<16; ++i)n[i]=s;}
  tmat4(const tmat4& m){for(int i=0; i<16; ++i)n[i]=m.get(i);}
  // ######################################## Assignment operators
  tmat4& operator += (const tmat4& m){for(int i=0; i<16; ++i)n[i]+=m.get(i);return *this;}
  tmat4& operator -= (const tmat4& m){for(int i=0; i<16; ++i)n[i]-=m.get(i);return *this;}
  tmat4& operator *= (const T& s){for(int i=0; i<16; ++i)n[i]*=s;return *this;}
  tmat4& operator /= (const T& s){for(int i=0; i<16; ++i)n[i]/=s;return *this;}
  T& operator [] (const int& i){return n[i];}
  T& operator () (const int& r, const int c){return n[(r*4) + c];}
  // ######################################## matrix4 scalar operators
  friend tmat4 operator * (const tmat4& m, const T& s){tmat4 out;for(int i=0; i<16; ++i)out[i]=m.get(i)*s;return out;}
  friend tmat4 operator / (const tmat4& m, const T& s){tmat4 out;for(int i=0; i<16; ++i)out[i]=m.get(i)/s;return out;}
  // ######################################## matrix4 vector operators
  T rowVMult(const int& index, const tvec3<T>& v)const{T out = 0;for(int i=0;i<3;++i)out+=n[(index*4)+i]*v.get(i);return out+n[(index*4)+3];}// perhaps private?
  friend tvec3<T> operator * (const tmat4& m, const tvec3<T>& v){tvec3<T> out;for(int i=0;i<3;++i)out[i]=m.rowVMult(i,v);return out/m.rowVMult(3,v);} 
  // ######################################## matrix4 matrix4 operators
  tmat4 operator + (const tmat4& m){tmat4 out;for(int i=0; i<16; ++i)out[i]=n[i]+m.get(i);return out;}
  tmat4 operator - (const tmat4& m){tmat4 out;for(int i=0; i<16; ++i)out[i]=n[i]-m.get(i);return out;}
  friend tmat4 operator * (const tmat4& mA, const tmat4& mB){tmat4 out(0);for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int k=0;k<4;++k)out(i,j)+=mA.get(i,k)*mB.get(k,j);return out;}
  // ######################################## equality operators
  bool operator == (const tmat4& m){for(int i=0; i<16; ++i)if(n[i]!=m.get(i))return false;return true;}
  bool operator != (const tmat4& m){return !(*this==m);}
  // ######################################## accessor functions
  T get(const int& i)const{return n[i];}
  T get(const int& r, const int& c)const{return (n[(r*4) + c]);}
  tvec3<T> getRowVec3(int index){tvec3<T> out; for(int i=0; i<3; ++i)out[i]=get(index, i);return out;}
  tvec3<T> getColVec3(int index){tvec3<T> out; for(int i=0; i<3; ++i)out[i]=get(i, index);return out;}
  void set(const tmat4& m){for(int i=0; i<16; ++i){n[i]=m.get(i);}}
  void setVecRow(const int i,const tvec3<T> v){n[(4*i)+0]=v.get(0); n[(4*i)+1]=v.get(1); n[(4*i)+2]=v.get(2);}
  // ######################################## utility functions
  friend std::ostream& operator << (std::ostream& s, const tmat4& m){
    s<<"\n("<<m.n[0]<<", "<<m.n[1]<<", "<<m.n[2]<<", "<<m.n[3]<<")";
    s<<"\n("<<m.n[4]<<", "<<m.n[5]<<", "<<m.n[6]<<", "<<m.n[7]<<")";
    s<<"\n("<<m.n[8]<<", "<<m.n[9]<<", "<<m.n[10]<<", "<<m.n[11]<<")";
    s<<"\n("<<m.n[12]<<", "<<m.n[13]<<", "<<m.n[14]<<", "<<m.n[15]<<")";
    return s;}
  void identity(){
    n[0] =1;n[1] =0;n[2] =0;n[3] =0;
    n[4] =0;n[5] =1;n[6] =0;n[7] =0;
    n[8] =0;n[9] =0;n[10]=1;n[11]=0;
    n[12]=0;n[13]=0;n[14]=0;n[15]=1;}
  void setRotation(const tvec3<T>& Axis, const T angle) {
    T c = cos(angle); T s = sin(angle); T t = 1.0 - c; tvec3<T> A,B,C;
    A.set(t*Axis.get(0)*Axis.get(0) + c, t*Axis.get(0)*Axis.get(1) + s*Axis.get(2), t*Axis.get(0)*Axis.get(2) - s*Axis.get(1));
    B.set(t*Axis.get(0)*Axis.get(1) - s*Axis.get(2), t*Axis.get(1)*Axis.get(1) + c, t*Axis.get(1)*Axis.get(2) + s*Axis.get(0));
    C.set(t*Axis.get(0)*Axis.get(2) + s*Axis.get(1), t*Axis.get(1)*Axis.get(2) - s*Axis.get(0), t*Axis.get(2)*Axis.get(2) + c);
    setVecRow(0, A); setVecRow(1, B); setVecRow(2, C);}
  void rotate(const tvec3<T>& Axis, const T angle) {
    tmat4<T> Mr, result(0); Mr.setRotation(Axis, angle);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int k=0;k<4;++k)result(i,j)+=get(i,k)*Mr.get(k,j);
    set(result);}
  void setRotation(const tvec3<T>& Rot){
    tmat4<T> xyz[3];
    int idxA, idxB, idxC, idxD;
    for(int i=0; i<3; ++i){
      T Crot = cos(Rot.get(i)); T Srot = sin(Rot.get(i));
      if(i==0){idxA=1;idxB=2;idxC=1,idxD=2;} if(i==1){idxA=0;idxB=2;idxC=2,idxD=0;} if(i==2){idxA=0;idxB=1;idxC=0,idxD=1;}
      xyz[i](idxA,idxA)=Crot; xyz[i](idxB,idxB)=Crot; xyz[i](idxC,idxD)=Srot; xyz[i](idxD,idxC)=-Srot;}
    set(xyz[0]*xyz[1]*xyz[2]);}
  void rotate(const tvec3<T>& Rot) {
    tmat4<T> Mr, result(0); Mr.setRotation(Rot);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int k=0;k<4;++k)result(i,j)+=get(i,k)*Mr.get(k,j);
    set(result);}
  void translate(const tvec3<T>& v){n[12]+=v.get(0); n[13]+=v.get(1); n[14]+=v.get(2);}
  void scale(const tvec3<T>& v){for(int r=0; r<3; ++r){for(int c=0; c<3; ++c){n[(r*4)+c] *= v.get(c);}}}
  void transpose(){
    tmat4<T> holder;
    for(int c=0;c<4;++c)for(int r=0;r<4;++r)holder(c,r) = n[(r*4) + c];
    for(int i=0; i<16; ++i)n[i] = holder[i];}
  T determinant2d(const int &a, const int &b, const int &c, const int &d){return n[a] * n[d] - n[c] * n[b];}
  T determinant(){
    T ab=determinant2d(8,12,9,13);
    T ac=determinant2d(8,12,10,14);
    T ad=determinant2d(8,12,11,15);
    T bc=determinant2d(9,13,10,14);
    T bd=determinant2d(9,13,11,15);
    T cd=determinant2d(10,14,11,15);
    T ABC=n[4]*bc-n[5]*ac+n[6]*ab;
    T ACD=n[4]*cd-n[6]*ad+n[7]*ac;
    T ABD=n[4]*bd-n[5]*ad+n[7]*ab;
    T BCD=n[5]*cd-n[6]*bd+n[7]*bc;
    T one=n[0]*BCD;T two=n[1]*ACD;T three=n[2]*ABD;T four=n[3]*ABC;
    return one-two+three-four;}
  tmat4 adjoint(){
  	tmat4 out;
	  T abl = determinant2d( 8,12, 9,13);T abs = determinant2d(4,12,5,13);T abu = determinant2d(4,8,5,9 );
	  T acl = determinant2d( 8,12,10,14);T acs = determinant2d(4,12,6,14);T acu = determinant2d(4,8,6,10);
	  T adl = determinant2d( 8,12,11,15);T ads = determinant2d(4,12,7,15);T adu = determinant2d(4,8,7,11);
	  T bcl = determinant2d( 9,13,10,14);T bcs = determinant2d(5,13,6,14);T bcu = determinant2d(5,9,6,10);
	  T bdl = determinant2d( 9,13,11,15);T bds = determinant2d(5,13,7,15);T bdu = determinant2d(5,9,7,11);
	  T cdl = determinant2d(10,14,11,15);T cds = determinant2d(6,14,7,15);T cdu = determinant2d(6,10,7,11);	
	  T ABCll=n[4]*bcl-n[5]*acl+n[6]*abl;T ABCul=n[0]*bcl-n[1]*acl+n[2]*abl;T ABCus=n[0]*bcs-n[1]*acs+n[2]*abs;T ABCuu=n[0]*bcu-n[1]*acu+n[2]*abu;
	  T ACDll=n[4]*cdl-n[6]*adl+n[7]*acl;T ACDul=n[0]*cdl-n[2]*adl+n[3]*acl;T ACDus=n[0]*cds-n[2]*ads+n[3]*acs;T ACDuu=n[0]*cdu-n[2]*adu+n[3]*acu;
	  T ABDll=n[4]*bdl-n[5]*adl+n[7]*abl;T ABDul=n[0]*bdl-n[1]*adl+n[3]*abl;T ABDus=n[0]*bds-n[1]*ads+n[3]*abs;T ABDuu=n[0]*bdu-n[1]*adu+n[3]*abu;
	  T BCDll=n[5]*cdl-n[6]*bdl+n[7]*bcl;T BCDul=n[1]*cdl-n[2]*bdl+n[3]*bcl;T BCDus=n[1]*cds-n[2]*bds+n[3]*bcs;T BCDuu=n[1]*cdu-n[2]*bdu+n[3]*bcu;
	  out[0]=BCDll;out[1]=-ACDll;out[2]=ABDll;out[3]=-ABCll;
	  out[4]=BCDul;out[5]=-ACDul;out[6]=ABDul;out[7]=-ABCul;
	  out[8]=BCDus;out[9]=-ACDus;out[10]=ABDus;out[11]=-ABCus;
	  out[12]=BCDuu;out[13]=-ACDuu;out[14]=ABDuu;out[15]=-ABCuu;
	  out.transpose();return out;}
  tmat4 inverse(){return adjoint() * (1.0/determinant());}
  void extract(tvec3<T>& Trans,tvec3<T>& Rot,tvec3<T>& Scl){
    for(int i=0; i<3; ++i)Scl[i] = getColVec3(i).length();
    Trans = getRowVec3(3);
    Rot[0] = -atan2(-(get(1,2)/Scl[2]), (get(2,2)/Scl[2]));
    T cosYangle = sqrt(pow((get(0,0)/Scl[0]), 2) + pow((get(0,1)/Scl[1]), 2));
    Rot[1] = -atan2((get(0,2)/Scl[2]), cosYangle);
    T sinXangle = sin(-Rot[0]);
    T cosXangle = cos(-Rot[0]);
    Rot[2] = -atan2(cosXangle * (get(1,0)/Scl[0]) + sinXangle * (get(2,0)/Scl[0]), cosXangle * (get(1,1)/Scl[1]) + sinXangle * (get(2,1)/Scl[1]));
  }
private:
  T n[16];
};
} //end lwtv namespace
#endif

// determinant hint:
//  1 | 2 | 3 | 4 
// ##########------
//  A | B | C | D
// ##########------
//  a | b | c | d   
// -|---|---|---|--
//  a | b | c | d 

//Copyright 2019 Ivan DeWolf
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
