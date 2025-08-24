// Lightweight templated matrix3 library
// Aug 2025
// Ivan DeWolf
//
// this was written with the goal of brevity and ease of use.
// all code is in this .h file
// there is no error checking,
// this was not optimized for execution speed or mimimizing RAM usage.
// this was not written to handle every possible requirement.
// this is not documented, or written for ease of comprehension.
// see https://google.github.io/mathfu/quaternion_8h_source.html

#ifndef __nquat_h__
#define __nquat_h__
#include <iostream>
#include <cmath>
#include "nmat3.h"
#include "nvec3.h"
namespace ntv {
    template <typename T>
    class nquat // nano quaternion, quaternions for geometric transforms
    {
    public:
        // ------------------------------------------------- constructors
        nquat(){this->identity();}
        nquat(T r, T i, T j, T k){n[0]=r;n[1]=i;n[2]=j;n[3]=k;}
        nquat(T r, nvec3<T> ijk){n[0]=r;n[1]=ijk[0];n[2]=ijk[1];n[3]=ijk[2];}
        // ----------------------------------------------- assignment operators
        T& operator [] (int i){return n[i];}
        const T& operator[](int i)const{return n[i];}
        nquat& operator *= (const T& s){n[0]*=s;n[1]*=s;n[2]*=s;n[3]*=s;return *this;}
        void identity(){n[0]=1;n[1]=0;n[2]=0;n[3]=0;}
        void normalize(){T length = sqrt(this->dot(*this));*this*=1/length;}
        void set(T r, nvec3<T> ijk){n[0]=r;n[1]=ijk[0];n[2]=ijk[1];n[3]=ijk[2];}
        void fromAxisAngle(const nvec3<T>& axis, T angle){T halfAngle=0.5*angle;set(cos(halfAngle),axis.Normalized()*sin(halfAngle));}
        // ----------------------------------------------- builders
        nquat normalized()const{return nquat(*this).normalize();}
        nquat inverse(){return nquat(n[0], -n[1], -n[2], -n[3]);}
        T real()const{return n[0];}//real, scalar component of this quaternion
        nvec3<T> imag()const{return nvec3<T>(n[1],n[2],n[3]);}//imaginary, vector component of this quaternion
        // -------------------------------------------- quaternion scalar operators
        friend nquat operator*(const T s){nvec3<T> axis;T angle;toAxisAngle(axis,angle);return nquat().fromAxisAngle(axis,angle*s);}
        // ------------------------------------------- quaternion vector operators
        nvec3<T> operator*(nvec3<T>& v)const{T rr=n[0]+n[0];return rr*imag().cross(v)+(rr*n[0]-1)*v+2*imag().dot(v)*imag();}
        // ----------------------------------------- quaternion quaternion operators
        friend nquat operator*(const nquat& qA,const nquat& qB){return nquat(qA.dot(qB),qA.real()*qB.real()+qB.real()*qA.imag()+qA.imag().cross(qB.imag()));}
        // -------------------------------------------- equality functions
        // --------------------------------------------- utility functions
        T dot(const nquat& qB)const{return real()*qB.real()+imag().dot(qB.imag());}
        void toAxisAngle(nvec3<T>& axis, T& angle){*axis=real()>0?imag():-imag();*angle=2.0*atan2(axis.normalized(),real()>0?real():-real());}
        const nquat slerp(const nquat& q2,T tau) {
            if(this->dot(q2)>0.999999f)return nquat(this*(1-tau)+q2*tau);return this*((this.inverse() * q2) * tau);}
        nvec3<T> perpendicular(nvec3<T> v){nvec3<T> axis = nvec3(1,0,0).cross(v);if(axis.length2()<0.05)axis=nvec3(0,1,0).cross(v);return axis;}
        void RotateFromTo(const nvec3<T> v1, const nvec3<T> v2) {
            nvec3<T> start = v1.Normalized();nvec3<T> end = v2.Normalized();T dot_product = start.dot(end);
            if (dot_product >= 0.99999847691) {identity();return;}if (dot_product <= -0.99999847691) {perpendicular(start);return;}
            nvec3<T> crossProd = start.cross(end);set(1.0 + dot_product, crossProd);normalize();}
    private:
        T n[4];
    };
} //end ntv namespace
#endif

//Copyright 2025 Ivan DeWolf
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
