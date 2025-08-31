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
        nquat(T i,T j,T k, T r)noexcept:n{i, j, k, r} {}
        nquat(T r,nvec3<T> ijk)noexcept:n{ijk[0], ijk[1], ijk[2], r} {}
        nquat(const nquat&) = default;
        nquat(nquat&&) noexcept = default;
        // ----------------------------------------------- element access operators
        T& operator [] (int i){return n[i];}
        const T& operator[](int i)const{return n[i];}
        void set(T r, const nvec3<T> ijk)noexcept{n[0]=ijk[0];n[1]=ijk[1];n[2]=ijk[2];n[3]=r;}
        T real()const noexcept{return n[3];}//real, scalar component of this quaternion
        nvec3<T> imag()const noexcept{return nvec3<T>(n[0],n[1],n[2]);}//imaginary, vector component of this quaternion
        // ---------------------------------------------- mutators
        nquat& operator *= (const T& s){n[0]*=s;n[1]*=s;n[2]*=s;n[3]*=s;return *this;}
        void identity()noexcept{n[0]=0;n[1]=0;n[2]=0;n[3]=1;}
        void normalize(){*this=this->normalized();}
        void fromAxisAngle(const nvec3<T>& axis, T angle){T halfAngle=0.5*angle;set(cos(halfAngle),axis.normalized()*sin(halfAngle));}
        // ----------------------------------------------- builders
        nquat normalized()const{nquat q(*this);q*=1/q.length();return q;}// assumes initial nonzero length
        nquat conjugate()const noexcept{return nquat(-n[0], -n[1], -n[2], n[3]);}// assumes unit length input
        // ------------------------------------------- quaternion vector operator (rotate vector)
        nvec3<T> operator*(const nvec3<T>& v)const{T rr=real()+real();return rr*imag().cross(v)+(rr*real()-1)*v+2*imag().dot(v)*imag();}
        // ----------------------------------------- quaternion quaternion operator
        friend nquat operator*(const nquat& qA,const nquat& qB) {T r = qA.real()*qB.real()-qA.imag().dot(qB.imag());
            nvec3<T> i = qA.real()*qB.imag()+qB.real()*qA.imag()+qA.imag().cross(qB.imag());return nquat(r,i);}
        nquat& operator=(const nquat<T>& q){n[0]=q[0];n[1]=q[1];n[2]=q[2];n[3]=q[3];return *this;}
       // --------------------------------------------- utility functions
        friend std::ostream& operator << (std::ostream& s, const nquat<T>& q){s<<"\n("<<q.n[0]<<", "<<q.n[1]<<", "<<q.n[2]<<", "<<q.n[3]<<")";return s;}
        T length2() const noexcept {return n[0]*n[0] + n[1]*n[1] + n[2]*n[2] + n[3]*n[3];}
        T length() const noexcept {return std::sqrt(length2());}
        T dot(const nquat& qB)const noexcept{return real()*qB.real()+imag().dot(qB.imag());}
        nquat scaleAngle(T s)const {nvec3<T> axis;T angle;toAxisAngle(axis,angle);nquat q;q.fromAxisAngle(axis,angle*s);return q;}
        void toAxisAngle(nvec3<T>& axis, T& angle)const {axis = real() > 0.0 ? imag() : -imag();
            if(axis.length2()==0){axis.set(1,0,0),angle=0;return;}angle=2.0*atan2(axis.length2(),real()>0?real():-real());}
        nquat slerp(const nquat& q2,T tau)const {if(this->dot(q2)>0.999999f)return (this->scaleAngle(1-tau)*q2.scaleAngle(tau)).normalized();
            return (*this*((this->conjugate() * q2).scaleAngle(tau))).normalized();}
        nvec3<T> perpendicular(const nvec3<T>& v)const{nvec3<T> axis = nvec3<T>(1,0,0).cross(v);if(axis.length2()<0.05)axis=nvec3<T>(0,1,0).cross(v);return axis;}
        void RotateFromTo(const nvec3<T>& v1, const nvec3<T>& v2) {nvec3<T> start = v1.normalized();nvec3<T> end = v2.normalized();T dot_product = start.dot(end);
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
