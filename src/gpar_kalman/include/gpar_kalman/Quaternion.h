/*
 * Quaternion.h
 *
 *  Created on: 15 de jul de 2019
 *      Author: marcus
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "math.h"
#include "Vec3.h"

namespace MD {

class Quaternion {

public:


	float w;
	Vec3 v;

	//Methods

	Quaternion(){
		w = 1.0;
		v = Vec3(0,0,0);
	}

	Quaternion(float defw,float defx, float defy, float defz){
		w = defw;
		v = Vec3(defx,defy,defz);
	}

	inline const Quaternion operator+(const Quaternion& q) const {
		Quaternion r;
		r.w = w+q.w;
		r.v.x = v.x+q.v.x;
		r.v.y = v.y+q.v.y;
		r.v.z = v.z+q.v.z;
		return r;
	}

	inline const Quaternion operator-(const Quaternion& q) const {
		Quaternion r;
		r.w = w-q.w;
		r.v.x = v.x-q.v.x;
		r.v.y = v.y-q.v.y;
		r.v.z = v.z-q.v.z;
		return r;
	}

	inline const Quaternion operator*(const Quaternion& q) const {
		Quaternion r;
		r.w = w*q.w - v.Dot(q.v);
		r.v = v.Cross(q.v);

		Vec3 tmp1 = v*q.w;
		Vec3 tmp2 = q.v*w;
		r.v = r.v+tmp1+tmp2;
		return r;
	}

	//scanle
	inline const Quaternion operator*(const float k) const {
		Quaternion r;
		r.w = w*k;
		r.v = v*k;
		return r;
	}

	inline const float Norm() const {
		float d = v.Dot(v);
		d += w*w;
		return sqrtf(d);
	}

	inline const Quaternion Normalize() const {
		Quaternion r;
		float norm = Norm();
		r.w = w / norm;
		r.v = v*(1/norm);
		return r;
	}

	inline const Quaternion Conjugate() const {
		Quaternion r;
		r.w = this->w;
		r.v = this->v*-1.0f;
		return r;
	}

	inline const Quaternion Inverse() const{
		Quaternion r;
		float d = this->Norm();
		d = d*d;

		r = this->Conjugate();

		return (r*(1/d));
	}
	// r = qpq^-1 = qpq* || Body2Nav
    inline const Vec3 RotatePoint(const Vec3& point) const {
		Quaternion p,r;

		p.w = 0;
		p.v = point;

		Quaternion q_inv = this->Inverse();
		r = (*this)*p;
		r = r*q_inv;

		//TODO Testar r = (*this)*p*q_inv
		return r.v;
	}

	// r = qpq^-1 = qpq* || Body2Nav
//	inline const Vec3 RotatePoint(const Quaternion& qpoint){
//		Vec3 r = this->RotatePoint(qpoint.v);
//		return r;
//	}

	// r = q^-1pq = q*pq || Nav2Body
	inline  Vec3 RotateFrame(const Vec3& point) const {
		Quaternion p,r;

		p.w = 0;
		p.v = point;

		Quaternion q_inv = this->Inverse();
		r = q_inv*p;
		r = r*(*this);

		//TODO Testar r = q_inv*p*(*this)
		return r.v;
	}


	//Implementar na aplicação
	static void Print(const Quaternion& q);



};

};

#endif /* QUATERNION_H_ */
