/*
 * Vec3.h
 *
 *  Created on: 15 de out de 2019
 *      Author: marcus
 */

#ifndef VEC3_H_
#define VEC3_H_

class Vec3 {
public:
	Vec3(){};
	Vec3(float defx, float defy, float defz) :	x(defx),y(defy),z(defz){};
//	virtual ~Vec3(){};
	float x;
	float y;
	float z;

	inline const Vec3 Cross(const Vec3& q) const {
		Vec3 r;
		r.x = y*q.z - z*q.y; //i
		r.y = z*q.x - x*q.z; //j
		r.z = x*q.y - y*q.x; //k
		return r;
	}

	inline const float Dot(const Vec3& q) const{
		return x*q.x + y*q.y + z*q.z;
	}

	inline const Vec3 operator+(const Vec3& q) const {
		Vec3 r;
		r.x = x+q.x;
		r.y = y+q.y;
		r.z = z+q.z;
		return r;
	}

	inline const Vec3 operator*(const float k) const{
		Vec3 r;
		r.x = k*x;
		r.y = k*y;
		r.z = k*z;
		return r;
	}

	void Print(); // IMPLEMENTAR NA APP
};

#endif /* VEC3_H_ */
