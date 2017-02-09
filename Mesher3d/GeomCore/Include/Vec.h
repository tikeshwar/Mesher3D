#pragma once

#include <cmath>
#include <ostream>
#include "Tolerance.h"

namespace GCore
{
	template <typename T = double>
	class Vec2
	{
	public:
		Vec2()
			:x(0), y(0) {}
		Vec2(T _x, T _y)
			:x(_x), y(_y) {}
		Vec2(T xy[2])
			:x(xy[0]), y(xy[1]) {}

		bool operator==(const Vec2& right)const
		{
			return isEqual(x, right.x)
				&& isEqual(y, right.y);
		}

		void normalize()
		{
			T magInv = 1 / magnitude();
			x *= magInv;
			y *= magInv;
		}

		T angleCounterClockwise(const Vec2& vec2)const
		{
			T a = atan2(vec2.y, vec2.x);
			T b = atan2(y, x);
			if (a < 0)
				a = 2 * PI + a;
			if (b < 0)
				b = 2 * PI + b;
			return a - b;
		}
		void set(T _x, T _y)
		{
			x = _x;
			y = _y;
		}
		T magnitudeSqr()const
		{
			return (x*x + y*y);
		}
		T magnitude()const
		{
			return sqrt(magnitudeSqr());
		}
		T maxofxy()const
		{
			return fmax(x, y);
		}
		T minofxy()const
		{
			return fmin(x, y);
		}
		Vec2 operator+(const Vec2& vec2)const
		{
			return Vec2(vec2.x + x, vec2.y + y);
		}
		Vec2 operator-(const Vec2& vec2) const
		{
			return Vec2(-vec2.x + x, -vec2.y + y);
		}
		Vec2 operator*(double scalar)const
		{
			return Vec2(x*scalar, y*scalar);
		}
		Vec2 operator/(double scalar)const
		{
			return Vec2(x / scalar, y / scalar);
		}
		Vec2 direction(const Vec2& vec2)const
		{
			Vec2 dir = vec2 - *this;
			dir.normalize();
			return dir;
		}
		T dot(const Vec2& vec2)	const
		{
			return vec2.x*x + vec2.y*y;
		}
		Vec2 projectionOnLine(const Vec2& p, const Vec2&q)
		{
			Vec2 dir = p.direction(q);
			return p + dir*dir.dot((*this) - p);
		}
		T perpendicularDistanceToLine(const Vec2& p, const Vec2&q)
		{
			return ((*this) - projectionOnLine(p, q)).magnitude();
		}
	public:
		T x, y;
	};

	template <typename T = double>
	class Vec3
	{
	public:
		Vec3()
			:x(0), y(0), z(0) {}

		Vec3(T _x, T _y, T _z)
			:x(_x), y(_y), z(_z) {}

		Vec3(T xyz[3])
			: x(xyz[0]), y(xyz[1]), z(xyz[2]) {}

		bool operator==(const Vec3& right)const
		{
			return isEqual(x, right.x)
				&& isEqual(y, right.y)
				&& isEqual(z, right.z);
		}

		void normalize()
		{
			T magInv = 1 / magnitude();
			x *= magInv;
			y *= magInv;
			z *= magInv;
		}

		Vec2<T> xy()const
		{
			return Vec2<T>(x, y);
		}

		Vec3 normalize()const
		{
			Vec3 vec = *this;
			T magInv = 1 / vec.magnitude();
			vec.x *= magInv;
			vec.y *= magInv;
			vec.z *= magInv;
			return vec;
		}

		T angleCounterClockwise(const Vec3& vec3)const
		{
			T a = atan2(vec3.y, vec3.x);
			T b = atan2(y, x);
			if (a < 0)
				a = 2 * PI + a;
			if (b < 0)
				b = 2 * PI + b;
			return a - b;
		}

		void set(T _x, T _y, T _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}
		T magnitudeSqr()const
		{
			return (x*x + y*y + z*z);
		}
		T magnitude()const
		{
			return sqrt(magnitudeSqr());
		}
		T maxofxyz()const
		{
			return fmax(x, fmax(y, z));
		}
		T minofxyz()const
		{
			return fmin(x, fmin(y, z));
		}
		Vec3 operator+(const Vec3& vec3)const
		{
			return Vec3(vec3.x + x, vec3.y + y, vec3.z + z);
		}
		Vec3 operator-(const Vec3& vec3) const
		{
			return Vec3(-vec3.x + x, -vec3.y + y, -vec3.z + z);
		}
		Vec3 operator*(double scalar)const
		{
			return Vec3(x*scalar, y*scalar, z*scalar);
		}
		Vec3 operator/(double scalar)const
		{
			return Vec3(x / scalar, y / scalar, z / scalar);
		}
		Vec3 direction(const Vec3& vec3)const
		{
			Vec3 dir = vec3 - *this;
			dir.normalize();
			return dir;
		}
		T dot(const Vec3& vec3)	const
		{
			return vec3.x*x + vec3.y*y + vec3.z*z;
		}
		Vec3 cross(const Vec3& vec3)const
		{
			return Vec3(y*vec3.z - z*vec3.y, z*vec3.x - x*vec3.z, x*vec3.y - y*vec3.x);
		}
		Vec3 projectionOnPlane(const Vec3& norm, const Vec3& pt)const
		{
			Vec3 v2pt = (*this) - pt;
			Vec3 nvec = norm*(v2pt.dot(norm));
			Vec3 dvec = v2pt - nvec;
			return pt + dvec;
		}
		Vec3 projectionOnLine(const Vec3& p, const Vec3& q)const
		{
			Vec3 dir = p.direction(q);
			Vec3 this2p = p.direction(*this);
			return p + dir*(this2p.dot(dir));
		}
		T perpendicularDistanceToLineSeg(const Vec3& p, const Vec3& q)const
		{
			Vec3 dir = p.direction(q);
			return ((*this) - p).cross(dir).magnitude();
		}
		T shortestDistanceToLineSeg(const Vec3& p, const Vec3& q)const
		{
			Vec3 t2p = ((*this) - p);
			Vec3 t2q = ((*this) - q);
			if (t2p.dot(q - p) < 0)
				return t2p.magnitude();
			else if (t2q.dot(p - q) < 0)
				return t2q.magnitude();

			return perpendicularDistanceToLineSeg(p, q);
		}
		Vec3 closestPointToLineSeg(const Vec3& p, const Vec3& q)const
		{
			Vec3 t2p = ((*this) - p);
			Vec3 t2q = ((*this) - q);
			if (t2p.dot(q - p) < 0)
				return p;
			else if (t2q.dot(p - q) < 0)
				return q;

			return projectionOnLine(p, q);
		}
		Vec3 rotate(T angle, const Vec3& axis)const
		{
			// using rodriguez roatation formula
			// https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
			// v' = vcosa + (kxv)sina + k*(k.v)(1-cosa)
			T radAngle = PI*angle / 180.0;
			Vec3 v = *this;
			Vec3 k = axis;
			k.normalize();
			return v*cos(radAngle) + (k.cross(v))*sin(radAngle) + k*(k.dot(v))*(1 - cos(radAngle));

		}

		friend	std::ostream & operator<<(std::ostream &output, const Vec3& p)
		{
			output << "(" << p.x << ", " << p.y << ", " << p.z << ")";
			return output;
		}

	public:
		T x, y, z;
	};
}