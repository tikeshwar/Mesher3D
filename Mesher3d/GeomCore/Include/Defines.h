#pragma once 

#include <unordered_set>
#include <limits>

constexpr static double PI = 3.14159265359;
constexpr static double TOL = 1e8*std::numeric_limits<double>::epsilon();
constexpr static double ZERO_TOL = TOL;

namespace GCore
{	
	template <typename T> class Vec2;
	template <typename T> class Vec3;
	template <typename T> class Vec4;
	template <typename T> class Point4;
	template <typename T> class Box2;
	template <typename T> class Box3;
	template <typename T> class Mat4x4;

	typedef Vec2<double> Vec2d;
	typedef Vec3<double> Vec3d;
	typedef Vec2<size_t> Vec2i;
	typedef Vec3<size_t> Vec3i;
	typedef Vec2<char> Vec2c;
	typedef Vec3<char> Vec3c;
	typedef Box2<double> Box2d;
	typedef Box3<double> Box3d;
	typedef Mat4x4<double> Mat4x4d;

	typedef Vec4<double> Vec4d;
	typedef Point4<double> Point4d;
}
