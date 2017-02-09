#pragma once

#include "Tolerance.h"
#include "Defines.h"
#include "Vec.h"
#include "BBox.h"

namespace GCore
{
	class Sphere
	{
	public:
		Sphere() = default;
		Sphere(double _radius, const Vec3d& _center)
			:radius(_radius), center(_center) {}

		Sphere(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);

		bool inside(const Vec3d& point)const
		{
			return isSmaller<double>((center - point).magnitudeSqr(), radius*radius);
		}

		Box3d boundingBox()const;

	public:
		double radius;
		Vec3d center;
	};
}