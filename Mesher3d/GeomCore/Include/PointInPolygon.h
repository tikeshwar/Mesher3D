#pragma once

#include <vector>
#include "Defines.h"
#include "Vec.h"
#include "Mat4x4.h"

namespace GCore
{
	// http://geomalgorithms.com/a03-_inclusion.html
	class PointInPolygon
	{
	public:
		PointInPolygon(const std::vector<Vec3d>& points)
			:mPoints(points) 
		{
			transformToXY();
		}

		bool inside(const Vec3d& point)const
		{
			return orientTest(point) != 0;
		}

	private:
		void transformToXY();

		double isLeft(const Vec3d& P0, const Vec3d& P1, const Vec3d& P2)const
		{
			return ((P1.x - P0.x) * (P2.y - P0.y)
				- (P2.x - P0.x) * (P1.y - P0.y));
		}

		//crossing number test for a point in a polygon
		//0 = outside, 1 = inside
		int cnPnPoly(const Vec3d& P)const;

		//winding number test for a point in a polygon
		//wn = the winding number(= 0 only when P is outside)
		int wnPnPoly(const Vec3d& P)const;

		int orientTest(const Vec3d& P)const;

	private:
		std::vector<Vec3d> mPoints;
		Mat4x4d mXYMat;
	};
}
