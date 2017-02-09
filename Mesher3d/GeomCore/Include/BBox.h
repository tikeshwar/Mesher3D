#pragma once

#include <cmath>
#include "Defines.h"
#include "Vec.h"

namespace GCore
{
	template <typename T = double>
	class Box2
	{
	public:
		Box2()
		{
			lower.set(HUGE, HUGE);
			upper.set(-HUGE, -HUGE);
		}

		Box2(const Vec2d& p, const Vec2d& q)
		{
			upper.set(fmax(p.x, q.x), fmax(p.y, q.y));
			lower.set(fmin(p.x, q.x), fmin(p.y, q.y));
		}

		bool operator==(const Box2& right)const
		{
			return lower == right.lower && upper == right.upper;
		}

		void extend(const Vec2d& point)
		{
			lower.set(fmin(lower.x, point.x), fmin(lower.y, point.y));
			upper.set(fmax(upper.x, point.x), fmax(upper.y, point.y));
		}

		void scale(double fx, double fy)
		{
			Vec2d centre = center();
			Vec2d diagnl = diagonal();
			diagnl.set(diagnl.x*fx, diagnl.y*fy);

			//double diaglen = diagnl.magnitude();
			extend(centre - (diagnl / 2.0));
			extend(centre + (diagnl / 2.0));
		}

		Vec2d diagonal()const
		{
			return (upper - lower);
		}

		Vec2d center()const
		{
			return (lower + upper) / 2.0;
		}

		bool contains(const Vec2d& point)const
		{
			return (isGreater(point.x, lower.x)
				&& isGreater(point.y, lower.y)
				&& isSmaller(point.x, upper.x)
				&& isSmaller(point.y, upper.y));
		}

		bool intersects(const Box2& box)const
		{
			return box.contains(lower) || box.contains(upper) ||
				contains(box.lower) || contains(box.upper);
		}

		friend	std::ostream & operator<<(std::ostream &output, const Box2& p)
		{
			output << "lower: " << p.lower << " upper: " << p.upper << endl;
			return output;
		}

	public:
		Vec2d lower, upper;
	};

	template <typename T = double>
	class Box3
	{
	public:
		Box3()
		{
			lower.set(HUGE, HUGE, HUGE);
			upper.set(-HUGE, -HUGE, -HUGE);
		}

		Box3(const Vec3d& p, const Vec3d& q)
		{
			lower.set(fmin(p.x, q.x), fmin(p.y, q.y), fmin(p.z, q.z));
			upper.set(fmax(p.x, q.x), fmax(p.y, q.y), fmax(p.z, q.z));
		}

		bool operator==(const Box3& right)const
		{
			return lower == right.lower && upper == right.upper;
		}

		void extend(const Vec3d& point)
		{
			lower.set(fmin(lower.x, point.x), fmin(lower.y, point.y), fmin(lower.z, point.z));
			upper.set(fmax(upper.x, point.x), fmax(upper.y, point.y), fmax(upper.z, point.z));
		}

		void scale(double fx, double fy, double fz)
		{
			Vec3d centre = center();
			Vec3d diagnl = diagonal();
			diagnl.set(diagnl.x*fx, diagnl.y*fy, diagnl.z*fz);

			//double diaglen = diagnl.magnitude();
			extend(centre - (diagnl / 2.0));
			extend(centre + (diagnl / 2.0));
		}

		Vec3d diagonal()
		{
			return (upper - lower);
		}

		Vec3d center()const
		{
			return (lower + upper) / 2.0;
		}

		bool contains(const Vec3d& point)const
		{
			return (isGreater(point.x, lower.x)
				&& isGreater(point.y, lower.y)
				&& isGreater(point.z, lower.z)
				&& isSmaller(point.x, upper.x)
				&& isSmaller(point.y, upper.y)
				&& isSmaller(point.z, upper.z));
		}

		bool intersects(const Box3& box)const
		{
			return box.contains(lower) || box.contains(upper) ||
				contains(box.lower) || contains(box.upper);
		}

		friend	std::ostream & operator<<(std::ostream &output, const Box3& p)
		{
			output << "lower: " << p.lower << " upper: " << p.upper << endl;
			return output;
		}

	public:
		Vec3d lower, upper;
	};
}