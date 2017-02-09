#pragma once

#include "Defines.h"
#include "Vec.h"

namespace GCore
{
	class Circle
	{
	public:
		Circle(Vec2d inCenter, double inRadius)
			:center(inCenter), radius(inRadius) {}

		bool isInside(const Vec2d & inPoint)const
		{
			return (inPoint - center).magnitudeSqr() < radius*radius;
		}

	public:
		Vec2d center;
		double radius;
	};
}