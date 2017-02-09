#pragma once

#include <vector>
#include "Defines.h"
#include "BBox.h"

namespace M2d
{
	class Pslg
	{
	public:
		typedef std::vector<GCore::Vec2d> PointList;
		typedef std::vector<GCore::Vec2i> SegmentList;

	public:
		Pslg() = default;
		Pslg(const PointList& pointlist, const SegmentList& boundarySegs, const SegmentList& constraintSegs, const PointList& holePts)
			:points(pointlist), boundarySegments(boundarySegs), constraintSegments(constraintSegs), holePoints(holePts) 
		{
			calculateBBox();
		}
		~Pslg() = default;
	
		void calculateBBox()
		{
			for (auto it = points.begin(); it != points.end(); it++)
				box.extend(*it);
		}

	public:
		PointList points;
		SegmentList boundarySegments;
		SegmentList constraintSegments;
		PointList holePoints;

		GCore::Box2d box;
	};
}
