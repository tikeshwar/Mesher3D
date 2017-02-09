#pragma once

#include <vector>
#include "Defines.h"
#include "BBox.h"

namespace M3d
{
	class Pslg
	{
	public:
		typedef std::vector<GCore::Vec3d> PointList;
		typedef std::vector<GCore::Vec3d> SegmentList;
		typedef std::vector<size_t> Polygon;
		typedef std::vector<Polygon> FaceList;

	public:
		Pslg() = default;
		Pslg(const PointList& pointlist, const FaceList& boundaryFacets, const FaceList& constraintFacets,
			const SegmentList& constraintSegs, const PointList& holePts)
			:points(pointlist), boundaryFaces(boundaryFacets), constraintFaces(constraintFacets),
			constraintSegments(constraintSegs), holePoints(holePts)
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
		FaceList boundaryFaces;
		FaceList constraintFaces;
		SegmentList constraintSegments;
		PointList holePoints;

		GCore::Box3d box;
	};
}
