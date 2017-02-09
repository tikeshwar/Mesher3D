#pragma once

#include "Defines3d.h"
#include "Defines.h"
#include "Vec.h"
#include "Grid3D.h"

namespace M3d
{
	class Edge;
	class EdgeGrid : public Grid3D<Edge*>
	{
	public:
		typedef Grid3D<Edge*> EdgeGrid3d;

	public:
		EdgeGrid(const GCore::Vec3d& start, const GCore::Vec3i& div, const GCore::Vec3d& delta)
			:EdgeGrid3d(start, div, delta) {}

		void emplace(Edge* edge);
		void erase(Edge* edge);

		Edge* findClosest(const GCore::Vec3d& point, const Edgeset& edgesNotToConsider)const;
		void getEdges(size_t li, size_t lj, size_t lk,
			size_t ui, size_t uj, size_t uk, Edgeset & outEdges)const;

	};
}
