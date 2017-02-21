#pragma once

#include <unordered_map>
#include "Defines.h"
#include "BBox.h"
#include "PointInPolygon.h"
#include "Tessellation.h"

static int gStackCount = 0;
static int gFlipCount = 0;

namespace M3d
{
	class PointInsertion
	{
	public:
		PointInsertion(Tessellation& tess)
			:mTess(tess) {}

	public:

		bool insertWatson(const GCore::Vec3d& point);
		bool insertLawson(const GCore::Vec3d& point);

	private:
		// laswson
		void tryInsertPoint(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInTetra(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInFace(Face* face, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInEdge(Edge* edge, const GCore::Vec3d& point, Tetraset& outTetras);
		void flip(Face* face, const Edgeset& reflexEdges, Tetraset& outTetras, Faceset& deletedTetras);
		void flip14(const GCore::Vec3d& point, Tetra* tetra, Tetraset& outTetras, Faceset& deletedFaces);
		void flip44(Face* face, Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		void flip23(Face* face, Tetraset& outTetras, Faceset& deletedTetras);
		void flip32(Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		void flip41(/*Face* face, Tetraset& outTetras, Tetraset& deletedTetras*/);

	private:
		Tessellation& mTess;
	};
}
