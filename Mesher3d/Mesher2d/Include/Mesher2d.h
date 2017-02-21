#pragma once

#include <unordered_set>
#include "Pslg2d.h"
#include "Defines2d.h"
#include "Triangulation.h"

namespace M2d
{
	class Vertex;
	class Edge;
	class Triangle;
	class TriangleGrid;

	class Mesher2d
	{
	public:
		Mesher2d(const Pslg& pslg)
			:mPslg(pslg), mTessGrid(nullptr) {}
		~Mesher2d();

		void createDelaunay2D();

		void getFacets(std::vector<GCore::Vec3i>& outFacets);

		void writeToOff(const char* filename)
		{
			mTess.writeToOff(filename);
		}

	private:

		void setUpGrid(const GCore::Box2d& box);
		Triangle* addTriangle(Vertex* v1, Vertex* v2, Vertex* v3);
		void removeTriangle(Triangle* triangle);

		void createSuperSquare();
		void inserVertices();

		void insert(const GCore::Vec2d& point);
		Triangle* locatePoint(const GCore::Vec2d& point);
		void findCircumscribingTriangles(const GCore::Vec2d& point, Triangleset& outTriangle);
		void findWrapperEdges(const Triangleset& triangles, Edgeset& outEdgeset);
		void removeTriangles(const Triangleset& triangles);
		void triangulateCavity(const GCore::Vec2d& point, const Edgeset& wrapperEdges, Triangleset& outNewTriangles);
		void deleteTriangles(const Triangleset& triangles);

		void recoverEdges();
		void recoverEdge(Vertex* v1, Vertex* v2);
		bool recoverEdgeWithFlip(Vertex* v1, Vertex* v2);
		void removeExternalTriangles();

		bool getBoundaryEdges(Edgeset& outBoundaryEdges);
		void getExteriorTriangle(const Edgeset& boundaryEdges, Triangleset& outExteriorTriangles);

	private:
		Pslg mPslg;
		Triangulation mTess;
		TriangleGrid* mTessGrid;
	};
}

