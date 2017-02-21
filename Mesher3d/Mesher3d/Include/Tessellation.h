#pragma once

#include <unordered_set>
#include <set>
#include "Defines.h"
#include "Vec.h"
#include "Sphere.h"
#include "TetraGrid.h"
#include "Vertex.h"

namespace M3d
{
#define PRIME2d 2147483648

	class Tetra;
	class Face;
	class Edge;
	class Vertex;

	class Tessellation
	{
	private:
		struct VertexHash {
			size_t operator() (const Vertex* v) const {
				return size_t(v->coord.x*PRIME2d*PRIME2d + v->coord.y*PRIME2d + v->coord.z);
			}
		};

		struct VertexEqual {
			size_t operator() (const Vertex* v1, const Vertex* v2) const {
				return v1->coord == v2->coord;
			}
		};

		struct VertexLess {
			bool operator() (const Vertex* v1, const Vertex* v2) const {
				if (!GCore::isEqual(v1->coord.x, v2->coord.x))
					return v1->coord.x < v2->coord.x;
				else if (!GCore::isEqual(v1->coord.y, v2->coord.y))
					return v1->coord.y < v2->coord.y;
				else if (!GCore::isEqual(v1->coord.z, v2->coord.z))
					return v1->coord.z < v2->coord.z;
				else
					return false;
			}
		};

	public:

		//typedef std::unordered_set<Vertex*, VertexHash, VertexEqual> UniqueVertexset;
		typedef std::set<Vertex*, VertexLess> UniqueVertexset;

		Tessellation() = default;
		~Tessellation() = default;

		void setUpGrid(const GCore::Vec3d& start, const GCore::Vec3i& div, const GCore::Vec3d& delta);

		Vertex* addVertex(const GCore::Vec3d& vec3d);
		Edge* addEdge(Vertex* vertex1, Vertex* vertex2);
		Face* addFace(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3);
		Tetra* addTetra(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Vertex* vertex4);

		Edge* addEdge(const GCore::Vec3d& vec1, const GCore::Vec3d& vec2);
		Face* addFace(const GCore::Vec3d& vec1, const GCore::Vec3d& vec2, const GCore::Vec3d& vec3);
		Tetra* addTetra(const GCore::Vec3d& vec1, const GCore::Vec3d& vec2, const GCore::Vec3d& vec3, const GCore::Vec3d& vec4);

		//bool removeVertex(Vertex* vertex);
		//bool removeEdge(Edge* edge);
		//bool removeFace(Face* face);
		//bool removeTetra(Tetra* tetra);

		bool deleteVertex(Vertex* vertex);
		bool deleteEdge(Edge* edge);
		bool deleteFace(Face* face);
		bool deleteTetra(Tetra* tetra);

		Vertex* findVertex(const GCore::Vec3d& v);
		Edge* findEdge(const GCore::Vec3d& v1, const GCore::Vec3d& v2);
		Face* findFace(const GCore::Vec3d& v1, const GCore::Vec3d& v2, const GCore::Vec3d& v3);
		Tetra* findTetra(const GCore::Vec3d& v1, const GCore::Vec3d& v2, const GCore::Vec3d& v3, const GCore::Vec3d& v4);

		Edge* findEdge(Vertex* vertex1, Vertex* vertex2);
		Face* findFace(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3);
		Tetra* findTetra(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Vertex* vertex4);

		Tetra* locateTetra(const GCore::Vec3d& point)const
		{
			return mGrid->find(point);
		}

		void locatePoints(const GCore::Sphere& sphere, Vertexset& outVertexset);
		void locateTetras(const GCore::Sphere& sphere, Tetraset& outTetraset);

		const Tetraset& getTetras()const
		{
			return mTetraset;
		}
		const Faceset& getFaces()const
		{
			return mFaceset;
		}
		const Edgeset& getEdges()const
		{
			return mEdgeset;
		}
		const UniqueVertexset& getVertices()const
		{
			return mVertexset;
		}
		GCore::Box3d boundingBox()
		{
			return mBBox;
		}

		void clearTessellation();

		void writeToOff(const char* filname)const;

	private:
		Tetraset mTetraset;
		Faceset mFaceset;
		Edgeset mEdgeset;
		UniqueVertexset mVertexset;
		GCore::Box3d mBBox;
		TetraGrid* mGrid;
	};
}
