#pragma once

#include <unordered_set>
#include <unordered_map>
#include "Defines.h"
#include "Vec.h"
#include "Vertex.h"

namespace M2d
{
	class Entity;
	class Vertex;
	class Edge;
	class Triangle;
	class  Triangulation
	{
	private:
		struct VertexHash {
			size_t operator() (const Vertex * v) const {
				return hash(v->coord);
			}
		};
		struct VertexComparator {
			bool operator()(const Vertex* a, const Vertex* b) const
			{
				return equal(a, b);
			}
		};

	public:
		typedef std::unordered_set<Vertex*, VertexHash, VertexComparator> UniqueVertexset;

	public:
		Triangulation() = default;
		~Triangulation();

		Triangle* addTriangle(Vertex* v1, Vertex* v2, Vertex* v3);
		Edge* addEdge(Vertex* v1, Vertex* v2);
		Vertex* addVertex(const GCore::Vec2d& Vec2d);

		Triangle* findTriangle(Vertex* v1, Vertex* v2, Vertex* v3);
		Edge* findEdge(Vertex* v1, Vertex* v2);
		Vertex* findVertex(const GCore::Vec2d& Vec2d);

		bool removeTriangle(Triangle* triangle);
		bool removeEdge(Edge* edge);
		bool removeVertex(Vertex* vertex);

		void deleteTriangle(Triangle* triangle);
		void deleteEdge(Edge* edge);
		void deleteVertex(Vertex* vertex);

		const Triangleset& triangles()const
		{
			return mTriangleset;
		}
		const Edgeset& edges()const
		{
			return mEdgeset;
		}
		const UniqueVertexset& vertices()const
		{
			return mVertexset;
		}

		const GCore::Box2d& boundingBox()const
		{
			return mBox;
		}
		GCore::Box2d boundingBox()
		{
			return mBox;
		}

		void getFacets(std::vector<GCore::Vec3i>& outFacets);
		void writeToOff(const char* filname);

	private:
		static bool equal(const Vertex* v1, const Vertex* v2);
		static bool equal(const Edge* e1, const Edge* e2);
		static bool equal(const Triangle* t1, const Triangle* t2);

		static size_t hash(const GCore::Vec2d & point);
		static bool	isCounterClockwise(const GCore::Vec2d & inPoint1,
			const GCore::Vec2d & inPoint2,
			const GCore::Vec2d & inPoint3);

	private:
		Triangleset mTriangleset;
		Edgeset mEdgeset;
		UniqueVertexset mVertexset;

		GCore::Box2d mBox;
	};
}
