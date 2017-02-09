#pragma once

#include <unordered_set>
#include "Defines.h"
#include "Defines3d.h"
#include "Vec.h"

namespace M3d
{
	class Edge;
	class Vertex
	{
	private:
		Vertex(const GCore::Vec3d& _coord)
			:coord(_coord) {}

		~Vertex() = default;

	public:
		bool isConnected();

		bool addEdge(Edge* edge)
		{
			return edges.emplace(edge).second;
		}

		bool removeEdge(Edge* edge)
		{
			return edges.erase(edge) > 0;
		}

		bool isAcute(double angleRad, Edgeset& outEdges);
		Edge* shortestEdge();

	public:
		GCore::Vec3d coord;
		std::unordered_set<Edge*> edges;

		friend class Tessellation;
	};
}