#pragma once

#include "Defines2d.h"
#include "Defines.h"
#include "Vec.h"

namespace M2d
{
	class Vertex
	{
	private:
		Vertex(GCore::Vec2d inPoint)
			:coord(inPoint){}
		~Vertex() = default;
	public:
		bool addEdge(Edge* edge)
		{
			return edges.emplace(edge).second;
		}
		bool removeEdge(Edge* edge)
		{
			return edges.erase(edge) > 0;
		}
	public:
		GCore::Vec2d coord;
		Edgeset edges;

		friend class Triangulation;
	};
}
