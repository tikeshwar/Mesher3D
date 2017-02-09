#pragma once

#include "Defines2d.h"
#include "Vertex.h"

namespace M2d
{
	class Triangle;
	class Edge
	{
	private:
		Edge(Vertex* v1, Vertex* v2);
		~Edge() {};

	public:

		Vertex*					otherVertex(const Vertex* inVertex);
		const Vertex*			otherVertex(const Vertex* inVertex)const;

		Triangle*				otherTriangle(Triangle*);
		const Triangle*			otherTriangle(const Triangle*)const;

		bool					addTriangle(Triangle* inTriangle);
		bool					removeTriangle(Triangle* inTriangle);
		bool					removeGhostTriangle(Triangle* inTriangle);

		bool					isEndPoint(const Vertex* inVertex)const
		{
			return (vertex[0] == inVertex || vertex[1] == inVertex);
		}
		bool					isNeighbour(const Triangle* inTriangle)const
		{
			return (triangle[0] == inTriangle || triangle[1] == inTriangle);
		}
		double					length()const
		{
			return (vertex[0]->coord - vertex[1]->coord).magnitude();
		}
		double					lengthSqr()const
		{
			return (vertex[0]->coord - vertex[1]->coord).magnitudeSqr();
		}
		GCore::Vec2d				midPoint()const
		{
			return (vertex[0]->coord + vertex[1]->coord) / 2.0;
		}
		double					orientation(const GCore::Vec2d& point)
		{
			return (vertex[0]->coord.x - point.x)*(vertex[1]->coord.y - point.y) -
				(vertex[0]->coord.y - point.y)*(vertex[1]->coord.x - point.x);
				
		}
	public:
		Triangle* triangle[2];			//!<neighbour triangle set.
		Vertex*	vertex[2];
		Triangle* ghostTriangle[2];

		friend class Triangulation;
	};
}

