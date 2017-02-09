#pragma once

#include "Defines.h"
#include "BBox.h"
#include "Circle.h"

class GCore::Circle;

namespace M2d
{
	class Vertex;
	class Edge;
	class  Triangle
	{
	public:

		//enum Position
		//{
		//	Inside,
		//	OnEdge,
		//	OnVertex,
		//	Outside
		//};

		static void findCircumCircle				( const GCore::Vec2d & p1,
													  const GCore::Vec2d & p2,
													  const GCore::Vec2d & p3,
													  GCore::Vec2d & outCenter, 
													  double & outRadius);
	private:
		Triangle(Vertex* v1, Vertex* v2, Vertex* v3, Edge* e1, Edge* e2, Edge* e3);
		~Triangle();

	public:

		bool					isVertex		( const Vertex* inVertex)const;
		bool					isEdge			( const Edge* inEdge)const;

		void					neighbours		( Triangle* neighbors[3]);

		double					angleAt			( const Vertex* inVertex)const;

		Edge*					opposite		( const Vertex* inVertex);
		const Edge*				opposite		( const Vertex* inVertex)const;

		Vertex*					opposite		( const Edge* inVertex);
		const Vertex*			opposite		( const Edge* inVertex)const;

		bool					inCircumcircle  (const GCore::Vec2d & inPoint)const;
		bool					inCircumcircle  (const GCore::Vec2d & inPoint);
		bool					isInside		( const GCore::Vec2d & inPoint)const;
		GCore::Vec2d			centroid		( )const;
		double					area			( )const;
		const Edge*				smallestEdge	( )const;
		const Edge*				largestEdge		( )const;
		double					smallestAngle	( )const;
		double					largestAngle	( )const;
		GCore::Box2d			boundingBox     ( )const;

	public:
		Edge* edge[3];
		Vertex*	vertex[3];
		GCore::Circle* circle;

	friend class Triangulation;
	};
}

