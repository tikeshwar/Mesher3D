#pragma once

#include "Defines.h"
#include "Vec.h"
#include "Vertex.h"

namespace M3d
{
	class Edge;
	class Tetra;
	class Face
	{
	private:
		Face(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3,
			Edge* edge1, Edge* edge2, Edge* edge3);
		~Face();

		void dispose();

	public:
		bool isConnected();

		Edge* opposite(Vertex* inVertex);
		Vertex* opposite(Edge* inEdge);

		Tetra* otherTetra(Tetra* tetra);

		bool addTetra(Tetra* inTetra);
		bool removeTetra(Tetra* inTetra);

		double distanceTo(const GCore::Vec3d& point, const Tetra* refRetra)const;

		GCore::Vec3d centroid()const
		{
			return (vertex[0]->coord + vertex[1]->coord + vertex[2]->coord) / 3.0;
		}

		GCore::Vec3d pointAt(double u, double v, double w)const
		{
			return vertex[0]->coord*u + vertex[1]->coord*v + vertex[3]->coord*w;
		}

		GCore::Vec3d refNormal(const GCore::Vec3d& refPoint)const;

	public:
		Vertex* vertex[3];
		Edge *edge[3];
		Tetra* tetra[2];

		GCore::Vec3d normal;

		friend class Tessellation;
	};
}
