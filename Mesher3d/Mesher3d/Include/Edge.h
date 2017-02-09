#pragma once

#include <unordered_set>

#include "Defines3d.h"
#include "Defines.h"
#include "Vec.h"
#include "Vertex.h"

namespace M3d
{
	class Face;

	class Edge
	{
	private:
		Edge(Vertex* _v1, Vertex* _v2);
		~Edge()
		{
			vertex[0] = nullptr;
			vertex[1] = nullptr;
		}

		void dispose();

	public:

		bool isConnected();

		Vertex* otherVertex(const Vertex* inVertex);

		bool addFace(Face* face)
		{
			return faces.emplace(face).second;
		}

		bool removeFace(Face* face);

		GCore::Vec3d pointAt(double u, double v)
		{
			return vertex[0]->coord*u + vertex[1]->coord*v;
		}

		void getSurroundingTetras(Tetraset& outTetras);

		double length()
		{
			return (vertex[0]->coord - vertex[1]->coord).magnitude();
		}

	public:
		Vertex* vertex[2];
		std::unordered_set<Face*> faces;

		friend class Tessellation;
	};
}