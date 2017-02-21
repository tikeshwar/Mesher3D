#pragma once

#include <vector>
#include "Sphere.h"
#include "BBox.h"
#include "GeomTest.h"
#include "Vertex.h"

class GCore::Sphere;

namespace M3d
{
	class Face;
	class Tetra
	{
	private:
		Tetra(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Vertex* vertex4,
			Face* face1, Face* face2, Face* face3, Face* face4);
		~Tetra();

		void dispose();

	public:
		bool contains(const GCore::Vec3d& point)const;
		std::vector<Tetra*>&& neighborTetras();

		Face* opposite(Vertex* inVertex);
		Vertex* opposite(Face* inFace);

		GCore::Vec3d centroid()const
		{
			return (vertex[0]->coord + vertex[1]->coord + vertex[2]->coord + vertex[3]->coord)*0.25;
		}

		GCore::Vec3d pointAt(double u, double v, double w, double x)const
		{
			return vertex[0]->coord*u + vertex[1]->coord*v + vertex[2]->coord*w + vertex[3]->coord*x;
		}

		double volume()const;

		bool intersects(const GCore::Vec3d& norm, const GCore::Vec3d& point)const;
		
		GCore::GeomTest::Pos isInsideCircumsphere(const GCore::Vec3d& point)const;

		void getSurroundingTetras(Tetraset& outTetras);

	public:
		Vertex* vertex[4];
		Face *face[4];
		//GCore::Sphere circumSphere;
		GCore::Box3d box;
		friend class Tessellation;
	};
}