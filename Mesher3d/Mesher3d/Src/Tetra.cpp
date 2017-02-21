#include <cassert>
#include "Defines.h"
#include "Vec.h"
#include "GeomTest.h"
#include "Face.h"
#include "Vertex.h"
#include "Edge.h"
#include "Tetra.h"

using namespace GCore;
using namespace M3d;

Tetra::Tetra(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Vertex* vertex4,
	Face* face1, Face* face2, Face* face3, Face* face4)
{
	vertex[0] = vertex1;
	vertex[1] = vertex2;
	vertex[2] = vertex3;
	vertex[3] = vertex4;

	face[0] = face1;
	face[1] = face2;
	face[2] = face3;
	face[3] = face4;

	for (int i = 0; i < 4; i++)
	{
		if (face[i])
			face[i]->addTetra(this);
	}

	for (int i = 0; i < 4; i++)
		box.extend(vertex[i]->coord);

	//circumSphere = GCore::Sphere(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord, vertex[3]->coord);
}

Tetra::~Tetra()
{
	for (size_t i = 0; i < 4; i++)
		vertex[i] = nullptr;

	for (size_t i = 0; i < 4; i++)
		face[i] = nullptr;
}

void Tetra::dispose()
{
	for (size_t i = 0; i < 4; i++)
	{
		if (face[i])
			face[i]->removeTetra(this);
	}
}

bool Tetra::contains(const Vec3d& point)const
{
	//http://people.sc.fsu.edu/~jburkardt/presentations/cg_lab_barycentric_tetrahedrons.pdf

	double b1 = face[0]->distanceTo(point, this) / face[0]->distanceTo(vertex[0]->coord, this);
	if (!GCore::isPositive(b1))
		return false;
	double b2 = face[1]->distanceTo(point, this) / face[1]->distanceTo(vertex[1]->coord, this);
	if (!GCore::isPositive(b2))
		return false;
	double b3 = face[2]->distanceTo(point, this) / face[2]->distanceTo(vertex[2]->coord, this);
	if (!GCore::isPositive(b3))
		return false;
	double b4 = face[3]->distanceTo(point, this) / face[3]->distanceTo(vertex[3]->coord, this);
	if (!GCore::isPositive(b4))
		return false;
	return isEqual(b1 + b2 + b3 + b4, 1.0);
}

std::vector<Tetra*>&& Tetra::neighborTetras()
{
	std::vector<Tetra*> neighbors(4);
	for (size_t i = 0; i < neighbors.size(); i++)
		neighbors[i] = face[i]->otherTetra(this);

	return std::move(neighbors);
}

Face* Tetra::opposite(Vertex* inVertex)
{
	for (size_t i = 0; i < 4; i++)
	{
		if (inVertex == vertex[i])
			return face[i];
	}
	return nullptr;
}

Vertex* Tetra::opposite(Face* inFace)
{
	for (size_t i = 0; i < 4; i++)
	{
		if (inFace == face[i])
			return vertex[i];
	}
	return nullptr;
}

double Tetra::volume()const
{
	return GeomTest::signedVolume(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord, vertex[3]->coord);
}

bool Tetra::intersects(const Vec3d& norm, const Vec3d& point)const
{
	int sign = 0;
	for (int i = 0; i < 4; i++)
	{
		if ((point - vertex[i]->coord).dot(norm) > 0)
			sign++;
	}

	return (sign > 0 && sign < 4) || sign == 0;
}

GeomTest::Pos Tetra::isInsideCircumsphere(const GCore::Vec3d& point)const
{
	double o3d = GeomTest::orient3d(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord, vertex[3]->coord);
	double insphere = GeomTest::inSphere(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord, vertex[3]->coord, point);
	if (GCore::isZero(o3d))
	{
		double A = GeomTest::unSignedArea3d(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord);
		double A1 = GeomTest::unSignedArea3d(vertex[0]->coord, vertex[1]->coord, vertex[3]->coord);
		double A2 = GeomTest::unSignedArea3d(vertex[1]->coord, vertex[2]->coord, vertex[3]->coord);
		double A3 = GeomTest::unSignedArea3d(vertex[2]->coord, vertex[0]->coord, vertex[3]->coord);

		if (GCore::isEqual(A, A1 + A2 + A3))
			return GeomTest::kInside;
		return GeomTest::kOutside;
	}
	if (GCore::isZero(insphere))
		return GeomTest::kOnBoundary;
	else if (!(GCore::isPositive(insphere) ^ GCore::isPositive(o3d)))
		return GeomTest::kInside;
	else
		return GeomTest::kOutside;
}

void Tetra::getSurroundingTetras(Tetraset& outTetras)
{
	for (int i = 0; i < 4; i++)
	{
		Tetra* otherTera = face[i]->otherTetra(this);
		if (otherTera)
			outTetras.emplace(otherTera);
	}
}

