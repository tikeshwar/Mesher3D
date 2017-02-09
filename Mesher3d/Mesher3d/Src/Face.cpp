#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Tetra.h"

using namespace GCore;
using namespace M3d;

Face::Face(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3,
	Edge* edge1, Edge* edge2, Edge* edge3)
{
	vertex[0] = vertex1;
	vertex[1] = vertex2;
	vertex[2] = vertex3;

	edge[0] = edge1;
	edge[1] = edge2;
	edge[2] = edge3;

	for (int i = 0; i < 3; i++)
		edge[i]->addFace(this);

	tetra[0] = nullptr;
	tetra[1] = nullptr;

	normal = (vertex[0]->coord - vertex[1]->coord).cross(vertex[1]->coord - vertex[2]->coord);
	normal.normalize();
}

Face::~Face()
{
	vertex[0] = nullptr;
	vertex[1] = nullptr;
	vertex[2] = nullptr;

	edge[0] = nullptr;
	edge[1] = nullptr;
	edge[2] = nullptr;

	tetra[0] = nullptr;
	tetra[1] = nullptr;
}

void Face::dispose()
{
	for (int i = 0; i < 3; i++)
		if (edge[i])
			edge[i]->removeFace(this);
}

bool Face::isConnected()
{
	return tetra[0] != nullptr || tetra[1] != nullptr;
}

Edge* Face::opposite(Vertex* inVertex)
{
	for (int i = 0; i < 3; i++)
	{
		if (vertex[i] == inVertex)
			return edge[i];
	}
	return nullptr;
}

Vertex* Face::opposite(Edge* inEdge)
{
	for (int i = 0; i < 3; i++)
	{
		if (edge[i] == inEdge)
			return vertex[i];
	}
	return nullptr;
}

Tetra* Face::otherTetra(Tetra * inTetra)
{
	if (inTetra == tetra[0])
		return tetra[1];
	else if (inTetra == tetra[1])
		return tetra[0];
	return nullptr;
}


bool Face::addTetra(Tetra* inTetra)
{
	if (nullptr == tetra[0])
	{
		tetra[0] = inTetra;
		return true;
	}
	else if (nullptr == tetra[1])
	{
		tetra[1] = inTetra;
		return true;
	}
	return false;
}

bool Face::removeTetra(Tetra* inTetra)
{
	if (inTetra == tetra[0])
	{
		tetra[0] = nullptr;
		return true;
	}
	else if (inTetra == tetra[1])
	{
		tetra[1] = nullptr;
		return true;
	}
	return false;
}

double Face::distanceTo(const Vec3d& point, const Tetra* refTetra)const
{
	GCore::Vec3d faceNorm = normal;

	Vec3d tcentroid = refTetra->centroid();
	Vec3d fcentroid = centroid();

	if (GCore::isNegative(faceNorm.dot(fcentroid.direction(tcentroid))))
		faceNorm.set(-faceNorm.x, -faceNorm.y, -faceNorm.z);
	return normal.dot(point - vertex[0]->coord);
}

GCore::Vec3d Face::refNormal(const GCore::Vec3d& refPoint)const
{
	GCore::Vec3d faceNorm = normal.normalize();
	Vec3d fcentroid = centroid();
	GCore::Vec3d refNorm = fcentroid.direction(refPoint);
	if (GCore::isNegative(refNorm.dot(faceNorm)))
		faceNorm.set(-faceNorm.x, -faceNorm.y, -faceNorm.z);
	return faceNorm;
}