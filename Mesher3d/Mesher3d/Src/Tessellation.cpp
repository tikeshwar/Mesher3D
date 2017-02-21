#include <fstream>
#include <cassert>
#include <unordered_map>
#include "Defines.h"
#include "Vec.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Tetra.h"
#include "Tessellation.h"

using namespace GCore;
using namespace M3d;


void Tessellation::setUpGrid(const Vec3d & start, const Vec3i & div, const Vec3d & delta)
{
	mGrid = new TetraGrid(start, div, delta);
}

Vertex* Tessellation::addVertex(const Vec3d& vec3d)
{
	mBBox.extend(vec3d);

	Vertex* vertex = new Vertex(vec3d);
	auto it = mVertexset.emplace(vertex);
	if (!it.second)
		delete vertex;
	return *(it.first);
}

Edge* Tessellation::addEdge(Vertex* vertex1, Vertex* vertex2)
{
	for (auto it = vertex1->edges.begin(); it != vertex1->edges.end(); it++)
		if (vertex2->edges.find(*it) != vertex2->edges.end())
			return *it;

	Edge* edge = new Edge(vertex1, vertex2);
	mEdgeset.emplace(edge);
	return edge;
}

Face* Tessellation::addFace(Vertex * vertex1, Vertex * vertex2, Vertex * vertex3)
{
	Edge* edge1 = addEdge(vertex1, vertex2);
	Edge* edge2 = addEdge(vertex1, vertex3);
	Edge* edge3 = addEdge(vertex2, vertex3);

	for (auto it = edge1->faces.begin(); it != edge1->faces.end(); it++)
		if (edge2->faces.find(*it) != edge2->faces.end() &&
			edge3->faces.find(*it) != edge3->faces.end())
			return *it;

	Face* face = new Face(vertex1, vertex2, vertex3, edge3, edge2, edge1);
	mFaceset.emplace(face);
	return face;
}

Tetra* Tessellation::addTetra(Vertex * vertex1, Vertex * vertex2, Vertex * vertex3, Vertex * vertex4)
{
	//double vol = GeomTest::signedVolume(vertex1->coord, vertex2->coord, vertex3->coord, vertex4->coord);
	//if (GCore::isZero(vol))
	//	return nullptr;

	Face* face1 = addFace(vertex1, vertex2, vertex3);
	Face* face2 = addFace(vertex1, vertex2, vertex4);
	Face* face3 = addFace(vertex1, vertex3, vertex4);
	Face* face4 = addFace(vertex2, vertex3, vertex4);

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				for (int l = 0; l < 2; l++)
					if (face1->tetra[i] && face2->tetra[j] && face3->tetra[k] && face4->tetra[l]
						&& face1->tetra[i] == face2->tetra[j] && face2->tetra[j] == face3->tetra[k]
						&& face3->tetra[k] == face4->tetra[l])
						return face1->tetra[i];

#ifndef _DEBUG
	if ((face1->tetra[0] != nullptr && face1->tetra[1] != nullptr))
		return nullptr;
	if ((face2->tetra[0] != nullptr && face2->tetra[1] != nullptr))
		return nullptr;
	if ((face3->tetra[0] != nullptr && face3->tetra[1] != nullptr))
		return nullptr;
	if ((face4->tetra[0] != nullptr && face4->tetra[1] != nullptr))
		return nullptr;
#else
	assert(!(face1->tetra[0] != nullptr && face1->tetra[1] != nullptr));
	assert(!(face2->tetra[0] != nullptr && face2->tetra[1] != nullptr));
	assert(!(face3->tetra[0] != nullptr && face3->tetra[1] != nullptr));
	assert(!(face4->tetra[0] != nullptr && face4->tetra[1] != nullptr));
#endif

	Tetra* tetra = new Tetra(vertex1, vertex2, vertex3, vertex4, face4, face3, face2, face1);
	mTetraset.emplace(tetra);
	if (mGrid)
		mGrid->emplace(tetra);
	return tetra;

}

Edge* Tessellation::addEdge(const Vec3d& vec1, const Vec3d& vec2)
{
	Vertex* v1 = addVertex(vec1);
	Vertex* v2 = addVertex(vec2);

	return addEdge(v1, v2);
}

Face* Tessellation::addFace(const Vec3d& vec1, const Vec3d& vec2, const Vec3d& vec3)
{
	Vertex* v1 = addVertex(vec1);
	Vertex* v2 = addVertex(vec2);
	Vertex* v3 = addVertex(vec3);

	return addFace(v1, v2, v3);
}

Tetra* Tessellation::addTetra(const Vec3d& vec1, const Vec3d& vec2, const Vec3d& vec3, const Vec3d& vec4)
{
	Vertex* v1 = addVertex(vec1);
	Vertex* v2 = addVertex(vec2);
	Vertex* v3 = addVertex(vec3);
	Vertex* v4 = addVertex(vec4);

	return addTetra(v1, v2, v3, v4);
}

//bool Tessellation::removeVertex(Vertex* vertex)
//{
//	if (vertex->edges.empty())
//	{
//		mVertexset.erase(vertex);
//		return true;
//	}
//	return false;
//}
//
//bool Tessellation::removeEdge(Edge* edge)
//{
//	if (edge->faces.empty())
//	{
//		edge->dispose();
//		mEdgeset.erase(edge);
//		return true;
//	}
//	return false;
//}
//
//bool Tessellation::removeFace(Face* face)
//{
//	if (face->tetra[0] == nullptr && face->tetra[1] == nullptr)
//	{
//		face->dispose();
//		mFaceset.erase(face);
//		return true;
//	}
//	return false;
//}
//
//bool Tessellation::removeTetra(Tetra* tetra)
//{
//	tetra->dispose();
//	mTetraset.erase(tetra);
//	if (mGrid)
//		mGrid->erase(tetra);
//	return true;
//}

bool Tessellation::deleteVertex(Vertex* vertex)
{
	if (!vertex->isConnected())
	{
		mVertexset.erase(vertex);
		delete vertex;
		return true;
	}
	return false;
}

bool Tessellation::deleteEdge(Edge* edge)
{
	if (!edge->isConnected())
	{
		Vertex* vertices[2] = { edge->vertex[0], edge->vertex[1] };
		for (size_t i = 0; i < 2; i++)
			if (vertices[i])
				assert(vertices[i]->removeEdge(edge));

		mEdgeset.erase(edge);
		delete edge;

		for (size_t i = 0; i < 2; i++)
			if (vertices[i] && !vertices[i]->isConnected())
				deleteVertex(vertices[i]);

		return true;
	}
	return false;
}

bool Tessellation::deleteFace(Face* face)
{
	if (!face->isConnected())
	{
		Edge* edges[3] = { face->edge[0], face->edge[1], face->edge[2] };
		for (size_t i = 0; i < 3; i++)
			if (edges[i])
				assert(edges[i]->removeFace(face));

		mFaceset.erase(face);
		delete face;

		for (size_t i = 0; i < 3; i++)
		{
			if (edges[i] && !edges[i]->isConnected())
				deleteEdge(edges[i]);
		}
		return true;
	}
	return false;
}

bool Tessellation::deleteTetra(Tetra* tetra)
{
	if (tetra)
	{
		Face* faces[4] = { tetra->face[0], tetra->face[1], tetra->face[2], tetra->face[3] };
		for (size_t i = 0; i < 4; i++)
			if (faces[i])
				assert(faces[i]->removeTetra(tetra));

		mTetraset.erase(tetra);
		mGrid->erase(tetra);
		delete tetra;

		for (size_t i = 0; i < 4; i++)
		{
			if (faces[i] && !faces[i]->isConnected())
				deleteFace(faces[i]);
		}
		return true;
	}
	return false;
}

Vertex* Tessellation::findVertex(const Vec3d& v)
{
	Vertex* vertex = new Vertex(v);
	auto it = mVertexset.find(vertex);
	delete vertex;

	if (it != mVertexset.end())
		return *it;
	else
		return nullptr;
}

Edge* Tessellation::findEdge(const Vec3d& v1, const Vec3d& v2)
{
	Vertex* vertex1 = findVertex(v1);
	Vertex* vertex2 = findVertex(v2);

	return findEdge(vertex1, vertex2);
}

Face* Tessellation::findFace(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3)
{
	Vertex* vertex1 = findVertex(v1);
	Vertex* vertex2 = findVertex(v2);
	Vertex* vertex3 = findVertex(v3);

	return findFace(vertex1, vertex2, vertex3);
}

Tetra* Tessellation::findTetra(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3, const Vec3d& v4)
{
	Vertex* vertex1 = findVertex(v1);
	Vertex* vertex2 = findVertex(v2);
	Vertex* vertex3 = findVertex(v3);
	Vertex* vertex4 = findVertex(v4);

	return findTetra(vertex1, vertex2, vertex3, vertex4);
}

Edge* Tessellation::findEdge(Vertex* vertex1, Vertex* vertex2)
{
	if (vertex1 == nullptr || vertex2 == nullptr)
		return nullptr;

	if (vertex1 == vertex2)
		return nullptr;

	for (auto it = vertex1->edges.begin(); it != vertex1->edges.end(); it++)
	{
		if (vertex2->edges.find(*it) != vertex2->edges.end())
			return *it;
	}
	return nullptr;
}
Face* Tessellation::findFace(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3)
{
	Edge* edge1 = findEdge(vertex1, vertex2);
	if (!edge1)
		return nullptr;

	Edge* edge2 = findEdge(vertex2, vertex3);
	if (!edge2)
		return nullptr;

	Edge* edge3 = findEdge(vertex3, vertex1);
	if (!edge3)
		return nullptr;

	if (edge1 == edge2 || edge2 == edge3)
		return nullptr;

	for (auto it = edge1->faces.begin(); it != edge1->faces.end(); it++)
	{
		if (edge2->faces.find(*it) != edge2->faces.end() &&
			edge3->faces.find(*it) != edge3->faces.end())
			return *it;
	}
	return nullptr;
}
Tetra* Tessellation::findTetra(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Vertex* vertex4)
{
	Face* face1 = findFace(vertex1, vertex2, vertex3);
	if (!face1)
		return nullptr;

	Face* face2 = findFace(vertex2, vertex3, vertex4);
	if (!face2)
		return nullptr;

	Face* face3 = findFace(vertex3, vertex4, vertex1);
	if (!face3)
		return nullptr;

	Face* face4 = findFace(vertex4, vertex1, vertex2);
	if (!face4)
		return nullptr;

	if (face1 == face2 || face2 == face3 || face3 == face4)
		return nullptr;

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				for (int l = 0; l < 2; l++)
				{
					if (face1->tetra[i] && face2->tetra[j] && face3->tetra[k] && face4->tetra[l]
						&& face1->tetra[i] == face2->tetra[j] && face2->tetra[j] == face3->tetra[k]
						&& face3->tetra[k] == face4->tetra[l])
						return face1->tetra[i];
				}
	}
	return nullptr;
}

void Tessellation::clearTessellation()
{
	for (auto it = mFaceset.begin(); it != mFaceset.end(); it++)
		if (!(*it)->isConnected())
		{
			deleteFace(*it);
		}

	for (auto it = mTetraset.begin(); it != mTetraset.end(); it++)
		if (isZero((*it)->volume()))
		{
			deleteTetra(*it);
			it = mTetraset.begin();
		}
}

void Tessellation::locatePoints(const GCore::Sphere& sphere, Vertexset& outVertexset)
{
	Tetraset outTetras;
	locateTetras(sphere, outTetras);

	for (auto it = outTetras.begin(); it != outTetras.end(); it++)
	{
		for (int v = 0; v < 4; v++)
		{
			if (sphere.inside((*it)->vertex[v]->coord))
				outVertexset.emplace((*it)->vertex[v]);
		}
	}
}

void Tessellation::locateTetras(const GCore::Sphere& sphere, Tetraset& outTetraset)
{
	Box3d box = sphere.boundingBox();
	Vec3i lijk = mGrid->getIJK(box.lower);
	Vec3i uijk = mGrid->getIJK(box.upper);

	Tetraset outTetras;
	mGrid->getTetras(lijk.x, lijk.y, lijk.z, uijk.x, uijk.y, uijk.z, outTetras);

	for (auto it = outTetras.begin(); it != outTetras.end(); it++)
		outTetraset.emplace(*it);
}

void Tessellation::writeToOff(const char* filname)const
{
	std::ofstream file;
	file.open(filname);

	file << "OFF" << std::endl;
	file << mVertexset.size() << " " << mFaceset.size() << " 0" << std::endl;

	std::unordered_map<Vertex*, int> v2iMap;

	int v = 0;
	for (auto const vertex : mVertexset)
	{
		file << vertex->coord.x << " " << vertex->coord.y << " " << vertex->coord.z << std::endl;
		v2iMap.emplace(vertex, v++);
	}

	for (auto const face : mFaceset)
	{
		file << "3 "
			<< v2iMap[face->edge[0]->vertex[0]] << " "
			<< v2iMap[face->edge[0]->vertex[1]] << " "
			<< v2iMap[face->opposite(face->edge[0])]
			<< std::endl;

	}
}
