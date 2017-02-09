#include <fstream>
#include "Triangle.h"
#include "Edge.h"
#include "Vertex.h"
#include "Triangulation.h"

using namespace std;
using namespace M2d;

Triangulation::~Triangulation()
{
	while (!mTriangleset.empty())
	{
		deleteTriangle(*mTriangleset.begin());
	}
}

Triangle* Triangulation::addTriangle(Vertex* v1, Vertex* v2, Vertex* v3)
{
	Triangle* triangle = findTriangle(v1, v2, v3);
	if (!triangle)
	{
		Edge* edge1 = addEdge(v1, v2);
		Edge* edge2 = addEdge(v1, v3);
		Edge* edge3 = addEdge(v2, v3);

		triangle = new Triangle(v3, v2, v1, edge1, edge2, edge3);

		edge1->addTriangle(triangle);
		edge2->addTriangle(triangle);
		edge3->addTriangle(triangle);

		mTriangleset.emplace(triangle);
	}
	return triangle;
}
Edge* Triangulation::addEdge(Vertex* v1, Vertex* v2)
{
	Edge* edge = findEdge(v1, v2);
	if (!edge)
	{
		edge = new Edge(v1, v2);
		v1->addEdge(edge);
		v2->addEdge(edge);
		mEdgeset.emplace(edge);
	}
	return edge;
}
Vertex* Triangulation::addVertex(const GCore::Vec2d& Vec2d)
{
	Vertex* vertex = new Vertex(Vec2d);
	auto it = mVertexset.find(vertex);
	if (it != mVertexset.end())
	{
		delete vertex;
		return *it;
	}
	mBox.extend(Vec2d);
	return *mVertexset.emplace(vertex).first;
}

bool Triangulation::removeTriangle(Triangle* triangle)
{
	for (int i = 0; i < 3; i++)
		triangle->edge[i]->removeTriangle(triangle);
	mTriangleset.erase(triangle);
	return true;
}
bool Triangulation::removeEdge(Edge* edge)
{
	if (edge->triangle[0] == nullptr && edge->triangle[1] == nullptr)
	{
		for (int i = 0; i < 2; i++)
			edge->vertex[i]->removeEdge(edge);
		mEdgeset.erase(edge);
		return true;
	}
	return false;
}
bool Triangulation::removeVertex(Vertex* vertex)
{
	if (vertex->edges.empty())
	{
		mVertexset.erase(vertex);
		return true;
	}
	return false;
}

void Triangulation::deleteTriangle(Triangle* triangle)
{
	if (removeTriangle(triangle))
	{
		Edge* edges[3] = { triangle->edge[0], triangle->edge[1], triangle->edge[2] };
		for (int i = 0; i < 3; i++)
			edges[i]->removeGhostTriangle(triangle);
		delete triangle;

		for (int i = 0; i < 3; i++)
		{
			if(edges[i]->ghostTriangle[0] == nullptr && edges[i]->ghostTriangle[1] == nullptr)
				deleteEdge(edges[i]);
		}
	}
}
void Triangulation::deleteEdge(Edge* edge)
{
	if (removeEdge(edge))
	{
		Vertex* vertex[2] = { edge->vertex[0], edge->vertex[1] };
		delete edge;

		for (int i = 0; i < 2; i++)
			deleteVertex(vertex[i]);
	}
}
void Triangulation::deleteVertex(Vertex* vertex)
{
	if (removeVertex(vertex))
		delete vertex;
}

Triangle* Triangulation::findTriangle(Vertex* v1, Vertex* v2, Vertex* v3)
{
	if (v1 == nullptr || v2 == nullptr || v2 == nullptr)
		return nullptr;

	if (v1 == v2 || v2 == v3 || v1 == v3)
		return nullptr;

	Edge* edge1 = findEdge(v1, v2);
	Edge* edge2 = findEdge(v1, v3);
	Edge* edge3 = findEdge(v2, v3);

	if (edge1 == nullptr || edge2 == nullptr || edge3 == nullptr)
		return nullptr;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
			{
				if (edge1->triangle[i] && edge2->triangle[j] && edge3->triangle[k]
					&& edge1->triangle[i] == edge2->triangle[j] && edge1->triangle[j] == edge2->triangle[k])
					return edge1->triangle[i];
			}
	return nullptr;
}
Edge* Triangulation::findEdge(Vertex* v1, Vertex* v2)
{
	if (v1 == nullptr || v2 == nullptr)
		return nullptr;

	if (v1 == v2)
		return nullptr;

	for (auto it = v1->edges.begin(); it != v1->edges.end(); it++)
		if (v2->edges.find(*it) != v2->edges.end())
			return *it;

	return nullptr;
}
Vertex* Triangulation::findVertex(const GCore::Vec2d& Vec2d)
{
	Vertex* vertex = new Vertex(Vec2d);
	auto it = mVertexset.find(vertex);
	if (it != mVertexset.end())
	{
		delete vertex;
		return *it;
	}
	else
	{
		delete vertex;
		return nullptr;
	}
}

bool Triangulation::equal(const Vertex* v1, const Vertex* v2)
{
	return (v1->coord == v2->coord);

}
bool Triangulation::equal(const Edge* e1, const Edge* e2)
{
	auto & p1 = e1->vertex[0]->coord;
	auto & p2 = e1->vertex[1]->coord;
	auto & q1 = e2->vertex[0]->coord;
	auto & q2 = e2->vertex[1]->coord;

	return (p1 == q1 && p2 == q2) || (p1 == q2 && p2 == q1);
}
bool Triangulation::equal(const Triangle* t1, const Triangle* t2)
{
	auto & p1 = t1->vertex[0]->coord;
	auto & p2 = t1->vertex[1]->coord;
	auto & p3 = t1->vertex[2]->coord;
	auto & q1 = t2->vertex[0]->coord;
	auto & q2 = t2->vertex[1]->coord;
	auto & q3 = t2->vertex[2]->coord;

	return	(p1 == q1 && p2 == q2 && p3 == q3) ||
		(p1 == q1 && p2 == q3 && p2 == q2) ||
		(p1 == q2 && p2 == q1 && p3 == q3) ||
		(p1 == q2 && p2 == q3 && p3 == q1) ||
		(p1 == q3 && p2 == q1 && p3 == q2) ||
		(p1 == q3 && p2 == q2 && p3 == q1);
}
size_t	Triangulation::hash(const GCore::Vec2d & point)
{
	return std::hash<double>()(point.x*PRIME + point.y);
}
bool Triangulation::isCounterClockwise(const GCore::Vec2d & inPoint1,
	const GCore::Vec2d & inPoint2,
	const GCore::Vec2d & inPoint3)
{
	return	(inPoint3.y - inPoint1.y)*(inPoint2.x - inPoint1.x) >
		(inPoint2.y - inPoint1.y)*(inPoint3.x - inPoint1.x);
}
void Triangulation::getFacets(std::vector<GCore::Vec3i>& outFacets)
{
	std::unordered_map<Vertex*, int> v2iMap;

	int v = 0;
	for (auto const vertex : mVertexset)
	{
		v2iMap.emplace(vertex, v++);
	}

	for (auto const face : mTriangleset)
	{
		outFacets.push_back(GCore::Vec3i(v2iMap[face->edge[0]->vertex[0]], v2iMap[face->edge[0]->vertex[1]], v2iMap[face->opposite(face->edge[0])]));
	}
}
void Triangulation::writeToOff(const char* filname)
{
	std::ofstream file;
	file.open(filname);

	file << "OFF" << std::endl;
	file << mVertexset.size() << " " << mTriangleset.size() << " 0" << std::endl;

	std::unordered_map<Vertex*, int> v2iMap;

	int v = 0;
	for (auto const vertex : mVertexset)
	{
		file << vertex->coord.x << " " << vertex->coord.y << " 0" << std::endl;
		v2iMap.emplace(vertex, v++);
	}

	for (auto const face : mTriangleset)
	{
		file << "3 "
			<< v2iMap[face->edge[0]->vertex[0]] << " "
			<< v2iMap[face->edge[0]->vertex[1]] << " "
			<< v2iMap[face->opposite(face->edge[0])]
			<< std::endl;

	}
}