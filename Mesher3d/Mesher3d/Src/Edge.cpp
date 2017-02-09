#include "Vertex.h"
#include "Edge.h"
#include "Face.h"

using namespace GCore;
using namespace M3d;

Edge::Edge(Vertex* _v1, Vertex* _v2)
{
	vertex[0] = _v1;
	vertex[1] = _v2;

	vertex[0]->addEdge(this);
	vertex[1]->addEdge(this);
}

void Edge::dispose()
{
	vertex[0]->removeEdge(this);
	vertex[1]->removeEdge(this);
}

bool Edge::removeFace(Face* face)
{
	if (faces.empty())
		return false;

	auto it = faces.find(face);
	if (it != faces.end())
	{
		faces.erase(it);
		return true;
	}
	return false;
}

bool Edge::isConnected()
{
	/*bool connected = false;
	for (auto it = faces.begin(); it != faces.end(); it++)
		if ((*it) != nullptr)
		{
			connected = true;
			break;
		}
	return connected;*/
	return !faces.empty();
}

Vertex* Edge::otherVertex(const Vertex* inVertex)
{
	if (inVertex == vertex[0])
		return vertex[1];
	else if (inVertex == vertex[1])
		return vertex[0];
	return nullptr;
}

void Edge::getSurroundingTetras(Tetraset& outTetras)
{
	for (auto it = faces.begin(); it != faces.end(); it++)
		for (int i = 0; i < 2; i++)
			if ((*it)->tetra[i])
				outTetras.emplace((*it)->tetra[i]);
}

