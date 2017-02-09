#pragma once

#include <unordered_set>

#include "Defines.h"

#define PRIME	2147483647

namespace M2d
{
	class Vertex;
	class Edge;
	class Triangle;

	typedef std::unordered_set<Vertex*> Vertexset;
	typedef std::unordered_set<Edge*> Edgeset;
	typedef std::unordered_set<Triangle*> Triangleset;
}
