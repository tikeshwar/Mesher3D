#pragma once

#include <unordered_set>
#include <list>
#include "Defines.h"
#include "Vec.h"
//#include "../../GeomCore/Include/Defines.h"
//#include "../../Mesher2d/Include/Defines.h"

namespace M3d
{
	class Tetra;
	class Face;
	class Edge;
	class Vertex;

	typedef std::unordered_set<Tetra*> Tetraset;
	typedef std::unordered_set<Face*> Faceset;
	typedef std::unordered_set<Edge*> Edgeset;
	typedef std::unordered_set<Vertex*> Vertexset;
	typedef std::list<GCore::Vec3d> Coordset;
}