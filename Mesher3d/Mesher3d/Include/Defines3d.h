#pragma once

#include <unordered_set>
#include <unordered_map>
#include <list>
#include <vector>
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

	typedef std::unordered_map<Vertex*, double> LFSMap;
	typedef std::vector<size_t> Polygon;
	typedef std::vector<GCore::Vec3i> FaceList;

	typedef std::tuple<GCore::Vec3d, GCore::Vec3d> EdgeTuple;
	typedef std::tuple<GCore::Vec3d, GCore::Vec3d, GCore::Vec3d> FaceTuple;
	typedef std::tuple<GCore::Vec3d, GCore::Vec3d, GCore::Vec3d, GCore::Vec3d> TetraTuple;
	typedef std::tuple<Vertex*, Vertex*, Vertex*> FaceVtxTuple;

	typedef std::list<EdgeTuple> EdgeCoordList;
	typedef std::list<FaceTuple> FaceCoordList;
	typedef std::list<TetraTuple> TetraCoordList;

	typedef std::list<Vertex*> VertexList;
	typedef std::list<Edge*> EdgeList;
	typedef std::vector<Vertex*> VertexArr;
	typedef std::vector<Edge*> EdgeArr;
	typedef std::vector<Face*> FaceArr;
	typedef std::list<VertexArr> FaceVertexArr;
}