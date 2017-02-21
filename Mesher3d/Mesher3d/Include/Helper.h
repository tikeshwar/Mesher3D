#pragma once

#include "Defines.h"
#include "Defines3d.h"
#include "Tessellation.h"

static bool gDebug = false;

namespace M3d
{
	class Vertex;
	class Edge;
	class Face;
	class Tetra;

	namespace Helper
	{
		bool locallyDelaunay(Face* face);
		void reflexEdges(Face* face, Edgeset& outReflexEdges);
		void deleteTetras(Tessellation& tess, const Tetraset& tetraToDelete);
		
		bool checkIfZeroVolumeTetra(const Faceset& faceset, const GCore::Vec3d& point);
		bool checkIfDelaunay(const Tetraset& tetraset);
		int checkZeroVolume(const Tessellation& tess);
		bool checkFaceSanity(const Tessellation& tess);


		void visualize(const Tessellation& tess, const char* filename);

		//void visualize(const Vertex* v1, const Vertex* v2, const char* filename);
		void visualizeFaces(const Tessellation& tess, const Faceset& faceset, const char* filename);
		void visualizeTetras(const Tessellation& tess, const Tetraset& tetraset, const char* filename);

		void getEdgeCoordList(const Edgeset& edgeset, EdgeCoordList& outEdgeCoordList);
		void getFaceCoordList(const Faceset& edgeset, FaceCoordList& outFaceCoordList);
		void getTetraCoordList(const Tetraset& edgeset, TetraCoordList& outTetraCoordList);


		void triangulateCavity(Tessellation& tess, const Tetraset& tetraToDelete, const GCore::Vec3d& point, Tetraset& outTetras);
		//void findInitialCavity(const GCore::Vec3d& point, Tetraset& outTetraset);
		bool findCircumscribingTetras(const Tessellation& tess, const GCore::Vec3d& point, Tetraset& outTetraset);
		void correctCavity(const GCore::Vec3d& point, Tetraset& outTetraset);
		void findDeletedPoints(const Vertexset& verticesToCheck, const Faceset& nonhullfaces, std::list<GCore::Vec3d>& outPoints);
		void findHullFaces(const Tetraset& tetraToDelete, Faceset& outFaceset, Faceset& outFacesToDelete);
		void findAllCoords(const Tetraset& tetraToDelete, Coordset& outCoords);

		void fillvoid(Tessellation& tess, const FaceCoordList& faceCoords, const GCore::Vec3d& point, Tetraset& newTetras);
	};
}
