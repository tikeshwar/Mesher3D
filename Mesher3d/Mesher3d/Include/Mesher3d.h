#pragma once

#include <unordered_map>
#include "Defines.h"
#include "BBox.h"
#include "Tessellation.h"
#include "Pslg.h"

namespace M3d
{
	class Vertex;
	class Edge;
	class Face;
	class Tetra;
	class EdgeGrid;

	class Mesher3d
	{
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
		typedef std::list<FaceVtxTuple> FaceVtxTupleList;

	public:
		Mesher3d(const Pslg& _pslg);
		~Mesher3d() = default;

		void createDelaunay3D();

		void writeToOff(const char* filename)
		{
			mTess.writeToOff(filename);
		}

		void visualize(const char* filename)
		{
			mTess.writeToOff(filename);
			system("start meshlab.exe cubeTet.off");

		}

		void visualize(const Faceset& faceset, const char* filename);
		void visualize(const Tetraset& tetraset, const char* filename);
		
	private:

		void createSuperTetra();
		void createInitialDelaunay();
		void refineDelaunay();
		void hashPslg();

		bool insertWatson(const GCore::Vec3d& point);
		void insertLawson(const GCore::Vec3d& point);
		//void swapFace(Face* face, Tetraset& outTetras);
		//bool flippable(const GCore::Vec3d& e1, const GCore::Vec3d& e2,
		//	const GCore::Vec3d& f1, const GCore::Vec3d& f2, const GCore::Vec3d& f3);

		void tryInsertPoint(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInTetra(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInFace(Face* face, const GCore::Vec3d& point, Tetraset& outTetras);
		void insertInEdge(Edge* edge, const GCore::Vec3d& point, Tetraset& outTetras);
		void flip(Face* face, const Edgeset& reflexEdges, Tetraset& outTetras, Faceset& deletedTetras);
		void flip44(Face* face, Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		void flip23(Face* face, Tetraset& outTetras, Faceset& deletedTetras);
		void flip32(Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		void flip41(/*Face* face, Tetraset& outTetras, Tetraset& deletedTetras*/);
		bool locallyDelaunay(Face* face);
		void reflexEdges(Face* face, Edgeset& outReflexEdges);


		void triangulateCavity(const Tetraset& tetraToDelete, const GCore::Vec3d& point,Tetraset& outTetras);
		void findInitialCavity(const GCore::Vec3d& point, Tetraset& outTetraset);
		bool findCircumscribingTetras(const GCore::Vec3d& point, Tetraset& outTetraset);
		void findHullFaces(const Tetraset& tetraToDelete, Faceset& outFaceset);
		void findAllCoords(const Tetraset& tetraToDelete, Coordset& outCoords);

		void deleteTetras(const Tetraset& tetraToDelete);
		void fillvoid(const FaceCoordList& faceCoords, const GCore::Vec3d& point, Tetraset& newTetras);
		void updateBoundary(Tetraset& newTetras, Tetraset& tetrasTodel);
		void triangulate(const Polygon& polygon, FaceList& outFaces);

		bool findInBetweenVertices(Vertex* v1, Vertex* v2, VertexList& outVertices, EdgeList& outEdges);
		void findBoundaryFaces(Faceset& outbdryFaces, FaceVtxTupleList& outMissingFaces);

		void getEdgeCoordList(const Edgeset& edgeset, EdgeCoordList& outEdgeCoordList);
		void getFaceCoordList(const Faceset& edgeset, FaceCoordList& outFaceCoordList);
		void getTetraCoordList(const Tetraset& edgeset, TetraCoordList& outTetraCoordList);

		double localFeatureSize(Vertex* vertex);
		//bool isVisible(Tetra* tetra, const GCore::Vec3d& point);
		bool isEdgeEncroached(Vertex* v1, Vertex* v2, Vertexset& outEncroachingVertices);
		bool getStienerPoint(Vertex* v1, Vertex* v2, GCore::Vec3d& outStienerPoint);
		
		void findCavity(const std::vector<GCore::Vec3d>& faceBdry, Tetraset& outCavityTetras);
		void separateCavities(const std::vector<GCore::Vec3d>& faceBdry, Tetraset& cavityTetras,
			Vertexset& outUpperCavityVertexset, Vertexset& outLowerCavityVertexset);
		void triangulateCavity(const std::vector<GCore::Vec3d>& faceBdry, Vertexset& cavity);

		void recoverEdges(size_t& outMissingCount, size_t& outRecovered);
		bool recoverByEdgeFlip(Vertex* v1, Vertex* v2);
		void recoverEdge(Vertex* v1, Vertex* v2);
		void recoverFaces();
		
		void removExteriorTetras();
		void detectExteriorTetras(Tetra* extTetra, const Faceset& boundaryFaces, Tetraset& outeExteriorTetras);

		bool checkIfDelaunay(const Tetraset& tetraset);
		bool checkFaceSanity();
		//void getBoundaryEdges(Edgeset& outEdgeset);
		void getBoundaryFaces(Faceset& outFaceset);

		void cleanTetrasOutOfDomainBox();
	private:
		Tessellation mTess;

		Tessellation mPslgTess;
		EdgeGrid* mPslgGrid;
		LFSMap mLfsMap;

		Faceset mBdryFaces;
		Vertexset mBdryVertices;

		Pslg pslg;
	};
}
