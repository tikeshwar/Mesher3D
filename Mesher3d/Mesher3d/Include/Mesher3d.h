#pragma once

#include <unordered_map>
#include "Defines.h"
#include "Defines3d.h"
#include "BBox.h"
#include "PointInPolygon.h"
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
	public:
		Mesher3d(const Pslg& _pslg);
		~Mesher3d() = default;

		void createDelaunay3D();

		void writeToOff(const char* filename)
		{
			mTess.writeToOff(filename);
		}



	private:

		void createSuperTetra();
		void createInitialDelaunay();
		void refineDelaunay();
		void hashPslg();

		//bool insertWatson(const GCore::Vec3d& point);
		//bool insertLawson(const GCore::Vec3d& point);
		////void swapFace(Face* face, Tetraset& outTetras);
		////bool flippable(const GCore::Vec3d& e1, const GCore::Vec3d& e2,
		////	const GCore::Vec3d& f1, const GCore::Vec3d& f2, const GCore::Vec3d& f3);

		//void tryInsertPoint(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		//void insertInTetra(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras);
		//void insertInFace(Face* face, const GCore::Vec3d& point, Tetraset& outTetras);
		//void insertInEdge(Edge* edge, const GCore::Vec3d& point, Tetraset& outTetras);
		//void flip(Face* face, const Edgeset& reflexEdges, Tetraset& outTetras, Faceset& deletedTetras);
		//void flip14(const GCore::Vec3d& point, Tetra* tetra, Tetraset& outTetras, Faceset& deletedFaces);
		//void flip44(Face* face, Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		//void flip23(Face* face, Tetraset& outTetras, Faceset& deletedTetras);
		//void flip32(Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedTetras);
		//void flip41(/*Face* face, Tetraset& outTetras, Tetraset& deletedTetras*/);
		//bool locallyDelaunay(Face* face);
		//void reflexEdges(Face* face, Edgeset& outReflexEdges);

		void findNonlinearPoints(const VertexArr& polygon, VertexArr& outVertices);
		void findOrderedPoints(const Edgeset& polygonEdges, VertexArr& outOrderedPts);

		//void triangulateCavity(const Tetraset& tetraToDelete, const GCore::Vec3d& point, Tetraset& outTetras);
		////void findInitialCavity(const GCore::Vec3d& point, Tetraset& outTetraset);
		//bool findCircumscribingTetras(const GCore::Vec3d& point, Tetraset& outTetraset);
		//void correctCavity(const GCore::Vec3d& point, Tetraset& outTetraset);
		//void findDeletedPoints(const Vertexset& verticesToCheck, const Faceset& nonhullfaces, std::list<GCore::Vec3d>& outPoints);
		//void findHullFaces(const Tetraset& tetraToDelete, Faceset& outFaceset, Faceset& outFacesToDelete);
		//void findAllCoords(const Tetraset& tetraToDelete, Coordset& outCoords);
		//bool checkIfZeroVolumeTetra(const Faceset& faceset, const GCore::Vec3d& point);

		//void deleteTetras(const Tetraset& tetraToDelete);
		//void fillvoid(const FaceCoordList& faceCoords, const GCore::Vec3d& point, Tetraset& newTetras);
		void updateBoundary(Tetraset& newTetras, Tetraset& tetrasTodel);
		void triangulate(const Polygon& polygon, FaceList& outFaces);
		void triangulate(const VertexArr& polygon, Faceset& outFaces);

		bool findInBetweenVertices(const Vertex* v1, const Vertex* v2, VertexList& outVertices, EdgeList& outEdges);
		void findBoundaryFaces(Faceset& outbdryFaces, FaceVertexArr& outMissingFaces);
		void findPolygonEdges(const VertexArr& vertexArr, Edgeset& outEdgeset);
		void findMediaryEdges(const VertexArr& vertexArr, const Edgeset& polygonEdges, Edgeset& outEdges);
		void findPolygonFaces(const Edgeset& polygonEdges, const Edgeset& mediaryEdges,
			Faceset& outFaceset, Edgeset& outWidowEdges);
		void findWidowEdges(const GCore::PointInPolygon& pip, const Edgeset& edges, const Vertexset& polygonVertices,
			Faceset& outFaceset, Edgeset& outWidowEdges, size_t faceCountCriteria);
		void findMissingFaces(const Edgeset& polygonEdges, const Edgeset& widowEdges, FaceVertexArr& missingFaces);



		double localFeatureSize(Vertex* vertex);
		//bool isVisible(Tetra* tetra, const GCore::Vec3d& point);
		bool isEdgeEncroached(Vertex* v1, Vertex* v2, Vertexset& outEncroachingVertices);
		bool getStienerPoint(Vertex* v1, Vertex* v2, GCore::Vec3d& outStienerPoint);

		void findCavity(const std::vector<GCore::Vec3d>& faceBdry, Tetraset& outCavityTetras);
		void separateCavities(const std::vector<GCore::Vec3d>& faceBdry, Tetraset& cavityTetras,
			Vertexset& outUpperCavityVertexset, Vertexset& outLowerCavityVertexset);
		void triangulateCavity(Faceset& faces, Tetraset& outTetras);

		void recoverEdges(size_t& outMissingCount, size_t& outRecovered);
		bool recoverByEdgeFlip(Vertex* v1, Vertex* v2);
		bool recoverEdge(Vertex* v1, Vertex* v2);
		void recoverFaces(Faceset& outBoundaryFaces);
		void recoverFace(const VertexArr& face, Faceset& outFaces);
		void findIntersectingTetras(const VertexArr& face, Tetraset& outTetras);
		bool polygonxTetra(const Edgeset& polygonEdges, const GCore::PointInPolygon& pip, const Tetra* tetra);

		void removExteriorTetras(const Faceset& boundaryFaces);
		void detectExteriorTetras(Tetra* extTetra, const Faceset& boundaryFaces, Tetraset& outeExteriorTetras);


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
