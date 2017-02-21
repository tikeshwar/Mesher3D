#include <iostream>
#include <unordered_map>
#include "GeomTest.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Tetra.h"
#include "UniqueQueue.h"
#include "Helper.h"
#include "PointInsertion.h"

using namespace GCore;
using namespace M3d;

bool PointInsertion::insertWatson(const Vec3d& point)
{
	Vertex* vertex = mTess.findVertex(point);
	if (vertex)
		return true;

	Tetraset tetraToDelete;
	if (!Helper::findCircumscribingTetras(mTess, point, tetraToDelete))
		return false;

	std::list<Vec3d> additionalPts;
	Helper::correctCavity(point, tetraToDelete);


	Coordset coordset;
	Helper::findAllCoords(tetraToDelete, coordset);

	Tetraset newTetras;
	Helper::triangulateCavity(mTess, tetraToDelete, point, newTetras);

	if (newTetras.empty())
		return false;

	/*for (auto it = deletedPts.begin(); it != deletedPts.end(); it++)
	{
	insertLawson(*it);
	}*/

	return true;
}

bool PointInsertion::insertLawson(const GCore::Vec3d& point)
{
	Vertex* vertex = mTess.findVertex(point);
	if (vertex)
		return true;

	Tetra* tetra = mTess.locateTetra(point);
	if (!tetra)
	{
		std::cout << "tetra not found" << std::endl;
		if (gDebug)
			Helper::visualizeTetras(mTess, mTess.getTetras(), "cubeTet.off");
	}

	if (tetra)
	{
		GCore::UniqueQueue<Face*> faceQueue;

		Tetraset newTetras;
		Faceset deletedFaces;

		//if (gDebug)
		//{
		//	Tetraset tset;
		//	tset.emplace(tetra);
		//	visualize(tset, "cubeTet.off");
		//}

		tryInsertPoint(tetra, point, newTetras);

		//if (gDebug)
		//{
		//	for (auto it = newTetras.begin(); it != newTetras.end(); it++)
		//	{
		//		Tetraset tset;
		//		tset.emplace(*it);
		//		visualize(tset, "cubeTet.off");

		//		for (int i = 0; i < 4; i++)
		//		{
		//			tset.clear();
		//			tset.emplace((*it)->face[i]->tetra[0]);
		//			tset.emplace((*it)->face[i]->tetra[1]);
		//			visualize(tset, "cubeTet.off");
		//		}
		//	}
		//}


		if (gDebug)
		{
			Helper::visualizeTetras(mTess, newTetras, "cubeTet.off");
			Helper::visualizeTetras(mTess, mTess.getTetras(), "cubeTet.off");
		}

		//flip14(point, tetra, newTetras, deletedFaces);

		for (auto it = newTetras.begin(); it != newTetras.end(); it++)
		{
			for (int i = 0; i < 4; i++)
				if (newTetras.find((*it)->face[i]->otherTetra(*it)) == newTetras.end())
					faceQueue.push((*it)->face[i]);
		}

		while (!faceQueue.empty())
		{
			Face* face = faceQueue.pop();

			if (!Helper::locallyDelaunay(face))
			{
				gFlipCount++;

				Edgeset outReflexEdges;
				Helper::reflexEdges(face, outReflexEdges);

				if (gDebug)
				{
					Tetraset fset;
					fset.emplace(face->tetra[0]);
					fset.emplace(face->tetra[1]);
					Helper::visualizeTetras(mTess, mTess.getTetras(), "cubeTet.off");
					Helper::visualizeTetras(mTess, fset, "cubeTet.off");
				}

				newTetras.clear();
				deletedFaces.clear();
				flip(face, outReflexEdges, newTetras, deletedFaces);

				//int zerovol = checkZeroVolume();
				//if (zerovol > 0)
				//{
				//	std::cout << "zerovol:" << zerovol << std::endl;
				//}

				if (gDebug)
				{
					Helper::visualizeTetras(mTess, newTetras, "cubeTet.off");
					std::cout << "insane" << std::endl;
				}

				for (auto it = deletedFaces.begin(); it != deletedFaces.end(); it++)
					faceQueue.erase((*it));

				for (auto it = newTetras.begin(); it != newTetras.end(); it++)
					for (int i = 0; i < 4; i++)
						if (newTetras.find((*it)->face[i]->otherTetra(*it)) == newTetras.end())
							faceQueue.push((*it)->face[i]);
			}
		}
	}

	return true;
}

void PointInsertion::tryInsertPoint(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras)
{
	// it means it lies on face
	Face* containerFace = nullptr;
	for (int i = 0; i < 4; i++)
	{
		Face* tface = tetra->face[i];
		double vol = GeomTest::signedVolume(
			tface->vertex[0]->coord,
			tface->vertex[1]->coord,
			tface->vertex[2]->coord,
			point);

		if (GCore::isZero<double>(vol))
		{
			containerFace = tface;
			break;
		}
	}

	Edge* containerEdge = nullptr;
	if (containerFace)
	{
		// let us check if it lies on some edge
		for (int i = 0; i < 3; i++)
		{
			Edge* faceEdge = containerFace->edge[i];
			double area = GeomTest::unSignedArea3d(point, faceEdge->vertex[0]->coord, faceEdge->vertex[1]->coord);
			if (GCore::isZero<double>(area))
			{
				containerEdge = faceEdge;
				break;
			}
		}
	}

	if (containerEdge)
		insertInEdge(containerEdge, point, outTetras);
	else if (containerFace)
		insertInFace(containerFace, point, outTetras);
	else
		insertInTetra(tetra, point, outTetras);
}

void PointInsertion::insertInTetra(Tetra* tetra, const Vec3d& point, Tetraset& outTetras)
{
	Tetraset tetraToDelete;
	tetraToDelete.emplace(tetra);

	Helper::triangulateCavity(mTess, tetraToDelete, point, outTetras);
}

void PointInsertion::insertInFace(Face* face, const GCore::Vec3d& point, Tetraset& outTetras)
{
	Tetraset tetraToDelete;
	for (int i = 0; i < 2; i++)
	{
		if (face->tetra[i])
			tetraToDelete.emplace(face->tetra[i]);
	}

	if (gDebug)
	{
		Faceset fset;
		fset.emplace(face);
		Helper::visualizeFaces(mTess, fset, "cubeTet.off");
		Helper::visualizeTetras(mTess, tetraToDelete, "cubeTet.off");
	}

	Helper::triangulateCavity(mTess, tetraToDelete, point, outTetras);
}

void PointInsertion::insertInEdge(Edge* edge, const GCore::Vec3d& point, Tetraset& outTetras)
{
	Tetraset tetraToDelete;
	edge->getSurroundingTetras(tetraToDelete);

	Helper::triangulateCavity(mTess, tetraToDelete, point, outTetras);
}

void PointInsertion::flip(Face* face, const Edgeset& reflexEdges, Tetraset& outTetras, Faceset& deletedFaces)
{
	switch (reflexEdges.size())
	{
	case 0:
		//case 3:
	{
		flip23(face, outTetras, deletedFaces);
	}
	break;
	case 1:
	{
		Edge* reflexEdge = *reflexEdges.begin();

		if (reflexEdge->faces.size() == 3)
			flip32(reflexEdge, outTetras, deletedFaces);
		else if (reflexEdge->faces.size() == 4)
			flip44(face, reflexEdge, outTetras, deletedFaces);
	}
	break;
	case 2:
		//flip41(face, outTetras, deletedFaces);
		if (gDebug)
		{
			Tetraset tset;
			for (int i = 0; i < 2; i++)
			{
				if (face->tetra[i])
					tset.emplace(face->tetra[i]);
			}
			Helper::visualizeTetras(mTess, tset, "cubeTet.off");
		}
		break;

	}
}

void PointInsertion::flip23(Face* face, Tetraset& outTetras, Faceset& deletedFaces)
{
	Tetraset tetraToDelete;
	tetraToDelete.emplace(face->tetra[0]);
	tetraToDelete.emplace(face->tetra[1]);

	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
	{
		for (int i = 0; i < 4; i++)
			deletedFaces.emplace((*it)->face[i]);
	}

	Vec3d refPoint = face->tetra[0]->opposite(face)->coord;

	if (gDebug)
		Helper::visualizeTetras(mTess, tetraToDelete, "cubeTet.off");

	Helper::triangulateCavity(mTess, tetraToDelete, refPoint, outTetras);

	//if (!outTetras.empty() && outTetras.size() != 3)
	//{
	//	visualize(outTetras, "cubeTet.off");
	//	visualize(mTess.getTetras(), "cubeTet.off");
	//}
	if (gDebug)
		Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
}

void PointInsertion::flip32(Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedFaces)
{
	if (reflexEdge->faces.size() != 3)
		return;

	Tetraset tetraToDelete;
	reflexEdge->getSurroundingTetras(tetraToDelete);

	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
	{
		for (int i = 0; i < 4; i++)
			deletedFaces.emplace((*it)->face[i]);
	}

	if (gDebug)
		Helper::visualizeTetras(mTess, tetraToDelete, "cubeTet.off");

	std::vector<Vec3d> faceCoords;
	for (auto it = reflexEdge->faces.begin(); it != reflexEdge->faces.end(); it++)
		faceCoords.push_back((*it)->opposite(reflexEdge)->coord);

	Vec3d refPoint;

	//GeomTest::IntersectionInfo intersectionInfo;
	//GeomTest::lineSegXTriangle(faceCoords[0], faceCoords[1], faceCoords[2],
	//	reflexEdge->vertex[0]->coord, reflexEdge->vertex[1]->coord, intersectionInfo);
	//if (intersectionInfo.intersectionPos == GeomTest::kInside)
	//	refPoint = intersectionInfo.intersectionPt;
	//else
	refPoint = (*reflexEdge->faces.begin())->opposite(reflexEdge)->coord;

	Helper::triangulateCavity(mTess, tetraToDelete, refPoint, outTetras);

	if (gDebug)
		Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
}

void PointInsertion::flip14(const GCore::Vec3d& point, Tetra* tetra, Tetraset& outTetras, Faceset& deletedFaces)
{
	Tetraset tetraToDelete;
	tetraToDelete.emplace(tetra);

	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
	{
		for (int i = 0; i < 4; i++)
			deletedFaces.emplace((*it)->face[i]);
	}

	Helper::triangulateCavity(mTess, tetraToDelete, point, outTetras);
}

void PointInsertion::flip44(Face* face, Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedFaces)
{
	Vertex* oppV1 = face->tetra[0]->opposite(face);
	Vertex* oppV2 = face->tetra[1]->opposite(face);

	double o3d = GeomTest::orient3d(oppV1->coord, oppV2->coord,
		reflexEdge->vertex[0]->coord, reflexEdge->vertex[1]->coord);
	if (!GCore::isZero(o3d))
		return;

	Vec3d center;
	double radius;
	GeomTest::findCircumcircle3d(oppV1->coord, reflexEdge->vertex[0]->coord, reflexEdge->vertex[1]->coord, center, radius);

	if ((center - oppV2->coord).magnitudeSqr() < radius*radius)
	{
		Vec3d nCenter;
		double nRadius;
		GeomTest::findCircumcircle3d(oppV1->coord, oppV2->coord, reflexEdge->vertex[0]->coord, nCenter, nRadius);

		if ((nCenter - reflexEdge->vertex[1]->coord).magnitudeSqr() > nRadius*nRadius)
		{
			Tetraset tetraToDelete;
			Vertexset leftside, rightside;
			reflexEdge->getSurroundingTetras(tetraToDelete);

			if (tetraToDelete.size() > 4)
				return;

			for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
			{
				for (int i = 0; i < 4; i++)
					deletedFaces.emplace((*it)->face[i]);
			}

			//Vec3d refPt = reflexEdge->pointAt(0.5, 0.5);
			Vec3d refPt = oppV1->coord;

			if (gDebug)
				Helper::visualizeTetras(mTess, tetraToDelete, "cubeTet.off");

			Helper::triangulateCavity(mTess, tetraToDelete, refPt, outTetras);

			if (gDebug)
				Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
		}
	}
}

void PointInsertion::flip41(/*Face* face, Tetraset& outTetras, Tetraset& deletedTetras*/)
{
	//Edge* nonReflexEdge = nullptr;
	//for (int i = 0; i < 3; i++)
	//	if (face->edge[i]->faces.size() == 3)
	//		reflexEdge = face->edge[i];
}


