#include <Windows.h>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <cassert>
#include "UniqueQueue.h"
#include "BBox.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Tetra.h"
#include "EdgeGrid.h"
#include "PointInPolygon.h"
#include "Mesher2d.h"
#include "Mesher3d.h"

using namespace GCore;
using namespace M3d;

Tessellation ttess;
bool gDebug = false;

M3d::Mesher3d::Mesher3d(const Pslg& _pslg)
	:pslg(_pslg)
{
	pslg.calculateBBox();
}

void M3d::Mesher3d::createDelaunay3D()
{
	createSuperTetra();

	//hashPslg();

	createInitialDelaunay();
	//cleanTetrasOutOfDomainBox();
	visualize(mTess.getFaces(), "cubetet.off");
	//checkFaceSanity();
	//bool valid = false;
	size_t missing = 0, recovered = 0;
	do {
		missing = 0, recovered = 0;
		recoverEdges(missing, recovered);
	} while (missing > 0);

	//size_t missing = 0, recovered = 0;
	//recoverEdges(missing, recovered);
	//recoverFaces();
	//mTess = ttess;
	//visualize(mTess.getFaces(), "cubetet.off");

	removExteriorTetras();
}

void M3d::Mesher3d::createSuperTetra()
{
	Box3d pslgBox = pslg.box;
	Vec3d oriDiagonal = pslgBox.diagonal();

	Vec3d center = pslgBox.center();
	double radius = oriDiagonal.magnitude() / 2.0;

	double edgelen = sqrt(24)*radius;

	// four vert of tetra
	Vec3d v1 = Vec3d(edgelen, edgelen, edgelen) + center;
	Vec3d v2 = Vec3d(edgelen, -edgelen, -edgelen) + center;
	Vec3d v3 = Vec3d(-edgelen, edgelen, -edgelen) + center;
	Vec3d v4 = Vec3d(-edgelen, -edgelen, edgelen) + center;

	double fx = 1.2;
	double fy = 1.2;
	double fz = 1.2;
	pslgBox.scale(fx, fy, fz);
	Vec3d diagonal = pslgBox.diagonal();

	Vec3i divs(10, 10, 10);
	Vec3d delta(diagonal.x / divs.x, diagonal.y / divs.y, diagonal.z / divs.z);
	mTess.setUpGrid(pslgBox.lower, divs, delta);
	mPslgGrid = new EdgeGrid(pslgBox.lower, divs, delta);

	mTess.addTetra(v1, v2, v3, v4);
	//for (auto it = pslg.points.begin(); it != pslg.points.end(); it++)
	//	assert(tetra->contains(*it));
}
int gi = 0;
void M3d::Mesher3d::createInitialDelaunay()
{
	std::vector<size_t> randList;
	size_t i = 0;
	for (auto it = pslg.points.begin(); it != pslg.points.end(); it++)
		randList.push_back(i++);

	//randList = { 4,17,21,12,19,16,1,14,9,0,15,6,8,18,3,13,7,5,2,20,11,10 };

	std::srand(unsigned(std::time(0)));
	std::random_shuffle(randList.begin(), randList.end());

	for (auto it = randList.begin(); it != randList.end(); it++)
	{
		insertLawson(pslg.points[*it]);
		//insertWatson(pslg.points[*it]);
		//insertLawson(pslg.points[*it]);
		//if (gi == 23)
		//	visualize(mTess.getTetras(), "cubeTet.off");
		if (gi % 100 == 0)
			std::cout << gi << std::endl;
		gi++;
		//size_t bdryFaces = 0;
		//for (auto fit = mTess.getFaces().begin(); fit != mTess.getFaces().end(); fit++)
		//{
		//	if ((*fit)->tetra[0] == nullptr || (*fit)->tetra[1] == nullptr)
		//		bdryFaces++;
		//}

		//if (bdryFaces > 4)
		//	assert(false);

		//checkIfDelaunay();
		//visualize(mTess.getTetras(), "cubeTet.off");
	}

	//const Faceset& faceset = mTess.getFaces();
	//Faceset badFaces;
	//for (auto it = faceset.begin(); it != faceset.end(); it++)
	//{
	//	if ((*it)->tetra[0] && (*it)->tetra[1])
	//		if ((*it)->tetra[0]->circumSphere.inside((*it)->tetra[1]->opposite(*it)->coord))
	//			badFaces.emplace(*it);
	//}

	//while (!badFaces.empty())
	//{
	//	Face* face = *badFaces.begin();
	//	badFaces.erase(badFaces.begin());

	//	Tetraset tset;
	//	swapFace(face, tset);
	//}

}

void M3d::Mesher3d::refineDelaunay()
{
}

void M3d::Mesher3d::hashPslg()
{
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
	{
		//size_t idx[3] = { (*it).x, (*it).y, (*it).z };
		for (size_t i = 0; i < it->size(); i++)
		{
			Edge* edge = mPslgTess.addEdge(pslg.points[(*it)[i]], pslg.points[(*it)[(i + 1) % it->size()]]);
			mPslgGrid->emplace(edge);
		}
	}

	auto& vertices = mPslgTess.getVertices();
	for (auto it = vertices.begin(); it != vertices.end(); it++)
	{
		double lfs = localFeatureSize(*it);
		mLfsMap.emplace(*it, lfs);
	}
}

bool M3d::Mesher3d::insertWatson(const Vec3d& point)
{
	Vertex* vertex = mTess.findVertex(point);
	if (vertex)
		return false;

	Tetraset tetraToDelete;
	if (!findCircumscribingTetras(point, tetraToDelete))
		return false;

	Coordset coordset;
	findAllCoords(tetraToDelete, coordset);

	Tetraset newTetras;
	triangulateCavity(tetraToDelete, point, newTetras);

	return true;
}

void M3d::Mesher3d::insertLawson(const GCore::Vec3d& point)
{
	Vertex* vertex = mTess.findVertex(point);
	if (vertex)
		return;

	Tetra* tetra = mTess.locateTetra(point);
	if (!tetra)
	{
		const Tetraset& tset = mTess.getTetras();
		for (auto it = tset.begin(); it != tset.end(); it++)
		{
			if ((*it)->contains(point))
			{
				tetra = *it;
				break;
			}
		}

		if (gDebug)
			visualize(mTess.getTetras(), "cubeTet.off");
	}

	if (tetra)
	{
		UniqueQueue<Face*> faceQueue;

		Tetraset newTetras;
		Faceset deletedFaces;

		if (gDebug)
		{
			Tetraset tset;
			tset.emplace(tetra);
			visualize(tset, "cubeTet.off");
		}

		tryInsertPoint(tetra, point, newTetras);

		if (gDebug)
		{
			for (auto it = newTetras.begin(); it != newTetras.end(); it++)
			{
				Tetraset tset;
				tset.emplace(*it);
				visualize(tset, "cubeTet.off");

				for (int i = 0; i < 4; i++)
				{
					tset.clear();
					tset.emplace((*it)->face[i]->tetra[0]);
					tset.emplace((*it)->face[i]->tetra[1]);
					visualize(tset, "cubeTet.off");
				}
			}
		}


		if (gDebug)
		{
			visualize(newTetras, "cubeTet.off");
			visualize(mTess.getTetras(), "cubeTet.off");
		}

		for (auto it = newTetras.begin(); it != newTetras.end(); it++)
		{
			for (int i = 0; i < 4; i++)
				if (newTetras.find((*it)->face[i]->otherTetra(*it)) == newTetras.end())
					faceQueue.push((*it)->face[i]);
		}

		while (!faceQueue.empty())
		{
			Face* face = faceQueue.pop();

			if (!locallyDelaunay(face))
			{
				Edgeset outReflexEdges;
				reflexEdges(face, outReflexEdges);

				if (gDebug)
				{
					Tetraset fset;
					fset.emplace(face->tetra[0]);
					fset.emplace(face->tetra[1]);
					visualize(mTess.getTetras(), "cubeTet.off");
					visualize(fset, "cubeTet.off");
				}

				newTetras.clear();
				deletedFaces.clear();
				flip(face, outReflexEdges, newTetras, deletedFaces);

				for (auto it = deletedFaces.begin(); it != deletedFaces.end(); it++)
					faceQueue.erase((*it));

				for (auto it = newTetras.begin(); it != newTetras.end(); it++)
					for (int i = 0; i < 4; i++)
						if (newTetras.find((*it)->face[i]->otherTetra(*it)) == newTetras.end())
							faceQueue.push((*it)->face[i]);
			}
		}
	}
}

//void M3d::Mesher3d::swapFace(Face* face, Tetraset& outTetras)
//{
//	// basically its a 2-3 face swap
//	// 2 deletion and 3 creation
//	if (!face->tetra[0] || !face->tetra[1])
//		return;
//
//	Vec3d oppV0 = face->tetra[0]->opposite(face)->coord;
//	Vec3d oppV1 = face->tetra[1]->opposite(face)->coord;
//	Vec3d faceV0 = face->vertex[0]->coord;
//	Vec3d faceV1 = face->vertex[1]->coord;
//	Vec3d faceV2 = face->vertex[2]->coord;
//
//	// delete the tetrahedra
//	Tetraset markedTetras;
//	markedTetras.emplace(face->tetra[0]);
//	markedTetras.emplace(face->tetra[1]);
//	deleteTetras(markedTetras);
//
//	Tetra* newTetra = mTess.addTetra(oppV0, oppV1, faceV0, faceV1);
//	if (newTetra)
//		outTetras.emplace(newTetra);
//	newTetra = mTess.addTetra(oppV0, oppV1, faceV1, faceV2);
//	if (newTetra)
//		outTetras.emplace(newTetra);
//	newTetra = mTess.addTetra(oppV0, oppV1, faceV2, faceV0);
//	if (newTetra)
//		outTetras.emplace(newTetra);
//}

//bool M3d::Mesher3d::flippable(const GCore::Vec3d& e1, const GCore::Vec3d& e2,
//	const GCore::Vec3d& f1, const GCore::Vec3d& f2, const GCore::Vec3d& f3)
//{
//	//TODO:
//	// find four coplanar points
//	auto checkFlippable = [](const Vec3d& u, const Vec3d& v, const Vec3d& p, const Vec3d& q)->bool
//	{
//		// test if fourth point is inside
//		auto area = [](const Vec3d& p, const Vec3d& q, const Vec3d& r)->double
//		{
//			return 0.5*(p - q).cross(p - r).magnitude();
//		};
//
//		double e1e2f1 = area(u, v, p);
//		double e1e2f2 = area(u, v, q);
//		double f1f2e1 = area(p, q, u);
//		double f1f2e2 = area(p, q, v);
//
//		if (GCore::isEqual<double>(e1e2f1 + e1e2f2, f1f2e1 + f1f2e2))
//		{
//			// convex area but lets check if it folds into a triangle
//			if (GCore::isZero<double>(e1e2f1) &&
//				GCore::isZero<double>(e1e2f2) &&
//				GCore::isZero<double>(f1f2e1) &&
//				GCore::isZero<double>(f1f2e2))
//			{
//				return false;
//			}
//			else
//			{
//				return false;
//			}
//		}
//		else
//		{
//			return true;
//		}
//	};
//
//	if (GCore::isZero<double>(GeomTest::signedVolume(e1, e2, f1, f2))) /*||
//		!checkFlippable(e1, e2, f1, f2))*/
//		return false;
//	if (GCore::isZero<double>(GeomTest::signedVolume(e1, e2, f2, f3)))/*||
//		!checkFlippable(e1, e2, f2, f3))*/
//		return false;
//	if (GCore::isZero<double>(GeomTest::signedVolume(e1, e2, f3, f1))) /*||
//		!checkFlippable(e1, e2, f3, f1))*/
//		return false;
//	return true;
//}

void M3d::Mesher3d::tryInsertPoint(Tetra* tetra, const GCore::Vec3d& point, Tetraset& outTetras)
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

void M3d::Mesher3d::insertInTetra(Tetra* tetra, const Vec3d& point, Tetraset& outTetras)
{
	Tetraset tetraToDelete;
	tetraToDelete.emplace(tetra);

	triangulateCavity(tetraToDelete, point, outTetras);
}

void M3d::Mesher3d::insertInFace(Face* face, const GCore::Vec3d& point, Tetraset& outTetras)
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
		visualize(fset, "cubeTet.off");
		visualize(tetraToDelete, "cubeTet.off");
	}

	triangulateCavity(tetraToDelete, point, outTetras);
}

void M3d::Mesher3d::insertInEdge(Edge* edge, const GCore::Vec3d& point, Tetraset& outTetras)
{
	Tetraset tetraToDelete;
	edge->getSurroundingTetras(tetraToDelete);

	triangulateCavity(tetraToDelete, point, outTetras);
}

void M3d::Mesher3d::flip(Face* face, const Edgeset& reflexEdges, Tetraset& outTetras, Faceset& deletedFaces)
{
	switch (reflexEdges.size())
	{
	case 0:
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
			visualize(tset, "cubeTet.off");
		}
		break;
	}
}

void M3d::Mesher3d::flip23(Face* face, Tetraset& outTetras, Faceset& deletedFaces)
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
		visualize(tetraToDelete, "cubeTet.off");

	triangulateCavity(tetraToDelete, refPoint, outTetras);

	//if (!outTetras.empty() && outTetras.size() != 3)
	//{
	//	visualize(outTetras, "cubeTet.off");
	//	visualize(mTess.getTetras(), "cubeTet.off");
	//}

	if (gDebug)
		visualize(outTetras, "cubeTet.off");
}

void M3d::Mesher3d::flip32(Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedFaces)
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
		visualize(tetraToDelete, "cubeTet.off");

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

	triangulateCavity(tetraToDelete, refPoint, outTetras);

	if (gDebug)
		visualize(outTetras, "cubeTet.off");
}

void M3d::Mesher3d::flip44(Face* face, Edge* reflexEdge, Tetraset& outTetras, Faceset& deletedFaces)
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
				visualize(tetraToDelete, "cubeTet.off");

			triangulateCavity(tetraToDelete, refPt, outTetras);

			if (gDebug)
				visualize(outTetras, "cubeTet.off");
		}
	}
}

void M3d::Mesher3d::flip41(/*Face* face, Tetraset& outTetras, Tetraset& deletedTetras*/)
{
	//Edge* nonReflexEdge = nullptr;
	//for (int i = 0; i < 3; i++)
	//	if (face->edge[i]->faces.size() == 3)
	//		reflexEdge = face->edge[i];
}

bool M3d::Mesher3d::locallyDelaunay(Face* face)
{
	if (face->tetra[0] == nullptr || face->tetra[1] == nullptr)
		return true;

	//for (int i = 0; i < 2; i++)
	//{
	//	Vec3d point = face->tetra[i]->opposite(face)->coord;
	//	if (!pslg.box.contains(point))
	//		return true;
	//}

	GeomTest::Pos pos = face->tetra[0]->isInsideCircumsphere(face->tetra[1]->opposite(face)->coord);
	if (pos == GeomTest::kOutside || pos == GeomTest::kOnBoundary)
		return true;
	return false;
}

void M3d::Mesher3d::reflexEdges(Face* face, Edgeset& outReflexEdges)
{
	//double vol0 = face->tetra[0]->volume();
	//double vol1 = face->tetra[1]->volume();

	for (int i = 0; i < 3; i++)
	{
		Edge* edge = face->edge[i];
		//double edgelen = edge->length();

		Vec3d t0Centroid = face->tetra[0]->centroid();
		Vec3d t1Centroid = face->tetra[1]->centroid();

		Vertex* oppV0 = face->tetra[0]->opposite(face);
		Vertex* oppV1 = face->tetra[1]->opposite(face);

		Face* f0 = mTess.findFace(edge->vertex[0], edge->vertex[1], oppV0);
		Face* f1 = mTess.findFace(edge->vertex[0], edge->vertex[1], oppV1);

		double dot0 = f0->refNormal(t0Centroid).dot(face->refNormal(t0Centroid));
		double dot1 = f1->refNormal(t1Centroid).dot(face->refNormal(t1Centroid));

		double angle0 = acos(dot0);
		double angle1 = acos(dot1);

		if (GCore::isSmallerOrEqual(angle0 + angle1, 3.142))
			outReflexEdges.emplace(edge);
		else
		{
			//double o3dt0Centroid = GeomTest::orient3d(edge->vertex[0]->coord, edge->vertex[1]->coord, oppV0->coord, t0Centroid);
			double o3doppV1 = GeomTest::orient3d(edge->vertex[0]->coord, edge->vertex[1]->coord, oppV0->coord, oppV1->coord);

			if (GCore::isZero(o3doppV1))
				outReflexEdges.emplace(edge);
		}
	}
}

void M3d::Mesher3d::findInitialCavity(const GCore::Vec3d& point, Tetraset& outTetraset)
{
	Tetra* tetra = mTess.locateTetra(point);

	if (tetra)
	{
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
				double area = (point - faceEdge->vertex[0]->coord).cross(point - faceEdge->vertex[1]->coord).magnitudeSqr();
				if (GCore::isZero<double>(area))
				{
					containerEdge = faceEdge;
					break;
				}
			}
		}

		if (containerEdge)
			containerEdge->getSurroundingTetras(outTetraset);
		else if (containerFace)
		{
			for (int i = 0; i < 2; i++)
				if (containerFace->tetra[i])
					outTetraset.emplace(containerFace->tetra[i]);
		}
		else
			outTetraset.emplace(tetra);
	}
}

bool M3d::Mesher3d::findCircumscribingTetras(const Vec3d& point, Tetraset& outTetraset)
{
	auto blindTetraLambda = [](Tetra* tetra, const Tetraset& tetraset, const Vec3d& point)->bool
	{
		for (int i = 0; i < 4; i++)
		{
			if (tetra->face[i]->tetra[0] && tetraset.find(tetra->face[i]->tetra[0]) != tetraset.end() ||
				tetra->face[i]->tetra[1] && tetraset.find(tetra->face[i]->tetra[1]) != tetraset.end())
				continue;

			Vec3d faceNorm = tetra->face[i]->normal;
			Vec3d faceCentroid = tetra->face[i]->centroid();
			Vec3d tetCenter = tetra->centroid();

			double onside = faceCentroid.direction(tetCenter).dot(faceNorm);
			if (GCore::isNegative(onside))
				faceNorm.set(-faceNorm.x, -faceNorm.y, -faceNorm.z);

			onside = faceNorm.dot(faceCentroid.direction(point));
			if (GCore::isSmaller(onside, 0.0175))
				return true;
		}
		return false;
	};

	Tetra* seedTetra = mTess.locateTetra(point);
	if (!seedTetra)
		return false;

	Tetraset tetraQueue;
	tetraQueue.emplace(seedTetra);

	while (!tetraQueue.empty())
	{
		auto it = tetraQueue.begin();
		Tetra* tetra = *it;
		tetraQueue.erase(it);

		outTetraset.emplace(tetra);
		for (int i = 0; i < 4; i++)
		{
			Tetra* otherTetra = tetra->face[i]->otherTetra(tetra);
			if (otherTetra && outTetraset.find(otherTetra) == outTetraset.end()
				&& otherTetra->isInsideCircumsphere(point) != GeomTest::kOutside)
				//if (!blindTetraLambda(otherTetra, outTetraset, point))
				tetraQueue.emplace(otherTetra);
		}
	}


	//Faceset faceset;
	//findHullFaces(outTetraset, faceset);

	//for (auto iit = faceset.begin(); iit != faceset.end(); iit++)
	//{
	//	if ((*iit)->tetra[0] == nullptr || (*iit)->tetra[1] == nullptr)
	//		visualize(faceset, "cubeTet.off");
	//}
	/*
	bool isNonConvexCavity = true;

	while (isNonConvexCavity)
	{
		isNonConvexCavity = false;

		for (auto iit = faceset.begin(); iit != faceset.end(); iit++)
		{
			Vec3d faceNorm = (*iit)->normal;

			Vec3d tetCenter;
			if ((*iit)->tetra[0] && outTetraset.find((*iit)->tetra[0]) != outTetraset.end())
				tetCenter = (*iit)->tetra[0]->centroid();
			else
				tetCenter = (*iit)->tetra[1]->centroid();

			double onside = (*iit)->vertex[0]->coord.direction(tetCenter).dot(faceNorm);
			if (GCore::isNegative(onside))
				faceNorm.set(-faceNorm.x, -faceNorm.y, -faceNorm.z);

			// point is visible to the face if this faceNorm.dot(p - faceVtx) > 0
			onside = faceNorm.dot((*iit)->vertex[0]->coord.direction(point));
			if (GCore::isSmaller(onside, 1e-5))
			{
				isNonConvexCavity = true;
				//return false;
				if (gDebug)
					visualize(faceset, "cubeTet.off");
				//isNonConvexCavity = true;
				for (int i = 0; i < 2; i++)
					if ((*iit)->tetra[i])
						outTetraset.emplace((*iit)->tetra[i]).second;
				//visualize(outTetraset, "cubeTet.off");
			}
		}

		faceset.clear();
		findHullFaces(outTetraset, faceset);

		/*
		Edgeset uniqueEdges;
		for (auto fit = faceset.begin(); fit != faceset.end(); fit++)
			for (int i = 0; i < 3; i++)
				uniqueEdges.emplace((*fit)->edge[i]);

		for (auto eit = uniqueEdges.begin(); eit != uniqueEdges.end(); eit++)
		{
			Faceset nonConformalFaces;
			for (auto fit = (*eit)->faces.begin(); fit != (*eit)->faces.end(); fit++)
				if (faceset.find(*fit) != faceset.end())
					nonConformalFaces.emplace(*fit);

			if (nonConformalFaces.size() > 2)
			{
				double maxCos = -HUGE;
				Face *validf1 = nullptr, *validf2 = nullptr;

				for (auto nit1 = nonConformalFaces.begin(); nit1 != nonConformalFaces.end(); nit1++)
				{
					Vec3d oppVec = (*nit1)->opposite(*eit)->coord;
					Vec3d norm1 = (oppVec - (*eit)->vertex[0]->coord).cross(oppVec - (*eit)->vertex[1]->coord);

					auto nit2 = nit1;
					nit2++;
					for (; nit2 != nonConformalFaces.end(); nit2++)
					{
						oppVec = (*nit2)->opposite(*eit)->coord;
						Vec3d norm2 = (oppVec - (*eit)->vertex[0]->coord).cross(oppVec - (*eit)->vertex[1]->coord);

						double nit1dotnit2 = norm1.dot(norm2);
						if (nit1dotnit2 > maxCos)
						{
							maxCos = nit1dotnit2;
							validf1 = *nit1;
							validf2 = *nit2;
						}
					}
				}

				nonConformalFaces.erase(validf1);
				nonConformalFaces.erase(validf2);

				for (auto nit = nonConformalFaces.begin(); nit != nonConformalFaces.end(); nit++)
				{
					for (int i = 0; i < 2; i++)
						if ((*nit)->tetra[i])
							outTetraset.emplace((*nit)->tetra[i]);
				}

				//visualize(nonConformalFaces, "cubeTet.off");
			}
		}
	}*/

	return true;
}

void M3d::Mesher3d::findHullFaces(const Tetraset& tetraToDelete, Faceset& outFaceset)
{
	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
	{
		Tetra* tetra = *it;
		for (int i = 0; i < 4; i++)
		{
			Face* face = tetra->face[i];

			size_t count = 0;
			for (int j = 0; j < 2; j++)
			{
				if (face->tetra[j] && tetraToDelete.find(face->tetra[j]) != tetraToDelete.end())
					count++;
			}

			if (count == 1)
				outFaceset.emplace(face);
		}
	}
}

void M3d::Mesher3d::findAllCoords(const Tetraset& tetraToDelete, Coordset& outCoords)
{
	Vertexset vertexset;
	for (auto tit = tetraToDelete.begin(); tit != tetraToDelete.end(); tit++)
		for (int i = 0; i < 4; i++)
			vertexset.emplace((*tit)->vertex[i]);

	for (auto vit = vertexset.begin(); vit != vertexset.end(); vit++)
		outCoords.push_back((*vit)->coord);
}

void M3d::Mesher3d::triangulateCavity(const Tetraset& tetraToDelete, const GCore::Vec3d& point, Tetraset& outTetras)
{
	Faceset hullfaces;
	findHullFaces(tetraToDelete, hullfaces);

	if (gDebug)
		visualize(hullfaces, "cubeTet.off");

	FaceCoordList faceCoordList;
	getFaceCoordList(hullfaces, faceCoordList);

	deleteTetras(tetraToDelete);

	if (gDebug)
		visualize(mTess.getTetras(), "cubeTet.off");

	Tetraset newTetras;
	fillvoid(faceCoordList, point, outTetras);

	if (gDebug)
	{
		visualize(outTetras, "cubeTet.off");
	}
}

void M3d::Mesher3d::deleteTetras(const Tetraset& tetraToDelete)
{
	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
		mTess.deleteTetra(*it);
}


void M3d::Mesher3d::fillvoid(const FaceCoordList& faceCoords, const GCore::Vec3d& point, Tetraset& newTetras)
{
	for (auto it = faceCoords.begin(); it != faceCoords.end(); it++)
	{
		const GCore::Vec3d& v1 = std::get<0>(*it);
		const GCore::Vec3d& v2 = std::get<1>(*it);
		const GCore::Vec3d& v3 = std::get<2>(*it);

		double vol = GeomTest::signedVolume(v1, v2, v3, point);
		if (GCore::isZero<double>(fabs(vol)))
			continue;

		Tetra* tetra = mTess.addTetra(v1, v2, v3, point);
		if (tetra)
			newTetras.emplace(tetra);
	}
}

void M3d::Mesher3d::updateBoundary(Tetraset& newTetras, Tetraset& tetrasTodel)
{
	for (auto it = tetrasTodel.begin(); it != tetrasTodel.end(); it++)
	{
		for (int i = 0; i < 4; i++)
			mBdryFaces.erase((*it)->face[i]);
	}

	for (auto it = newTetras.begin(); it != newTetras.end(); it++)
	{
		for (int i = 0; i < 4; i++)
		{
			auto it0 = mBdryVertices.find((*it)->face[i]->vertex[0]);
			auto it1 = mBdryVertices.find((*it)->face[i]->vertex[1]);
			auto it2 = mBdryVertices.find((*it)->face[i]->vertex[2]);
			auto end = mBdryVertices.end();

			if (it0 != end && it1 != end && it2 != end)
				mBdryFaces.emplace((*it)->face[i]);
		}
	}
}

void M3d::Mesher3d::triangulate(const Polygon& polygon, FaceList& outFaces)
{
	GCore::Mat4x4d xyMat;

	GCore::Vec3d center;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
		center = center + pslg.points[*it];
	center = center / (double)polygon.size();

	GCore::Vec3d norm = (pslg.points[polygon[2]] - pslg.points[polygon[1]]).cross(pslg.points[polygon[1]] - pslg.points[polygon[0]]);
	xyMat = xyMat* GCore::Mat4x4d::rotateBetweenAxes(norm, GCore::Vec3d(0, 0, 1))*GCore::Mat4x4d::translate(-center.x, -center.y, -center.z);

	M2d::Pslg pslg2d;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
	{
		GCore::Vec3d transformedPt = xyMat.transform(pslg.points[*it]);
		pslg2d.points.push_back(GCore::Vec2d(transformedPt.x, transformedPt.y));
	}

	for (size_t i = 0; i < polygon.size(); i++)
		pslg2d.boundarySegments.push_back(GCore::Vec2i(polygon[i], polygon[(i + 1) % polygon.size()]));

	M2d::Mesher2d mesher2d(pslg2d);
	mesher2d.createDelaunay2D();
	mesher2d.getFacets(outFaces);

}

bool M3d::Mesher3d::findInBetweenVertices(Vertex* v1, Vertex* v2, VertexList& outVertices, EdgeList& outEdges)
{
	if (!v1 || !v2)
		return false;

	if (v1 == v2)
		return true;

	Vertex* start = v1;
	Vec3d v1dirv2 = v1->coord.direction(v2->coord);

	for (auto it = start->edges.begin(); it != start->edges.end(); )
	{
		Vertex* otherVtx = (*it)->otherVertex(start);
		if (otherVtx == v2)
		{
			outEdges.push_back(*it);
			return true;
		}

		Vec3d v1dirov = start->coord.direction(otherVtx->coord);
		double dot = v1dirv2.dot(v1dirov);
		if (GCore::isGreater<double>(dot, 0.99))
		{
			outVertices.push_back(otherVtx);
			outEdges.push_back(*it);
			start = otherVtx;
			it = start->edges.begin();
		}
		else
		{
			it++;
		}
	}

	return false;
}

void M3d::Mesher3d::findBoundaryFaces(Faceset& outbdryFaces, FaceVtxTupleList& outMissingFaces)
{
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
	{
		std::vector<Vertex*> vertexArr(it->size());
		for (size_t i = 0; i < it->size(); i++)
			vertexArr[i] = mTess.findVertex(pslg.points[(*it)[i]]);

		Edgeset polygonEdges;
		for (size_t i = 0; i < vertexArr.size(); i++)
		{
			Vertex* v1 = vertexArr[i];
			Vertex* v2 = vertexArr[(i + 1) % vertexArr.size()];

			VertexList edgeVertexList;
			EdgeList edgeList;
			Edge* polyEdge = mTess.findEdge(v1, v2);
			if (!polyEdge)
			{
				if (!findInBetweenVertices(v1, v2, edgeVertexList, edgeList))
					continue;
				polygonEdges.insert(edgeList.begin(), edgeList.end());

				/*	edgeVertexList.push_front(v1);
					edgeVertexList.push_back(v2);

					for (auto vit = edgeVertexList.begin(); vit != edgeVertexList.end();)
					{
						Vertex* startVtx = *vit;
						if ((++vit) == edgeVertexList.end())
							break;

						Vertex* endVtx = *vit;
						polygonEdges.emplace(mTess.findEdge(startVtx, endVtx));
					}*/
			}
			else
				polygonEdges.emplace(polyEdge);
		}

		// find all mediary edge for the traversal
		Vertexset polygonVertices;
		for (auto eit = polygonEdges.begin(); eit != polygonEdges.end(); eit++)
		{
			polygonVertices.emplace((*eit)->vertex[0]);
			polygonVertices.emplace((*eit)->vertex[1]);
		}

		//std::vector<Vec3d> polygonPts(it->size());
		//for (size_t i = 0; i < it->size(); i++)
		//	polygonPts[i] = pslg.points[(*it)[i]];
		//PointInPolygon pip(polygonPts);

		Edgeset mediaryEdges;
		for (auto it1 = polygonVertices.begin(); it1 != polygonVertices.end(); it1++)
		{
			for (auto it2 = polygonVertices.begin(); it2 != polygonVertices.end(); it2++)
			{
				Edge* medEdge = mTess.findEdge(*it1, *it2);
				if (medEdge && polygonEdges.find(medEdge) == polygonEdges.end())
				{
					/*Vec3d edgeMidPt = medEdge->pointAt(0.5, 0.5);
					if (pip.inside(edgeMidPt))*/
						mediaryEdges.emplace(medEdge);
					//double a1 = (edgeMidPt - v1->coord).cross(edgeMidPt - v2->coord).magnitude();
					//double a2 = (edgeMidPt - v2->coord).cross(edgeMidPt - v3->coord).magnitude();
					//double a3 = (edgeMidPt - v3->coord).cross(edgeMidPt - v1->coord).magnitude();
					//if (GCore::isGreater<double>(a1, 0) && GCore::isGreater<double>(a2, 0) &&
					//	GCore::isGreater<double>(a3, 0) && GCore::isEqual<double>(a1 + a2 + a3, faceArea))
					//	mediaryEdges.emplace(medEdge);
				}
			}
		}

		Faceset bdryFaces;
		if (mediaryEdges.size() > 0)
		{
			// check if every mediary edge has two faces on the plane
			Edgeset widowEdges;
			for (auto ite = mediaryEdges.begin(); ite != mediaryEdges.end(); ite++)
			{
				size_t adjacentFaceCount = 0;
				for (auto itf = (*ite)->faces.begin(); itf != (*ite)->faces.end(); itf++)
				{
					Vertex* oppVtx = (*itf)->opposite(*ite);
					if (oppVtx && polygonVertices.find(oppVtx) != polygonVertices.end() /*&& pip.inside((*itf)->centroid())*/)
					{
						adjacentFaceCount++;
						bdryFaces.emplace(*itf);
					}
				}

				if (adjacentFaceCount < 2)
				{
					if (gDebug)
						visualize((*ite)->faces, "cubeTet.off");
					widowEdges.emplace(*ite);
				}
			}

			if (!widowEdges.empty())
				outMissingFaces.push_back(std::make_tuple(vertexArr[0], vertexArr[1], vertexArr[2]));
		}
		else
		{
			Face* face = mTess.findFace(vertexArr[0], vertexArr[1], vertexArr[2]);
			if (face)
				bdryFaces.emplace(face);
		}

		outbdryFaces.insert(bdryFaces.begin(), bdryFaces.end());

		if (gDebug)
		{
			visualize(bdryFaces, "cubeTet.off");

			FaceList outFaces;
			triangulate(*it, outFaces);

			Faceset polygonFaces;
			for (auto fit = outFaces.begin(); fit != outFaces.end(); fit++)
			{
				Face* face = mTess.findFace(pslg.points[(*fit).x], pslg.points[(*fit).y], pslg.points[(*fit).z]);
				if (face)
					polygonFaces.emplace(face);
			}
			visualize(polygonFaces, "cubeTet.off");
		}
	}
}

void M3d::Mesher3d::getEdgeCoordList(const Edgeset& edgeset, EdgeCoordList& outEdgeCoordList)
{
	for (auto it = edgeset.begin(); it != edgeset.end(); it++)
	{
		outEdgeCoordList.push_back(EdgeTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord)
			);
	}
}

void M3d::Mesher3d::getFaceCoordList(const Faceset& faceset, FaceCoordList& outFaceCoordList)
{
	for (auto it = faceset.begin(); it != faceset.end(); it++)
	{
		outFaceCoordList.push_back(FaceTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord)
			);
	}
}

void M3d::Mesher3d::getTetraCoordList(const Tetraset& tetraset, TetraCoordList& outTetraCoordList)
{
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
	{
		outTetraCoordList.push_back(TetraTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord,
			(*it)->vertex[3]->coord)
			);
	}
}

double M3d::Mesher3d::localFeatureSize(Vertex* vertex)
{
	double minR = HUGE;
	Edge* edge = mPslgGrid->findClosest(vertex->coord, vertex->edges);
	if (edge)
		minR = vertex->coord.shortestDistanceToLineSeg(edge->vertex[0]->coord, edge->vertex[1]->coord);

	for (auto it = vertex->edges.begin(); it != vertex->edges.end(); it++)
	{
		double d = (vertex->coord - (*it)->otherVertex(vertex)->coord).magnitude();
		if (d < minR)
			minR = d;
	}
	return minR;
}

//bool M3d::Mesher3d::isVisible(Tetra* tetra, const Vec3d& point)
//{
//	Vec3d centroid = tetra->centroid();
//	for (int i = 0; i < 4; i++)
//	{
//		if (mBdryFaces.find(tetra->face[i]) != mBdryFaces.end())
//		{
//			GeomTest::IntersectionInfo;
//			GeomTest::lineSegXTriangle
//			double t = 0;
//			if (tetra->face[i]->lineIntersects(point, centroid, t))
//				return false;
//		}
//	}
//	return true;
//}

bool M3d::Mesher3d::isEdgeEncroached(Vertex* v1, Vertex* v2, Vertexset& outEncroachingVertices)
{
	Vec3d center = (v1->coord + v2->coord) / 2.0;
	Vec3d diameter = v1->coord - v2->coord;
	double radius = diameter.magnitude() / 2.0;

	GCore::Sphere diametricalSphere(radius, center);

	mTess.locatePoints(diametricalSphere, outEncroachingVertices);
	outEncroachingVertices.erase(v1);
	outEncroachingVertices.erase(v2);

	return !outEncroachingVertices.empty();
}

bool M3d::Mesher3d::getStienerPoint(Vertex* v1, Vertex* v2, Vec3d& outStienerPoint)
{
	Vec3d stienerCenter; // to determine
	double stienerRadius; // to determine

	double v1v2edgelen = (v1->coord - v2->coord).magnitude();

	double threshold = 90 * PI / 180.0;
	Edgeset acuteEdges;
	bool isV1Acute = v1->isAcute(threshold, acuteEdges);
	acuteEdges.clear();
	bool isV2Acute = v2->isAcute(threshold, acuteEdges);

	Vertexset encroachingVertices;
	bool isEncroached = isEdgeEncroached(v1, v2, encroachingVertices);
	if (isEncroached)
	{
		// find the smallest radius for encroachment
		Vertex* encroachingVertex = nullptr;
		double maxRadius = 0;
		for (auto it = encroachingVertices.begin(); it != encroachingVertices.end(); it++)
		{
			Vec3d v1p = v1->coord - (*it)->coord;
			Vec3d v2p = v2->coord - (*it)->coord;
			Vec3d v1pxv2p = v1p.cross(v2p);
			Vec3d relCenter = (v2p*v1p.magnitudeSqr() - v1p*v2p.magnitudeSqr()).cross(v1pxv2p) / (v1pxv2p.magnitudeSqr() * 2);
			Vec3d center = relCenter + (*it)->coord;
			double radius = (center - (*it)->coord).magnitude();

			if (maxRadius < radius)
			{
				maxRadius = radius;
				encroachingVertex = *it;
			}
		}

		if (!encroachingVertex)
			return false;

		// type 2 vertex -  one of the vertices is acute
		if (isV1Acute^isV2Acute)
		{
			// rule 2
			Vertex* acuteVtx = (isV1Acute ? v1 : v2);
			Vertex* nonAcuteVtx = (acuteVtx == v1 ? v2 : v1);

			stienerCenter = acuteVtx->coord;
			stienerRadius = (acuteVtx->coord - encroachingVertex->coord).magnitude();

			outStienerPoint = stienerCenter + acuteVtx->coord.direction(nonAcuteVtx->coord)*stienerRadius;
			double sp2v2 = (outStienerPoint - nonAcuteVtx->coord).magnitude();
			double sp2e = (outStienerPoint - encroachingVertex->coord).magnitude();
			//double lfs = mLfsMap[acuteVtx];
			if (sp2v2 < sp2e)
			{
				// rule 3 here
				double sp1v1 = (outStienerPoint - acuteVtx->coord).magnitude();
				if (sp2e < 0.5*sp1v1)
					stienerRadius = sp1v1 - sp2e;
				else
					stienerRadius = 0.5*sp1v1;
			}

			outStienerPoint = stienerCenter + acuteVtx->coord.direction(nonAcuteVtx->coord)*stienerRadius;

		}
		// type 0 vertex - either both acute or both non-acute
		else
		{
			// rule 1 for insertion
			double v1e1Mag = (v1->coord - encroachingVertex->coord).magnitude();
			double v2e1Mag = (v2->coord - encroachingVertex->coord).magnitude();

			if (v1e1Mag < 0.5*v1v2edgelen)
			{
				stienerCenter = v1->coord;
				stienerRadius = v1e1Mag;
				outStienerPoint = stienerCenter + v1->coord.direction(v2->coord)*stienerRadius;
			}
			else if (v2e1Mag < 0.5*v1v2edgelen)
			{
				stienerCenter = v2->coord;
				stienerRadius = v2e1Mag;
				outStienerPoint = stienerCenter + v2->coord.direction(v1->coord)*stienerRadius;
			}
			else
			{
				stienerCenter = v1->coord;
				stienerRadius = 0.5*v1v2edgelen;
				outStienerPoint = stienerCenter + v1->coord.direction(v2->coord)*stienerRadius;
			}

		}
	}
	else
	{
		stienerCenter = v1->coord;
		stienerRadius = 0.5*v1v2edgelen;
		outStienerPoint = stienerCenter + v1->coord.direction(v2->coord)*stienerRadius;
	}

	return isEncroached;
}

void M3d::Mesher3d::findCavity(const std::vector<Vec3d>& faceBdry, Tetraset& outCavityTetras)
{
	Box3d box;
	for (auto it = faceBdry.begin(); it != faceBdry.end(); it++)
		box.extend(*it);

	Vec3d center = box.center();
	GCore::Sphere sphere(box.diagonal().magnitude() / 2.0, center);

	Vec3d norm = (faceBdry[0] - faceBdry[1]).cross(faceBdry[1] - faceBdry[2]);
	norm.normalize();

	Tetraset outTetraset;
	mTess.locateTetras(sphere, outTetraset);
	PointInPolygon pip(faceBdry);

	while (!outTetraset.empty())
	{
		Tetra* tetra = *outTetraset.begin();
		outTetraset.erase(outTetraset.begin());

		Vec3d centroid = tetra->centroid();
		Vec3d centroidProj = centroid.projectionOnPlane(norm, center);
		bool inside = pip.inside(centroidProj);
		if (inside && tetra->intersects(norm, center))
			outCavityTetras.emplace(tetra);
	}
}

void M3d::Mesher3d::separateCavities(const std::vector<Vec3d>& faceBdry, Tetraset& cavityTetras,
	Vertexset& /*outUpperCavityVertexset*/, Vertexset& /*outLowerCavityVertexset*/)
{
	// divide vertices in the cavity into two halves
	Vertexset upper, lower;

	Vec3d norm = (faceBdry[0] - faceBdry[1]).cross(faceBdry[1] - faceBdry[2]);
	for (auto tit = cavityTetras.begin(); tit != cavityTetras.end(); tit++)
	{
		for (int v = 0; v < 4; v++)
		{
			if (norm.dot(faceBdry[0].direction((*tit)->vertex[v]->coord)) > 0)
				upper.emplace((*tit)->vertex[v]);
			else
				lower.emplace((*tit)->vertex[v]);
		}
	}
}

void M3d::Mesher3d::triangulateCavity(const std::vector<GCore::Vec3d>& /*faceBdry*/, Vertexset& /*cavity*/)
{

}

bool M3d::Mesher3d::recoverByEdgeFlip(Vertex* v1, Vertex* v2)
{
	Faceset v1faces, v2faces;

	for (auto eit = v1->edges.begin(); eit != v1->edges.end(); eit++)
		v1faces.insert((*eit)->faces.begin(), (*eit)->faces.end());

	for (auto eit = v2->edges.begin(); eit != v2->edges.end(); eit++)
		v2faces.insert((*eit)->faces.begin(), (*eit)->faces.end());

	Edge* planarEdge = nullptr;
	Face* commonFace = nullptr;
	for (auto it1 = v1faces.begin(); it1 != v1faces.end(); it1++)
	{
		Edge* oppEdge1 = (*it1)->opposite(v1);
		double vol = GeomTest::orient3d(oppEdge1->vertex[0]->coord,
			oppEdge1->vertex[1]->coord, v1->coord, v2->coord);
		if (GCore::isZero(vol))
		{
			planarEdge = oppEdge1;
			commonFace = (*it1);
			break;
		}

		//for (auto it2 = v2faces.begin(); it2 != v2faces.end(); it2++)
		//{
		//	Edge* oppEdge2 = (*it2)->opposite(v2);
		//	if (oppEdge1 == oppEdge2)
		//	{
		//		
		//	}
		//}

		//if (planarEdge)
		//	break;
	}

	/*Vec3d midPt = (v1->coord + v2->coord) / 2.0;
	Tetra* tetra = mTess.locateTetra(midPt);

	Face* containingFace = nullptr;
	for (int i = 0; i < 4; i++)
	{
		Face* face = tetra->face[i];
		double o3dv1 = GeomTest::orient3d(face->vertex[0]->coord, face->vertex[1]->coord, face->vertex[2]->coord, midPt);
		if (GCore::isZero(o3dv1))
		{
			containingFace = face;
			break;
		}
	}

	if (!containingFace)
		return false;

	Edge* planarEdge = nullptr;
	for (int i = 0; i < 3; i++)
	{
		Edge* edge = containingFace->edge[i];
		Vertex* oppVtx = containingFace->opposite(edge);
		if (oppVtx == v1 || oppVtx == v2)
		{
			planarEdge = edge;
			break;
		}
	}*/

	//if (!planarEdge)
	//{
	//	if (gDebug)
	//	{
	//		Tetraset tset;
	//		for (int i = 0; i < 2; i++)
	//		{
	//			if (containingFace->tetra[i])
	//				tset.emplace(containingFace->tetra[i]);
	//		}

	//		visualize(tset, "cubeTet.off");
	//	}
	//	return false;
	//}

	//// find the second plane
	//Faceset planarFaces;
	//for (auto it = planarEdge->faces.begin(); it != planarEdge->faces.end(); it++)
	//{
	//	Vertex* oppVtx = (*it)->opposite(planarEdge);
	//	if (oppVtx == v1 || oppVtx == v2)
	//		planarFaces.emplace(*it);
	//}

	if (!planarEdge)
	{
		if (gDebug)
		{
			Faceset fset;
			fset.insert(v1faces.begin(), v1faces.end());
			fset.insert(v2faces.begin(), v2faces.end());

			visualize(fset, "cubeTet.off");
		}
		return false;
	}

	double t1 = 0, t2 = 0;
	GeomTest::Pos pos = GeomTest::linexline(v1->coord, v2->coord, planarEdge->vertex[0]->coord, planarEdge->vertex[1]->coord, t1, t2);
	if (pos != GeomTest::Pos::kIntersecting)
	{
		/*	if (gDebug)
			{
				Faceset fset(commonFaces.begin(), commonFaces.end());
				visualize(fset, "cubeTet.off");
			}*/

		return false;
	}

	Vec3d pointOnSeg = planarEdge->vertex[0]->coord + planarEdge->vertex[0]->coord.direction(planarEdge->vertex[1]->coord)*t2;

	Tetra* tetra = nullptr;
	if (commonFace->tetra[0])
		tetra = commonFace->tetra[0];
	else
		tetra = commonFace->tetra[1];

	//Tetraset newTetras;
	//tryInsertPoint(tetra, pointOnSeg, newTetras);
	insertLawson(pointOnSeg);

	//if (gDebug)
	//	visualize(newTetras, "cubeTet.off");

	VertexList midVertices;
	EdgeList edges;
	bool recovered = findInBetweenVertices(v1, v2, midVertices, edges);
	return recovered;
}

void M3d::Mesher3d::recoverEdges(size_t& outMissingCount, size_t& outRecovered)
{
	int igi = 0;
	Edgeset missingEdgeQueue;
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++, igi++)
	{
		for (size_t i = 0; i < it->size(); i++)
		{
			Vertex* v1 = mTess.findVertex(pslg.points[(*it)[i]]);
			Vertex* v2 = mTess.findVertex(pslg.points[(*it)[(i + 1) % it->size()]]);
			Edge* tedge = mTess.findEdge(v1, v2);

			if (!tedge)
			{
				VertexList midVerticesV1, midVerticesV2;
				EdgeList edgesV1, edgesV2;

				bool edgeExists = findInBetweenVertices(v1, v2, midVerticesV1, edgesV1);
				if (!edgeExists)
					edgeExists = findInBetweenVertices(v2, v1, midVerticesV2, edgesV2);
				if (!edgeExists)
				{
					outMissingCount++;
					v1 = midVerticesV1.empty() ? v1 : midVerticesV1.back();
					v2 = midVerticesV2.empty() ? v2 : midVerticesV2.back();

					//bool recoverd = recoverByEdgeFlip(v1, v2);
					//if (!recoverd)
					recoverEdge(v1, v2);
					outRecovered++;
					std::cout << outRecovered << std::endl;
				}
			}
		}
	}
}

void M3d::Mesher3d::recoverEdge(Vertex* v1, Vertex* v2)
{
	if (!v1 || !v2 || v1 == v2)
		return;

	Vec3d stienerPt;
	/*bool encroached =*/ getStienerPoint(v1, v2, stienerPt);
	//if (encroached)
	{
		Vec3d coordv1 = v1->coord;
		Vec3d coordv2 = v2->coord;

		insertLawson(stienerPt);
		v1 = mTess.findVertex(coordv1);
		Vertex* stienerVertex = mTess.findVertex(stienerPt);

		VertexList midVerticesv, midVerticesStiener;
		EdgeList edgesv, edgesStiener;

		bool edgeExists = findInBetweenVertices(v1, stienerVertex, midVerticesv, edgesv);
		if (!edgeExists)
			edgeExists = findInBetweenVertices(stienerVertex, v1, midVerticesStiener, edgesStiener);
		if (!edgeExists)
		{
			v1 = midVerticesv.empty() ? v1 : midVerticesv.back();
			stienerVertex = midVerticesStiener.empty() ? stienerVertex : midVerticesStiener.back();
			recoverEdge(v1, stienerVertex);
		}

		v2 = mTess.findVertex(coordv2);
		stienerVertex = mTess.findVertex(stienerPt);

		midVerticesStiener.clear();
		midVerticesv.clear();
		edgesv.clear();
		edgesStiener.clear();
		edgeExists = findInBetweenVertices(stienerVertex, v2, midVerticesStiener, edgesStiener);
		if (!edgeExists)
			edgeExists = findInBetweenVertices(v2, stienerVertex, midVerticesv, edgesv);
		if (!edgeExists)
		{
			v2 = midVerticesv.empty() ? v2 : midVerticesv.back();
			stienerVertex = midVerticesStiener.empty() ? stienerVertex : midVerticesStiener.back();
			recoverEdge(stienerVertex, v2);
		}
	}
}

void M3d::Mesher3d::recoverFaces()
{

}

void M3d::Mesher3d::removExteriorTetras()
{
	// clear the tetras out of the bbox
	//cleanTetrasOutOfDomainBox();

	Faceset boundaryFaces;
	FaceVtxTupleList missingFaces;
	//getBoundaryFaces(boundaryFaces);
	findBoundaryFaces(boundaryFaces, missingFaces);
	if (missingFaces.empty())
	{
		visualize(boundaryFaces, "cubetet.off");

		double dgnlLen = pslg.box.diagonal().magnitude() / 10;
		Vec3d extPt = (pslg.box.lower - Vec3d(dgnlLen, dgnlLen, dgnlLen));
		Tetra* extTetra = mTess.locateTetra(extPt);

		Tetraset exteriorTetras;
		detectExteriorTetras(extTetra, boundaryFaces, exteriorTetras);

		deleteTetras(exteriorTetras);
	}
}

void M3d::Mesher3d::detectExteriorTetras(Tetra* extTetra, const Faceset& boundaryFaces, Tetraset& outExteriorTetras)
{
	Tetraset tetraToTest;
	tetraToTest.emplace(extTetra);

	for (auto it = tetraToTest.begin(); it != tetraToTest.end(); )
	{
		Tetra* seedTetra = *it;
		outExteriorTetras.emplace(seedTetra);
		tetraToTest.erase(it);

		for (size_t i = 0; i < 4; i++)
		{
			if (boundaryFaces.find(seedTetra->face[i]) == boundaryFaces.end())
			{
				Tetra* otherTetra = seedTetra->face[i]->otherTetra(seedTetra);
				if (otherTetra != nullptr && outExteriorTetras.find(otherTetra) == outExteriorTetras.end())
					tetraToTest.emplace(otherTetra);
			}
		}

		it = tetraToTest.begin();
	}
}

bool M3d::Mesher3d::checkIfDelaunay(const Tetraset& tetraset)
{
	Faceset faces;
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
		for (int i = 0; i < 4; i++)
			faces.emplace((*it)->face[i]);

	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		Face* face = *it;
		if (face->tetra[0] == nullptr || face->tetra[1] == nullptr)
			continue;

		Vertex* oppVtx = face->tetra[1]->opposite(face);
		bool b1 = face->tetra[0]->isInsideCircumsphere(oppVtx->coord) != GeomTest::kOutside;
		oppVtx = face->tetra[0]->opposite(face);
		bool b2 = face->tetra[1]->isInsideCircumsphere(oppVtx->coord) != GeomTest::kOutside;
		if (b1^b2)
		{
			Tetraset tset;
			tset.emplace(face->tetra[0]);
			tset.emplace(face->tetra[1]);
			visualize(tset, "cubeTet.off");
			visualize(tetraset, "cubeTet.off");

			return false;
		}
	}
	return true;
}

bool M3d::Mesher3d::checkFaceSanity()
{
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
	{
		size_t vsize = it->size();
		Edgeset edges;
		for (size_t i = 0; i < vsize; i++)
			for (size_t j = 0; j < vsize; j++)
			{
				Edge* edge = mTess.findEdge(pslg.points[(*it)[i]], pslg.points[(*it)[j]]);
				if (edge)
					edges.emplace(edge);
			}

		assert(edges.size() == ((vsize - 2) * 2 + 1));
	}
	return false;
}


void M3d::Mesher3d::getBoundaryFaces(Faceset& outFaceset)
{
	//for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
	//{
	//	Vertex* v1 = mTess.findVertex(pslg.points[(*it).x]);
	//	Vertex* v2 = mTess.findVertex(pslg.points[(*it).y]);
	//	Vertex* v3 = mTess.findVertex(pslg.points[(*it).z]);

	//	if (v1 && v2 && v3)
	//	{
	//		Face* face = mTess.findFace(v1, v2, v3);
	//		if (face)
	//			outFaceset.emplace(face);
	//	}
	//}

	const Faceset& faceset = mTess.getFaces();
	for (auto it = faceset.begin(); it != faceset.end(); it++)
	{
		Vertex* v0 = mPslgTess.findVertex((*it)->vertex[0]->coord);
		Vertex* v1 = mPslgTess.findVertex((*it)->vertex[1]->coord);
		Vertex* v2 = mPslgTess.findVertex((*it)->vertex[2]->coord);

		Edge* edge0 = mPslgTess.findEdge(v0, v1);
		Edge* edge1 = mPslgTess.findEdge(v1, v2);
		Edge* edge2 = mPslgTess.findEdge(v2, v0);

		if (!edge0 || !edge1 || !edge2)
			continue;

		outFaceset.emplace(*it);
	}
}

//void M3d::Mesher3d::getBoundaryFaces(Faceset& outFaceset)
//{
//	for (auto it = pslg.indices.begin(); it != pslg.indices.end(); it++)
//	{
//		size_t psize = outFaceset.size();
//
//		std::vector<Vec3d> polygon(it->size());
//		size_t i = 0;
//		for (auto iit = it->begin(); iit != it->end(); iit++, i++)
//			polygon[i] = pslg.vertices[*iit];
//
//		PointInPolygon pip(polygon);
//
//		for (auto iit = it->begin(); iit != it->end(); iit++)
//			for (auto jit = it->begin(); jit != it->end(); jit++)
//				for (auto kit = it->begin(); kit != it->end(); kit++)
//				{
//					Face* face = mTess.findFace(
//						pslg.vertices[(*iit)],
//						pslg.vertices[(*jit)],
//						pslg.vertices[(*kit)]);
//
//					if (face)
//					{
//						Vec3d centroid = face->centroid();
//						if (pip.inside(centroid))
//							outFaceset.emplace(face);
//					}
//				}
//
//		if (psize == outFaceset.size())
//			continue;
//	}
//}

void M3d::Mesher3d::cleanTetrasOutOfDomainBox()
{
	Box3d box = pslg.box;
	box.scale(1.05, 1.05, 1.05);

	Tetraset tetrasToDel;
	const Tetraset& tetras = mTess.getTetras();
	for (auto it = tetras.begin(); it != tetras.end(); it++)
	{
		for (int i = 0; i < 4; i++)
		{
			if (!box.contains((*it)->vertex[i]->coord))
			{
				tetrasToDel.emplace(*it);
				break;
			}
		}
	}

	deleteTetras(tetrasToDel);
}

void M3d::Mesher3d::visualize(const Faceset& faceset, const char* filename)
{
	std::ofstream file;
	file.open(filename);

	const auto& vset = mTess.getVertices();

	file << "OFF" << std::endl;
	file << vset.size() << " " << faceset.size() << " 0" << std::endl;

	std::unordered_map<Vertex*, int> v2iMap;

	int v = 0;
	for (auto const vertex : vset)
	{
		file << vertex->coord.x << " " << vertex->coord.y << " " << vertex->coord.z << std::endl;
		v2iMap.emplace(vertex, v++);
	}

	for (auto const face : faceset)
	{
		file << "3 "
			<< v2iMap[face->edge[0]->vertex[0]] << " "
			<< v2iMap[face->edge[0]->vertex[1]] << " "
			<< v2iMap[face->opposite(face->edge[0])]
			<< std::endl;

	}

	file.close();
	system("start meshlab.exe cubeTet.off cubeTet.off");
}

void M3d::Mesher3d::visualize(const Tetraset& tetraset, const char* filename)
{
	Faceset faceset;
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
	{
		for (int f = 0; f < 4; f++)
		{
			faceset.emplace((*it)->face[f]);
		}
	}

	visualize(faceset, filename);
}




