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
#include "Helper.h"
#include "PointInsertion.h"
#include "Mesher2d.h"
#include "Mesher3d.h"

using namespace GCore;
using namespace M3d;

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
	Helper::visualizeFaces(mTess, mTess.getFaces(), "cubetet.off");
	//checkFaceSanity();
	//bool valid = false;

	std::cout << "points : " << mTess.getVertices().size() - 4 << std::endl;

	int zeroVol = Helper::checkZeroVolume(mTess);
	std::cout << "zerovol : " << zeroVol << std::endl;

	bool sane = Helper::checkFaceSanity(mTess);
	std::cout << sane << std::endl;

	size_t missing = 0, recovered = 0;
	do {
		missing = 0, recovered = 0;
		recoverEdges(missing, recovered);
	} while (missing > 0);

	Faceset boundaryFaces;
	recoverFaces(boundaryFaces);
	//visualize(mTess.getFaces(), "cubetet.off");
	Helper::visualizeFaces(mTess, boundaryFaces, "cubetet.off");

	removExteriorTetras(boundaryFaces);
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
	PointInsertion inserter(mTess);

	std::list<size_t> randList;
	size_t i = 0;
	for (auto it = pslg.points.begin(); it != pslg.points.end(); it++)
		randList.push_back(i++);

	//randList = { 4,17,21,12,19,16,1,14,9,0,15,6,8,18,3,13,7,5,2,20,11,10 };

	//std::srand(unsigned(std::time(0)));
	//std::random_shuffle(randList.begin(), randList.end());

	while (!randList.empty())
	{
		//insertLawson(pslg.points[*it]);
		int idx = randList.front();
		randList.pop_front();
		if (!inserter.insertLawson(pslg.points[idx]))
		{
			randList.push_back(idx);
		}

		/*if (!checkFaceSanity())
		{
		std::cout << "insane:" << gi << std::endl;
		}
		*/

		//if (gi == 64)
		//	visualize(mTess.getTetras(), "cubeTet.off");
		if (gi % 10 == 0)
			std::cout << gi << std::endl;
		gi++;
	}
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

void M3d::Mesher3d::findNonlinearPoints(const VertexArr& polygon, VertexArr& outVertices)
{
	Vertex * vp = nullptr, *vi = nullptr, *vn = nullptr;
	size_t size = polygon.size();
	for (size_t i = 0; i < polygon.size(); i++)
	{
		size_t p = i - 1;
		if (p > size)
			p = size - 1;

		size_t n = (i + 1) % size;

		vp = polygon[p];
		vi = polygon[i];
		vn = polygon[n];

		double o3d = GeomTest::unSignedArea3d(vp->coord, vi->coord, vn->coord);
		if (GCore::isZero(o3d))
			continue;

		outVertices.push_back(vp);
		outVertices.push_back(vi);
		outVertices.push_back(vn);

		break;
	}
}

void M3d::Mesher3d::findOrderedPoints(const Edgeset& polygonEdges, VertexArr& outOrderedPts)
{
	Edgeset edgesToVisit(polygonEdges.begin(), polygonEdges.end());

	Vertex* curVertex = (*edgesToVisit.begin())->vertex[0];
	Vertex* nextVertex = nullptr;
	outOrderedPts.push_back(curVertex);

	while (!edgesToVisit.empty())
	{
		for (auto eit = curVertex->edges.begin(); eit != curVertex->edges.end(); eit++)
		{
			if (edgesToVisit.find(*eit) != edgesToVisit.end())
			{
				nextVertex = (*eit)->otherVertex(curVertex);
				edgesToVisit.erase(*eit);
				break;
			}
		}

		if (!nextVertex)
			break;

		outOrderedPts.push_back(nextVertex);
		curVertex = nextVertex;
		nextVertex = nullptr;

		if (outOrderedPts.front() == outOrderedPts.back())
			break;

	}

	if (outOrderedPts.front() == outOrderedPts.back())
		outOrderedPts.pop_back();
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

void M3d::Mesher3d::triangulate(const VertexArr& polygon, Faceset& outFaces)
{
	GCore::Mat4x4d xyMat;

	GCore::Vec3d center;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
		center = center + (*it)->coord;
	center = center / (double)polygon.size();

	VertexArr nonlinearPts;
	findNonlinearPoints(polygon, nonlinearPts);

	GCore::Vec3d norm = (nonlinearPts[0]->coord - nonlinearPts[1]->coord).cross(nonlinearPts[2]->coord - nonlinearPts[1]->coord);
	xyMat = xyMat* GCore::Mat4x4d::rotateBetweenAxes(norm, GCore::Vec3d(0, 0, 1))*GCore::Mat4x4d::translate(-center.x, -center.y, -center.z);

	M2d::Pslg pslg2d;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
	{
		GCore::Vec3d transformedPt = xyMat.transform((*it)->coord);
		pslg2d.points.push_back(GCore::Vec2d(transformedPt.x, transformedPt.y));
	}

	for (size_t i = 0; i < polygon.size(); i++)
		pslg2d.boundarySegments.push_back(GCore::Vec2i(i, (i + 1) % polygon.size()));

	pslg2d.calculateBBox();

	M2d::Mesher2d mesher2d(pslg2d);
	mesher2d.createDelaunay2D();

	//mesher2d.writeToOff("cubeTet.off");
	//system("start meshlab.exe cubeTet.off");

	FaceList faces;
	mesher2d.getFacets(faces);

	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		Face* face = mTess.addFace(polygon[(*it).x], polygon[(*it).y], polygon[(*it).z]);
		if (face)
			outFaces.emplace(face);
	}
}


bool M3d::Mesher3d::findInBetweenVertices(const Vertex* v1, const Vertex* v2, VertexList& outVertices, EdgeList& outEdges)
{
	if (!v1 || !v2)
		return false;

	if (v1 == v2)
		return true;

	const Vertex* start = v1;
	Vec3d v1dirv2 = v1->coord.direction(v2->coord);
	double distv1v2 = (v1->coord - v2->coord).magnitude();

	for (auto it = start->edges.begin(); it != start->edges.end(); )
	{
		Vertex* otherVtx = (*it)->otherVertex(start);
		if (otherVtx == v2)
		{
			outEdges.push_back(*it);
			return true;
		}

		Vec3d v1dirov = start->coord.direction(otherVtx->coord);
		double distv1it = (v1->coord - otherVtx->coord).magnitude();
		double dot = v1dirv2.dot(v1dirov);
		if (GCore::isGreater<double>(dot, 0.99999))
		{
			if (GCore::isGreater(distv1it, distv1v2))
			{
				it++;
				continue;
			}

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

void M3d::Mesher3d::findBoundaryFaces(Faceset& outbdryFaces, FaceVertexArr& outMissingFaces)
{
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
	{
		VertexArr vertexArr(it->size());
		for (size_t i = 0; i < it->size(); i++)
			vertexArr[i] = mTess.findVertex(pslg.points[(*it)[i]]);

		Face* bdryface = nullptr;
		if (it->size() == 3)
		{
			bdryface = mTess.findFace(vertexArr[0], vertexArr[1], vertexArr[2]);
		}

		if (bdryface)
		{
			outbdryFaces.emplace(bdryface);
			continue;
		}


		Edgeset polygonEdges;
		findPolygonEdges(vertexArr, polygonEdges);

		Edgeset mediaryEdges;
		findMediaryEdges(vertexArr, polygonEdges, mediaryEdges);

		if (mediaryEdges.empty())
		{
			outMissingFaces.push_back(vertexArr);
			continue;
		}

		Faceset bdryFaces;
		Edgeset widowEdges;
		findPolygonFaces(polygonEdges, mediaryEdges, bdryFaces, widowEdges);
		if (!widowEdges.empty())
			outMissingFaces.push_back(vertexArr);

		outbdryFaces.insert(bdryFaces.begin(), bdryFaces.end());

		if (gDebug)
		{
			Helper::visualizeFaces(mTess, bdryFaces, "cubeTet.off");

			FaceList outFaces;
			triangulate(*it, outFaces);

			Faceset polygonFaces;
			for (auto fit = outFaces.begin(); fit != outFaces.end(); fit++)
			{
				Face* face = mTess.findFace(pslg.points[(*fit).x], pslg.points[(*fit).y], pslg.points[(*fit).z]);
				if (face)
					polygonFaces.emplace(face);
			}
			Helper::visualizeFaces(mTess, polygonFaces, "cubeTet.off");
		}
	}
}

void M3d::Mesher3d::findPolygonEdges(const VertexArr& vertexArr, Edgeset& outEdgeset)
{
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
			{
				outEdgeset.clear();
				return;
			}
			outEdgeset.insert(edgeList.begin(), edgeList.end());
		}
		else
			outEdgeset.emplace(polyEdge);
	}
}

void M3d::Mesher3d::findMediaryEdges(const VertexArr& vertexArr, const Edgeset& polygonEdges, Edgeset& mediaryEdges)
{
	// find all mediary edge for the traversal
	Vertexset polygonVertices;
	for (auto eit = polygonEdges.begin(); eit != polygonEdges.end(); eit++)
	{
		polygonVertices.emplace((*eit)->vertex[0]);
		polygonVertices.emplace((*eit)->vertex[1]);
	}

	std::vector<Vec3d> polygonPts;
	for (auto it = vertexArr.begin(); it != vertexArr.end(); it++)
		polygonPts.push_back((*it)->coord);
	PointInPolygon pip(polygonPts);

	for (auto it1 = polygonVertices.begin(); it1 != polygonVertices.end(); it1++)
	{
		for (auto it2 = polygonVertices.begin(); it2 != polygonVertices.end(); it2++)
		{
			Edge* medEdge = mTess.findEdge(*it1, *it2);
			if (medEdge && polygonEdges.find(medEdge) == polygonEdges.end())
			{
				Vec3d edgeMidPt = medEdge->pointAt(0.5, 0.5);
				if (pip.inside(edgeMidPt))
					mediaryEdges.emplace(medEdge);
			}
		}
	}
}

void M3d::Mesher3d::findPolygonFaces(const Edgeset& polygonEdges, const Edgeset& mediaryEdges,
	Faceset& outFaceset, Edgeset& outWidowEdges)
{
	// find all mediary edge for the traversal
	Vertexset polygonVertices;
	for (auto eit = polygonEdges.begin(); eit != polygonEdges.end(); eit++)
	{
		polygonVertices.emplace((*eit)->vertex[0]);
		polygonVertices.emplace((*eit)->vertex[1]);
	}

	if (mediaryEdges.empty())
		return;

	VertexArr orderedVertices;
	findOrderedPoints(polygonEdges, orderedVertices);

	std::vector<Vec3d> polygonPts;
	for (auto it = orderedVertices.begin(); it != orderedVertices.end(); it++)
		polygonPts.push_back((*it)->coord);
	PointInPolygon pip(polygonPts);

	// check if every mediary edge has two faces on the plane
	findWidowEdges(pip, mediaryEdges, polygonVertices, outFaceset, outWidowEdges, 2);
	findWidowEdges(pip, polygonEdges, polygonVertices, outFaceset, outWidowEdges, 1);

}

void M3d::Mesher3d::findWidowEdges(const PointInPolygon& pip, const Edgeset& edges, const Vertexset& polygonVertices,
	Faceset& outFaceset, Edgeset& outWidowEdges, size_t faceCountCriteria)
{
	// check if every mediary edge has two faces on the plane
	for (auto ite = edges.begin(); ite != edges.end(); ite++)
	{
		size_t adjacentFaceCount = 0;
		for (auto itf = (*ite)->faces.begin(); itf != (*ite)->faces.end(); itf++)
		{
			Vertex* oppVtx = (*itf)->opposite(*ite);
			if (oppVtx && polygonVertices.find(oppVtx) != polygonVertices.end() && pip.inside((*itf)->centroid()))
			{
				adjacentFaceCount++;
				outFaceset.emplace(*itf);
			}
		}

		if (adjacentFaceCount < faceCountCriteria)
		{
			if (gDebug)
				Helper::visualizeFaces(mTess, (*ite)->faces, "cubeTet.off");
			outWidowEdges.emplace(*ite);
		}
	}
}

void M3d::Mesher3d::findMissingFaces(const Edgeset& polygonEdges, const Edgeset& widowEdges, FaceVertexArr& missingFaces)
{
	Edgeset edgeset(widowEdges.begin(), widowEdges.end());

	while (!edgeset.empty() && edgeset.size() > 2)
	{
		VertexArr orderedVertices;
		findOrderedPoints(edgeset, orderedVertices);

		for (size_t i = 0; i < orderedVertices.size(); i++)
		{
			Edge* edge = mTess.findEdge(orderedVertices[i], orderedVertices[(i + 1) % orderedVertices.size()]);
			if (edge)
				edgeset.erase(edge);
		}

		missingFaces.push_back(orderedVertices);

		//TODO
		if (polygonEdges.size() > 0)
			continue;

		/*	for (auto wit = cyclicEdges.begin(); wit != cyclicEdges.end(); wit++)
		{
		for (int i = 0; i < 2; i++)
		{
		Vertex* vertex = (*wit)->vertex[i];
		for (auto eit = vertex->edges.begin(); eit != vertex->edges.end(); eit++)
		{
		if (polygonEdges.find(*eit) != polygonEdges.end())
		{
		VertexArr faceVertices = {
		(*wit)->vertex[0],
		(*wit)->vertex[1],
		(*eit)->otherVertex(vertex)
		};

		Face* face = mTess.findFace(faceVertices[0], faceVertices[1], faceVertices[2]);
		if (!face)
		missingFaces.push_back(faceVertices);
		}
		}
		}
		}*/
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

	double threshold = 60 * PI / 180.0;
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

	double dist = (stienerCenter - v1->coord).magnitude() + (stienerCenter - v2->coord).magnitude();
	if (v1v2edgelen < dist)
		return false;
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

void M3d::Mesher3d::triangulateCavity(Faceset& faces, Tetraset& outTetras)
{
	if (gDebug)
	{
		Helper::visualizeFaces(mTess, faces, "cubeTet.off");
	}

	if (faces.size() == 4)
	{
		Vertexset vertices;
		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			for (int i = 0; i < 3; i++)
				vertices.emplace((*it)->vertex[i]);
		}

		VertexArr vertexArr(vertices.begin(), vertices.end());
		Tetra* tetra = mTess.addTetra(vertexArr[0], vertexArr[1], vertexArr[2], vertexArr[3]);
		if (tetra)
			outTetras.emplace(tetra);
		return;
	}

	Vec3d center;
	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		center = center + (*it)->centroid();
	}
	center = center / faces.size();

	FaceCoordList faceCoordList;
	Helper::getFaceCoordList(faces, faceCoordList);

	Tetraset newTetras;
	Helper::fillvoid(mTess, faceCoordList, center, outTetras);

	/*
	Edgeset edges;
	for (auto fit = faces.begin(); fit != faces.end(); fit++)
	{
	edges.emplace((*fit)->edge[0]);
	edges.emplace((*fit)->edge[1]);
	edges.emplace((*fit)->edge[2]);
	}

	Vertexset vertices;
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
	vertices.emplace((*eit)->vertex[0]);
	vertices.emplace((*eit)->vertex[1]);
	}

	Tetra* newTetra = nullptr;
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
	Edge* edge = *eit;
	FaceArr faceArr;
	for (auto fit = edge->faces.begin(); fit != edge->faces.end(); fit++)
	{
	if (faces.find(*fit) != faces.end())
	faceArr.push_back(*fit);
	}

	VertexArr vertArr;
	for (int i = 0; i < 2; i++)
	vertArr.push_back(faceArr[i]->opposite(edge));

	double o3d = GeomTest::orient3d(edge->vertex[0]->coord, edge->vertex[1]->coord,
	vertArr[0]->coord, vertArr[1]->coord);
	if (GCore::isZero(o3d))
	continue;

	bool nonConvexEar = false;
	Vec3d tetCenter = (vertArr[0]->coord + vertArr[1]->coord + edge->vertex[0]->coord + edge->vertex[1]->coord) / 4.0;
	for (int i = 0; i < 2; i++)
	{
	Face* face = faceArr[i];

	Tetra* sideTetra = nullptr;
	if (face->tetra[0])
	sideTetra = face->tetra[0];
	else
	sideTetra = face->tetra[1];

	if (sideTetra && sideTetra->contains(tetCenter))
	nonConvexEar = true;
	}

	if (nonConvexEar)
	continue;

	Vec3d center;
	double radius;
	GeomTest::findCircumsphere(edge->vertex[0]->coord, edge->vertex[1]->coord,
	vertArr[0]->coord, vertArr[1]->coord, center, radius);

	Vertexset otherVertices(vertices.begin(), vertices.end());
	otherVertices.erase(edge->vertex[0]);
	otherVertices.erase(edge->vertex[1]);
	otherVertices.erase(vertArr[0]);
	otherVertices.erase(vertArr[1]);

	bool isEmptySphere = true;
	for (auto vit = otherVertices.begin(); vit != otherVertices.end(); vit++)
	{
	double distSqr = ((*vit)->coord - center).magnitudeSqr();
	if (GCore::isSmaller(distSqr, radius*radius))
	{
	isEmptySphere = false;
	break;
	}
	}

	if (isEmptySphere)
	{
	newTetra = mTess.addTetra(edge->vertex[0], edge->vertex[1], vertArr[0], vertArr[1]);
	if (newTetra)
	{
	outTetras.emplace(newTetra);
	for (int i = 0; i < 4; i++)
	{
	if (!faces.empty() && faces.find(newTetra->face[i]) != faces.end())
	faces.erase(newTetra->face[i]);
	else
	faces.emplace(newTetra->face[i]);
	}
	}
	break;
	}
	}

	if (newTetra)
	triangulateCavity(faces, outTetras);
	*/
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

			Helper::visualizeFaces(mTess, fset, "cubeTet.off");
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
	PointInsertion inserter(mTess);
	inserter.insertLawson(pointOnSeg);

	//if (gDebug)
	//	visualize(newTetras, "cubeTet.off");

	VertexList midVertices;
	EdgeList edges;
	bool recovered = findInBetweenVertices(v1, v2, midVertices, edges);
	return recovered;
}

void M3d::Mesher3d::recoverEdges(size_t& outMissingCount, size_t& outRecovered)
{
	for (auto it = pslg.boundaryFaces.begin(); it != pslg.boundaryFaces.end(); it++)
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

				bool edgev1v2 = findInBetweenVertices(v1, v2, midVerticesV1, edgesV1);
				bool edgev2v1 = findInBetweenVertices(v2, v1, midVerticesV2, edgesV2);

				if (edgev1v2^edgev2v1)
				{
					if (gDebug)
					{
						Edge* edge = mTess.findEdge(v1, v2);
						if (edge)
						{
							Tetraset outTetras;
							edge->getSurroundingTetras(outTetras);
							Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
						}

					}
				}

				if (!edgev1v2 && !edgev1v2)
				{
					outMissingCount++;
					v1 = midVerticesV1.empty() ? v1 : midVerticesV1.back();
					v2 = midVerticesV2.empty() ? v2 : midVerticesV2.back();

					//bool recoverd = recoverByEdgeFlip(v1, v2);
					//if (outRecovered < 33)
					{
						gStackCount = 0;
						if (recoverEdge(v1, v2))
						{
							/*if (gStackCount > 10)
							{
							visualize(v1, v2, "cubeTet.off");
							}*/
							outRecovered++;
							std::cout << outRecovered << std::endl;
						}
					}
				}
			}
		}
	}
}

bool M3d::Mesher3d::recoverEdge(Vertex* v1, Vertex* v2)
{
	if (!v1 || !v2 || v1 == v2)
		return true;

	//if (gStackCount > 8)
	//{
	//	visualize(v1, v2, "cubeTet.off");
	//	//	//return;
	//}
	gStackCount++;
	bool recovered = false;


	Vec3d stienerPt;
	/*bool encroached = */getStienerPoint(v1, v2, stienerPt);
	//if (encroached)
	{
		Vec3d coordv1 = v1->coord;
		Vec3d coordv2 = v2->coord;

		//insertWatson(stienerPt);
		PointInsertion inserter(mTess);
		bool validInsert = inserter.insertLawson(stienerPt);
		if (!validInsert)
		{
			return false;
			//visualize(v1, v2, "cubeTet.off");
		}
		v1 = mTess.findVertex(coordv1);
		Vertex* stienerVertex = mTess.findVertex(stienerPt);

		VertexList midVerticesv, midVerticesStiener;
		EdgeList edgesv, edgesStiener;

		bool edgev1st = findInBetweenVertices(v1, stienerVertex, midVerticesv, edgesv);
		bool edgestv1 = findInBetweenVertices(stienerVertex, v1, midVerticesStiener, edgesStiener);

		if (edgev1st^edgestv1)
		{
			if (gDebug)
			{
				Edge* edge = mTess.findEdge(v1, v2);
				if (edge)
				{
					Tetraset outTetras;
					edge->getSurroundingTetras(outTetras);
					Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
				}

			}
		}

		if (!edgev1st && !edgestv1)
		{
			v1 = midVerticesv.empty() ? v1 : midVerticesv.back();
			stienerVertex = midVerticesStiener.empty() ? stienerVertex : midVerticesStiener.back();
			recovered = recoverEdge(v1, stienerVertex);
		}

		v2 = mTess.findVertex(coordv2);
		stienerVertex = mTess.findVertex(stienerPt);

		midVerticesStiener.clear();
		midVerticesv.clear();
		edgesv.clear();
		edgesStiener.clear();

		bool edgestv2 = findInBetweenVertices(stienerVertex, v2, midVerticesStiener, edgesStiener);
		bool edgev2st = findInBetweenVertices(v2, stienerVertex, midVerticesv, edgesv);

		if (edgestv2^edgev2st)
		{
			if (gDebug)
			{
				Edge* edge = mTess.findEdge(v1, v2);
				if (edge)
				{
					Tetraset outTetras;
					edge->getSurroundingTetras(outTetras);
					Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
				}

			}
		}

		if (!edgestv2 && !edgev2st)
		{
			v2 = midVerticesv.empty() ? v2 : midVerticesv.back();
			stienerVertex = midVerticesStiener.empty() ? stienerVertex : midVerticesStiener.back();
			recovered = recoverEdge(stienerVertex, v2);
		}
	}

	return recovered;
}

void M3d::Mesher3d::recoverFaces(Faceset& outBoundaryFaces)
{
	FaceVertexArr missingPolygon;
	findBoundaryFaces(outBoundaryFaces, missingPolygon);

	FaceVertexArr missingFaces;
	for (auto it = missingPolygon.begin(); it != missingPolygon.end(); it++)
	{
		Edgeset polygonEdges;
		findPolygonEdges(*it, polygonEdges);

		Edgeset mediaryEdges;
		findMediaryEdges(*it, polygonEdges, mediaryEdges);

		if (mediaryEdges.empty())
		{
			missingFaces.push_back(*it);
			continue;
		}

		Faceset bdryFaces;
		Edgeset widowEdges;
		findPolygonFaces(polygonEdges, mediaryEdges, bdryFaces, widowEdges);

		findMissingFaces(polygonEdges, widowEdges, missingFaces);

	}

	Faceset recoveredFaces;
	FaceVertexArr unRecoveredFaces;
	for (auto it = missingFaces.begin(); it != missingFaces.end(); it++)
	{
		Faceset newFaces;
		recoverFace(*it, newFaces);
		recoveredFaces.insert(newFaces.begin(), newFaces.end());
	}

	missingPolygon.clear();
	outBoundaryFaces.clear();
	findBoundaryFaces(outBoundaryFaces, missingPolygon);
	outBoundaryFaces.insert(recoveredFaces.begin(), recoveredFaces.end());
}

void M3d::Mesher3d::recoverFace(const VertexArr& faceVertices, Faceset& outFaces)
{
	//Vec3d centroid = (faceVertices[0]->coord + faceVertices[1]->coord + faceVertices[2]->coord) / 3.0;
	//Tetra* tetra = mTess.locateTetra(centroid);

	//if (!tetra)
	//	return;

	Tetraset outTetras;
	findIntersectingTetras(faceVertices, outTetras);

	if (outTetras.empty())
	{
		return;
		//Face* f = mTess.addFace(faceVertices[0], faceVertices[1], faceVertices[2]);
		//Faceset fset;
		//fset.emplace(f);
		//visualize(fset, "cubeTet.off");
	}


	if (gDebug)
	{
		Helper::visualizeTetras(mTess, outTetras, "cubeTet.off");
	}

	// find upper and lower cavities
	Faceset hullFaces, facesToDelete;
	Helper::findHullFaces(outTetras, hullFaces, facesToDelete);

	Faceset upperhull, lowerhull;
	for (auto fit = hullFaces.begin(); fit != hullFaces.end(); fit++)
	{
		Vec3d faceCentroid = (*fit)->centroid();
		double o3dCentroid = GeomTest::orient3d(faceVertices[0]->coord, faceVertices[1]->coord, faceVertices[2]->coord, faceCentroid);
		if (o3dCentroid > 0)
			upperhull.emplace(*fit);
		else
			lowerhull.emplace(*fit);
	}

	Edgeset polygonEdges;
	findPolygonEdges(faceVertices, polygonEdges);

	Faceset faces;
	if (polygonEdges.size() > 3)
	{
		VertexArr polygonVertices;
		findOrderedPoints(polygonEdges, polygonVertices);

		triangulate(polygonVertices, faces);

		//visualize(faces, "cubeTet.off");
	}
	else
	{
		Face* sface = mTess.addFace(faceVertices[0], faceVertices[1], faceVertices[2]);
		faces.emplace(sface);
	}

	if (gDebug)
	{
		Helper::visualizeFaces(mTess, faces, "cubeTet.off");
	}

	upperhull.insert(faces.begin(), faces.end());
	lowerhull.insert(faces.begin(), faces.end());

	if (gDebug)
	{
		Helper::visualizeFaces(mTess, upperhull, "cubeTet.off");
		Helper::visualizeFaces(mTess, lowerhull, "cubeTet.off");
	}

	Helper::deleteTetras(mTess, outTetras);

	outTetras.clear();
	triangulateCavity(upperhull, outTetras);
	outTetras.clear();
	triangulateCavity(lowerhull, outTetras);
	//if (outTetras.size() != 3)
	//	return;



	outTetras.clear();
	outFaces.insert(faces.begin(), faces.end());
}

//void M3d::Mesher3d::findIntersectingTetras(const VertexArr& faceVtx, Tetraset& outTetras)
//{
//	for (int i = 0; i < faceVtx.size(); i++)
//	{
//		Edge* edge = mTess.findEdge(faceVtx[i], faceVtx[(i + 1) % 3]);
//		if (!edge)
//			continue;
//
//		for (auto it = edge->faces.begin(); it != edge->faces.end(); it++)
//		{
//			Face* face = *it;
//			Vertex* e1 = face->opposite(edge);
//
//			for (int j = 0; j < 2; j++)
//			{
//				Tetra* tetra = face->tetra[j];
//				if (outTetras.find(tetra) != outTetras.end())
//					continue;
//
//				Vertex* e2 = tetra->opposite(face);
//
//				double o3de1 = GeomTest::orient3d(faceVtx[0]->coord, faceVtx[1]->coord, faceVtx[2]->coord, e1->coord);
//				double o3de2 = GeomTest::orient3d(faceVtx[0]->coord, faceVtx[1]->coord, faceVtx[2]->coord, e2->coord);
//				if (signbit(o3de1*o3de2))
//				{
//					GeomTest::IntersectionInfo intersectionInfo;
//					GeomTest::lineSegXTriangle(faceVtx[0]->coord, faceVtx[1]->coord, faceVtx[2]->coord,
//						e1->coord, e2->coord, intersectionInfo);
//					if (intersectionInfo.intersectionPos == GeomTest::kInside)
//					{
//						outTetras.emplace(tetra);
//						Edge* insideEdge = mTess.findEdge(e1, e2);
//						for (auto eit = insideEdge->faces.begin(); eit != insideEdge->faces.end(); eit++)
//						{
//							outTetras.emplace((*eit)->tetra[0]);
//							outTetras.emplace((*eit)->tetra[1]);
//						}
//					}
//				}
//			}
//		}
//	}
//}

void M3d::Mesher3d::findIntersectingTetras(const VertexArr& faceVtx, Tetraset& outTetras)
{
	std::vector<Vec3d> points;
	for (size_t i = 0; i < faceVtx.size(); i++)
		points.push_back(faceVtx[i]->coord);
	PointInPolygon pip(points);

	Edgeset polygonEdges;
	findPolygonEdges(faceVtx, polygonEdges);

	for (auto eit = polygonEdges.begin(); eit != polygonEdges.end(); eit++)
	{
		Edge* edge = *eit;

		Tetraset surroundingTetras;
		edge->getSurroundingTetras(surroundingTetras);

		Tetra* seedTetra = nullptr;
		for (auto it = surroundingTetras.begin(); it != surroundingTetras.end(); it++)
		{
			if (polygonxTetra(polygonEdges, pip, *it))
			{
				seedTetra = *it;
				break;
			}
		}

		if (!seedTetra && gDebug)
		{
			Helper::visualizeTetras(mTess, surroundingTetras, "cubetet.off");
		}

		Tetraset tetraQ;
		if (seedTetra)
			tetraQ.emplace(seedTetra);
		while (!tetraQ.empty())
		{
			Tetra* tetra = *tetraQ.begin();
			outTetras.emplace(tetra);
			tetraQ.erase(tetraQ.begin());

			Tetraset neighborTetras;
			tetra->getSurroundingTetras(neighborTetras);

			for (auto tit = neighborTetras.begin(); tit != neighborTetras.end(); tit++)
			{
				if (outTetras.find(*tit) == outTetras.end() && polygonxTetra(polygonEdges, pip, *tit))
					tetraQ.emplace(*tit);
			}
		}
	}
}

bool M3d::Mesher3d::polygonxTetra(const Edgeset& polygonEdges, const PointInPolygon& pip, const Tetra* tetra)
{
	Edge* fedge = *polygonEdges.begin();
	Vec3d v1 = fedge->vertex[0]->coord;
	Vec3d v2 = fedge->vertex[1]->coord;
	Vec3d center;

	for (auto it = polygonEdges.begin(); it != polygonEdges.end(); it++)
	{
		if (*it == fedge)
			continue;

		center = (*it)->pointAt(0.5, 0.5);
		double area = GeomTest::unSignedArea3d(v1, v2, center);
		if (!GCore::isZero(area))
			break;
	}

	VertexList upperVertices, lowerVertices;
	for (int i = 0; i < 4; i++)
	{
		double o3dv = GeomTest::orient3d(v1, v2, center, tetra->vertex[i]->coord);
		if (GCore::isZero(o3dv))
			continue;
		else if (o3dv > 0)
			upperVertices.push_back(tetra->vertex[i]);
		else
			lowerVertices.push_back(tetra->vertex[i]);
	}

	if (upperVertices.empty() || lowerVertices.empty())
		return false;

	Vec3d e1 = upperVertices.front()->coord;
	Vec3d e2 = lowerVertices.front()->coord;

	double t = 0;
	GeomTest::Pos pos = GeomTest::lineXPlane(v1, v2, center, e1, e2, t);

	if (GCore::isZero(t))
		return false;

	Vec3d projectedPt;
	if (pos == GeomTest::kNonParallel)
		projectedPt = e1 + e1.direction(e2)*t;

	if (pip.inside(projectedPt))
		return true;

	//for (int i = 0; i < 4; i++)
	//{
	//	Edge* edge = mTess.findEdge(tetra->vertex[i], tetra->vertex[(i + 1) % 4]);
	//	if (edge && polygonEdges.find(edge) != polygonEdges.end())
	//		continue;

	//	double o3de1 = GeomTest::orient3d(v1, v2, center, edge->vertex[0]->coord);
	//	double o3de2 = GeomTest::orient3d(v1, v2, center, edge->vertex[1]->coord);

	//	if (GCore::isZero(o3de1) || GCore::isZero(o3de2))
	//		continue;

	//	std::cout << o3de1 << std::endl << o3de2 << std::endl << std::endl;

	//	if (signbit(o3de1*o3de2))
	//	{
	//		double t = 0;
	//		GeomTest::Pos pos = GeomTest::lineXPlane(v1, v2, center, edge->vertex[0]->coord, edge->vertex[1]->coord, t);

	//		Vec3d projectedPt;
	//		if (pos == GeomTest::kNonParallel)
	//			projectedPt = edge->vertex[0]->coord + edge->vertex[0]->coord.direction(edge->vertex[1]->coord)*t;

	//		if (pip.inside(projectedPt))
	//			return true;
	//	}
	//}

	return false;
}

void M3d::Mesher3d::removExteriorTetras(const Faceset& boundaryFaces)
{
	// clear the tetras out of the bbox
	//cleanTetrasOutOfDomainBox();

	//Faceset boundaryFaces;
	//FaceVertexArr missingFaces;
	//getBoundaryFaces(boundaryFaces);
	//findBoundaryFaces(boundaryFaces, missingFaces);
	//if (missingFaces.empty())
	{
		//visualize(boundaryFaces, "cubetet.off");

		double dgnlLen = pslg.box.diagonal().magnitude() / 10;
		Vec3d extPt = (pslg.box.lower - Vec3d(dgnlLen, dgnlLen, dgnlLen));
		Tetra* extTetra = mTess.locateTetra(extPt);

		Tetraset exteriorTetras;
		detectExteriorTetras(extTetra, boundaryFaces, exteriorTetras);

		Helper::deleteTetras(mTess, exteriorTetras);
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

	Helper::deleteTetras(mTess, tetrasToDel);
}





