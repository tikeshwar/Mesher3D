#pragma once

#include <fstream>
#include "GeomTest.h"
#include "Edge.h"
#include "EdgeGrid.h"
#include "CDTPslg.h"

using namespace M3d;
using namespace GCore;

void CDTPslg::createCDT()
{
	initGrid();
	insertPslgInGrid();
	calculateLFS();
	createProtectionBalls();
	divideWeakEdges();
	updatePslg();
}

void CDTPslg::initGrid()
{
	cdtPslg.calculateBBox();
	Box3d pslgBox = cdtPslg.box;
	Vec3d oriDiagonal = pslgBox.diagonal();

	double fx = 1.2;
	double fy = 1.2;
	double fz = 1.2;
	pslgBox.scale(fx, fy, fz);
	Vec3d diagonal = pslgBox.diagonal();

	Vec3i divs(10, 10, 10);
	Vec3d delta(diagonal.x / divs.x, diagonal.y / divs.y, diagonal.z / divs.z);
	mEdgeGrid = new EdgeGrid(pslgBox.lower, divs, delta);
}

void CDTPslg::insertPslgInGrid()
{
	for (auto it = cdtPslg.boundaryFaces.begin(); it != cdtPslg.boundaryFaces.end(); it++)
	{
		for (size_t i = 0; i < it->size(); i++)
			addEdge(cdtPslg.points[(*it)[i]], cdtPslg.points[(*it)[(i + 1) % it->size()]]);
	}

}

void CDTPslg::calculateLFS()
{
	auto& vertices = mTess.getVertices();
	for (auto it = vertices.begin(); it != vertices.end(); it++)
	{
		double lfs = getLocalFeatureSize(*it);
		mLfsMap.emplace(*it, lfs);
	}
}

double CDTPslg::getLocalFeatureSize(Vertex* vertex)
{
	double minR = HUGE;
	Edge* edge = mEdgeGrid->findClosest(vertex->coord, vertex->edges);
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

void CDTPslg::createProtectionBalls()
{
	double threshold = 90 * PI / 180.0;

	auto vertices = mTess.getVertices();
	for (auto vit = vertices.begin(); vit != vertices.end(); vit++)
	{
		Vertex* vertex = *vit;
		Edgeset acuteEdges;
		if (!vertex->isAcute(threshold, acuteEdges))
			continue;

		double shortestLen = vertex->shortestEdge()->length();
		double lfs = mLfsMap[vertex];

		double protectionRadi = std::fmin(lfs, shortestLen / 3.0);

		for (auto eit = acuteEdges.begin(); eit != acuteEdges.end(); eit++)
		{
			if (isEdgeProtected(*eit))
				continue;

			Vertex* otherVtx = (*eit)->otherVertex(vertex);
			Vec3d otherCoord = vertex->coord + vertex->coord.direction(otherVtx->coord)*protectionRadi;
			mProtectedEdges.emplace(addEdge(vertex->coord, otherCoord));
			addEdge(otherCoord, otherVtx->coord);
			deleteEdge(*eit);
		}
	}
}


bool CDTPslg::isEdgeProtected(Edge* edge)
{
	Vec3d center = (edge->vertex[0]->coord + edge->vertex[1]->coord) / 2.0;
	double rad = edge->length() / 2.0;

	Sphere sphere(rad, center);
	Box3d sphereBox = sphere.boundingBox();

	Vec3i lijk = mEdgeGrid->getIJK(sphereBox.lower);
	Vec3i uijk = mEdgeGrid->getIJK(sphereBox.upper);

	Edgeset outEdges;
	mEdgeGrid->getEdges(lijk.x, lijk.y, lijk.z, uijk.x, uijk.y, uijk.z, outEdges);

	outEdges.insert(edge->vertex[0]->edges.begin(), edge->vertex[0]->edges.end());
	outEdges.insert(edge->vertex[1]->edges.begin(), edge->vertex[1]->edges.end());
	outEdges.erase(edge);

	double closestd = HUGE;
	Vec3d firstClosestPt, secondClosestPt;
	bool found = false;
	for (auto it = outEdges.begin(); it != outEdges.end(); it++)
	{
		for (int i = 0; i < 2; i++)
		{
			Vertex* vertex = (*it)->vertex[i];
			if (vertex == edge->vertex[0] || vertex == edge->vertex[1])
				continue;

			if (sphere.inside(vertex->coord))
			{
				found = true;
				double dist = vertex->coord.perpendicularDistanceToLineSeg(edge->vertex[0]->coord, edge->vertex[1]->coord);
				if (dist < closestd)
				{
					closestd = dist;
					secondClosestPt = firstClosestPt;
					firstClosestPt = vertex->coord;
				}
			}
		}
	}

	if (found)
	{
		Vec3d tcenter;
		double tradius;
		GeomTest::findCircumcircle3d(edge->vertex[0]->coord, edge->vertex[1]->coord, firstClosestPt, tcenter, tradius);
		if (GCore::isSmaller((secondClosestPt - center).magnitude(), tradius))
			return false;
	}

	return true;
}

void CDTPslg::divideWeakEdges()
{
	Edgeset edges = mTess.getEdges(), weakEdges;
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		if (mProtectedEdges.find(*eit) == mProtectedEdges.end() && !isEdgeProtected(*eit))
			weakEdges.emplace(*eit);
	}

	while (!weakEdges.empty())
	{
		Edge* edge = *weakEdges.begin();
		weakEdges.erase(weakEdges.begin());
		if (edge->vertex[0] && edge->vertex[1])
		{
			if (!isEdgeProtected(edge))
			{
				Vec3d midPt = (edge->vertex[0]->coord + edge->vertex[1]->coord) / 2.0;
				weakEdges.emplace(addEdge(edge->vertex[0]->coord, midPt));
				weakEdges.emplace(addEdge(midPt, edge->vertex[1]->coord));
				deleteEdge(edge);
			}
		}
	}
}

void CDTPslg::updatePslg()
{
	if (mTess.getVertices().size() == cdtPslg.points.size())
		return;

	Vertexset vset(mTess.getVertices().begin(), mTess.getVertices().end());
	for (auto it = cdtPslg.points.begin(); it != cdtPslg.points.end(); it++)
	{
		Vertex* vertex = mTess.findVertex(*it);
		if (vertex)
			vset.erase(vertex);
	}

	for (auto it = vset.begin(); it != vset.end(); it++)
	{
		cdtPslg.points.push_back((*it)->coord);
	}
}

Edge* CDTPslg::addEdge(const Vec3d& v1, const Vec3d& v2)
{
	Edge* edge = mTess.addEdge(v1, v2);
	if (edge)
		mEdgeGrid->emplace(edge);
	return edge;
}

void CDTPslg::deleteEdge(Edge* edge)
{
	mEdgeGrid->erase(edge);
	mTess.deleteEdge(edge);
}

void CDTPslg::writeToOff(const char* filname)
{
	std::ofstream file;
	file.open(filname);

	file << "OFF" << std::endl;
	file << cdtPslg.points.size() << " 0 0" << std::endl;

	for (auto const point : cdtPslg.points)
	{
		file << point.x << " " << point.y << " " << point.z << std::endl;
	}
}

