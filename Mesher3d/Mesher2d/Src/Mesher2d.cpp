#include "Vertex.h"
#include "Edge.h"
#include "Triangle.h"
#include "TriangleGrid.h"
#include "Mesher2d.h"

using namespace M2d;
using namespace GCore;

#define orientation(p,q,t) (p.x - t.x)*(q.y - t.y) - (p.y - t.y)*(q.x - t.x)

Mesher2d::~Mesher2d()
{
	if (mTessGrid)
		delete mTessGrid;
}

void Mesher2d::createDelaunay2D()
{
	createSuperSquare();
	inserVertices();
	recoverEdges();
	removeExternalTriangles();
}

void Mesher2d::setUpGrid(const Box2d& box)
{
	Vec2i divs(10, 10);
	Vec2d boxdgnl = box.diagonal();
	Vec2d delta(boxdgnl.x / divs.x, boxdgnl.y / divs.y);
	mTessGrid = new TriangleGrid(box.lower, divs, delta);
}

Triangle* Mesher2d::addTriangle(Vertex* v1, Vertex* v2, Vertex* v3)
{
	Triangle* triangle = mTess.addTriangle(v1, v2, v3);
	if (triangle)
		mTessGrid->emplace(triangle);
	return triangle;
}

void Mesher2d::removeTriangle(Triangle* triangle)
{
	bool removed = mTess.removeTriangle(triangle);
	if (removed)
		mTessGrid->erase(triangle);
}

void Mesher2d::createSuperSquare()
{
	Box2d scaledBox = mPslg.box;
	scaledBox.scale(1.2, 1.2);
	setUpGrid(scaledBox);

	Vec2d bl(scaledBox.lower.x, scaledBox.lower.y);
	Vec2d br(scaledBox.upper.x, scaledBox.lower.y);
	Vec2d tr(scaledBox.upper.x, scaledBox.upper.y);
	Vec2d tl(scaledBox.lower.x, scaledBox.upper.y);

	Vertex* vbl = mTess.addVertex(bl);
	Vertex* vbr = mTess.addVertex(br);
	Vertex* vtr = mTess.addVertex(tr);
	Vertex* vtl = mTess.addVertex(tl);

	addTriangle(vbl, vbr, vtr);
	addTriangle(vbl, vtr, vtl);
}

void Mesher2d::inserVertices()
{
	for (auto it = mPslg.points.begin(); it != mPslg.points.end(); it++)
		insert(*it);
}

void Mesher2d::insert(const Vec2d& point)
{
	Vertex* vertex = mTess.findVertex(point);
	if (!vertex)
	{
		Triangleset circumscribingTrgls;
		findCircumscribingTriangles(point, circumscribingTrgls);

		Edgeset wrapperEdges;
		findWrapperEdges(circumscribingTrgls, wrapperEdges);

		removeTriangles(circumscribingTrgls);

		Triangleset newTriangles;
		triangulateCavity(point, wrapperEdges, newTriangles);

		deleteTriangles(circumscribingTrgls);
	}
}

Triangle* Mesher2d::locatePoint(const Vec2d& point)
{
	return mTessGrid->find(point);
}

void Mesher2d::findCircumscribingTriangles(const Vec2d& point, Triangleset& outTriangles)
{
	Triangle* containingTrgl = locatePoint(point);
	if (containingTrgl)
	{
		Triangleset trglQueue;
		trglQueue.emplace(containingTrgl);

		while (!trglQueue.empty())
		{
			Triangle* trgl = *trglQueue.begin();
			outTriangles.emplace(trgl);
			trglQueue.erase(trglQueue.begin());

			for (int i = 0; i < 3; i++)
			{
				Triangle* otherTrgl = trgl->edge[i]->otherTriangle(trgl);
				if (otherTrgl && outTriangles.find(otherTrgl) == outTriangles.end())
				{
					if (otherTrgl->circle->isInside(point))
						trglQueue.emplace(otherTrgl);
				}
			}
		}
	}
}

void Mesher2d::findWrapperEdges(const Triangleset& triangles, Edgeset& outEdgeset)
{
	for (auto it = triangles.begin(); it != triangles.end(); it++)
	{
		for (size_t i = 0; i < 3; i++)
		{
			Edge* edge = (*it)->edge[i];

			size_t count = 0;
			for (size_t j = 0; j < 2; j++)
			{
				if (triangles.find(edge->triangle[j]) != triangles.end())
					count++;
			}
			if (count < 2)
				outEdgeset.emplace(edge);
		}
	}
}

void Mesher2d::removeTriangles(const Triangleset& triangles)
{
	for (auto it = triangles.begin(); it != triangles.end(); it++)
		removeTriangle(*it);
}

void Mesher2d::triangulateCavity(const GCore::Vec2d& point, const Edgeset& wrapperEdges, Triangleset& outNewTriangles)
{
	Vertex* newVertex = mTess.addVertex(point);
	if (newVertex)
	{
		for (auto it = wrapperEdges.begin(); it != wrapperEdges.end(); it++)
		{
			Triangle* triangle = addTriangle((*it)->vertex[0], (*it)->vertex[1], newVertex);
			if (triangle)
				outNewTriangles.emplace(triangle);
		}
	}
}

void Mesher2d::deleteTriangles(const Triangleset& triangles)
{
	for (auto it = triangles.begin(); it != triangles.end(); it++)
		mTess.deleteTriangle(*it);
}

void Mesher2d::recoverEdges()
{
	auto recover = [this](size_t i, size_t j)
	{
		Vertex* v1 = mTess.findVertex(mPslg.points[i]);
		Vertex* v2 = mTess.findVertex(mPslg.points[j]);
		if (v1 && v2)
		{
			Edge* edge = mTess.findEdge(v1, v2);
			if (!edge)
				recoverEdge(v1, v2);
		}
	};

	for (auto it = mPslg.boundarySegments.begin(); it != mPslg.boundarySegments.end(); it++)
		recover((*it).x, (*it).y);

	for (auto it = mPslg.constraintSegments.begin(); it != mPslg.constraintSegments.end(); it++)
		recover((*it).x, (*it).y);

}

void Mesher2d::recoverEdge(Vertex* v1, Vertex* v2)
{
	Edge* edge = nullptr;
	do
	{
		recoverEdgeWithFlip(v1, v2);
		edge = mTess.findEdge(v1, v2);

	} while (!edge);
}

bool Mesher2d::recoverEdgeWithFlip(Vertex* v1, Vertex* v2)
{
	Edge* dividerEdge = nullptr;
	bool goodForFlip = false;
	for (auto it = v1->edges.begin(); it != v1->edges.end(); it++)
	{
		if (goodForFlip)
			break;

		for (int i = 0; i < 2; i++)
		{
			Triangle* trgl = (*it)->triangle[i];
			if (trgl)
			{
				Edge* oppEdge = trgl->opposite(v1);
				double p1 = orientation(v1->coord, v2->coord, oppEdge->vertex[0]->coord);
				double p2 = orientation(v1->coord, v2->coord, oppEdge->vertex[1]->coord);
				double q1 = orientation(oppEdge->vertex[0]->coord, oppEdge->vertex[1]->coord, v1->coord);
				double q2 = orientation(oppEdge->vertex[0]->coord, oppEdge->vertex[1]->coord, v2->coord);

				if (GCore::isSmaller<double>(p1*p2, 0) && GCore::isSmaller<double>(q1*q2, 0))
				{
					dividerEdge = oppEdge;
					goodForFlip = true;
					break;
				}
			}
		}
	}

	if (dividerEdge)
	{
		Triangle* t0 = dividerEdge->triangle[0];
		Triangle* t1 = dividerEdge->triangle[1];

		Vertex* dv1 = dividerEdge->vertex[0];
		Vertex* dv2 = dividerEdge->vertex[1];

		Vertex* v00 = t0->opposite(dividerEdge);
		Vertex* v01 = t1->opposite(dividerEdge);

		Triangle* dividerTrgls[2] = { dividerEdge->triangle[0] , dividerEdge->triangle[1] };
		for (int i = 0; i < 2; i++)
		{
			if (dividerTrgls[i])
				removeTriangle(dividerTrgls[i]);
		}

		addTriangle(dv1, v00, v01);
		addTriangle(dv2, v00, v01);

		for (int i = 0; i < 2; i++)
		{
			if (dividerTrgls[i])
				mTess.deleteTriangle(dividerTrgls[i]);
		}
		return true;
	}
	return false;
}

void Mesher2d::removeExternalTriangles()
{
	Edgeset boundaryEdges;
	if (getBoundaryEdges(boundaryEdges))
	{
		Triangleset exteriorTrglset;
		getExteriorTriangle(boundaryEdges, exteriorTrglset);

		deleteTriangles(exteriorTrglset);
	}
}

bool Mesher2d::getBoundaryEdges(Edgeset& outBoundaryEdges)
{
	for (auto it = mPslg.boundarySegments.begin(); it != mPslg.boundarySegments.end(); it++)
	{
		Vertex* v1 = mTess.findVertex(mPslg.points[(*it).x]);
		Vertex* v2 = mTess.findVertex(mPslg.points[(*it).y]);
		Edge* edge = mTess.findEdge(v1, v2);
		if (edge)
			outBoundaryEdges.emplace(edge);
	}
	return mPslg.boundarySegments.size() == outBoundaryEdges.size();
}

void Mesher2d::getExteriorTriangle(const Edgeset& boundaryEdges, Triangleset& outExteriorTriangles)
{
	Box2d tessbox = mTess.boundingBox();
	Box2d pslgbox = mPslg.box;

	Vec2d midPt = (tessbox.lower + pslgbox.lower) / 2.0;

	mPslg.holePoints.push_back(midPt);

	for (auto it = mPslg.holePoints.begin(); it != mPslg.holePoints.end(); it++)
	{
		Triangle* seedExtTrgl = mTessGrid->find(*it);
		if (!seedExtTrgl)
			return;

		Triangleset trglQueue;
		trglQueue.emplace(seedExtTrgl);

		while (!trglQueue.empty())
		{
			Triangle* trgl = *trglQueue.begin();
			outExteriorTriangles.emplace(trgl);
			trglQueue.erase(trglQueue.begin());

			for (int i = 0; i < 3; i++)
			{
				Edge* edge = trgl->edge[i];
				if (boundaryEdges.find(edge) == boundaryEdges.end())
				{
					Triangle* otherTrgl = edge->otherTriangle(trgl);
					if (otherTrgl && outExteriorTriangles.find(otherTrgl) == outExteriorTriangles.end())
						trglQueue.emplace(otherTrgl);
				}
			}
		}
	}
}


