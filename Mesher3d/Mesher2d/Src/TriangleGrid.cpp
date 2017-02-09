#include "Triangle.h"
#include "TriangleGrid.h"

using namespace GCore;
using namespace M2d;

void TriangleGrid::emplace(Triangle* triangle)
{
	GCore::Box2d trlgBox = triangle->boundingBox();
	mGrid2d.emplace(triangle, trlgBox.lower, trlgBox.upper);
}

void TriangleGrid::erase(Triangle* triangle)
{
	GCore::Box2d trlgBox = triangle->boundingBox();
	mGrid2d.erase(triangle, trlgBox.lower, trlgBox.upper);
}

Triangle* TriangleGrid::find(const Vec2d& point)const
{
	Vec2i ij = mGrid2d.getIJ(point);
	size_t idx = mGrid2d.getIndex(ij.x, ij.y);

	const auto& data = mGrid2d.nodeAt(idx).data;
	for (auto it = data.begin(); it != data.end(); it++)
	{
		if ((*it)->isInside(point))
			return (*it);
	}
	return nullptr;
}

void TriangleGrid::getTriangles(size_t li, size_t lj,
	size_t ui, size_t uj, Triangleset & outTriangles)const
{
	for (size_t i = li; i <= ui; i++)
		for (size_t j = lj; j <= uj; j++)
			{
				size_t idx = mGrid2d.getIndex(i, j);
				const auto& data = mGrid2d.nodeAt(idx).data;

				for (auto it = data.begin(); it != data.end(); it++)
					outTriangles.emplace(*it);
			}
}

