#include "BBox.h"
#include "Edge.h"
#include "EdgeGrid.h"

using namespace GCore;
using namespace M3d;

void EdgeGrid::emplace(Edge* edge)
{
	Box3d box;
	box.extend(edge->vertex[0]->coord);
	box.extend(edge->vertex[1]->coord);
	EdgeGrid3d::emplace(edge, box.lower, box.upper);
}

void EdgeGrid::erase(Edge* edge)
{
	Box3d box;
	box.extend(edge->vertex[0]->coord);
	box.extend(edge->vertex[1]->coord);
	EdgeGrid3d::erase(edge, box.lower, box.upper);
}

Edge* EdgeGrid::findClosest(const Vec3d& point, const Edgeset& edgesNotToConsider)const
{
	Vec3i ijk = getIJK(point);
	size_t idx = getIndex(ijk.x, ijk.y, ijk.z);
	//const Node3d& node = nodeAt(idx);

	Edgeset candidateEdges;

	Vec3i minIJK = ijk, maxIJK = ijk;
	while (candidateEdges.empty())
	{
		for (size_t i = minIJK.x; i <= maxIJK.x; i++)
			for (size_t j = minIJK.y; j <= maxIJK.y; j++)
				for (size_t k = minIJK.z; k <= maxIJK.z; k++)
				{
					if (i == minIJK.x || i == maxIJK.x
						|| j == minIJK.y || j == maxIJK.y
						|| k == minIJK.z || k == maxIJK.z)
					{
						idx = getIndex(i, j, k);
						const EdgeGrid3d::Node3d& node = nodeAt(idx);

						for (auto it = node.data.begin(); it != node.data.end(); it++)
							if (edgesNotToConsider.find(*it) == edgesNotToConsider.end())
								candidateEdges.emplace(*it);
					}
				}

		const GCore::Vec3i& div = mDiv;

		minIJK.x = (minIJK.x < 1) ? 0 : minIJK.x - 1;
		minIJK.y = (minIJK.y < 1) ? 0 : minIJK.y - 1;
		minIJK.z = (minIJK.z < 1) ? 0 : minIJK.z - 1;

		maxIJK.x = (maxIJK.x + 1 > div.x - 1) ? div.x - 1 : maxIJK.x + 1;
		maxIJK.y = (maxIJK.y + 1 > div.y - 1) ? div.y - 1 : maxIJK.y + 1;
		maxIJK.z = (maxIJK.z + 1 > div.z - 1) ? div.z - 1 : maxIJK.z + 1;

		if (minIJK.x - 1 < 0 && minIJK.y - 1 < 0 && minIJK.z - 1 < 0
			&& maxIJK.x + 1 < div.x - 1 && maxIJK.y + 1 < div.y - 1 && maxIJK.z + 1 < div.z - 1)
			break;
	}

	double minD = HUGE;
	Edge* edge = nullptr;
	for (auto it = candidateEdges.begin(); it != candidateEdges.end(); it++)
	{
		double d = point.shortestDistanceToLineSeg((*it)->vertex[0]->coord, (*it)->vertex[1]->coord);
		if (d < minD)
		{
			minD = d;
			edge = *it;
		}
	}
	return edge;
}

void EdgeGrid::getEdges(size_t li, size_t lj, size_t lk,
	size_t ui, size_t uj, size_t uk, Edgeset & outEdges)const
{
	for (size_t i = li; i <= ui; i++)
		for (size_t j = lj; j <= uj; j++)
			for (size_t k = lk; k <= uk; k++)
			{
				size_t idx = getIndex(i, j, k);
				const auto& data = nodeAt(idx).data;

				for (auto it = data.begin(); it != data.end(); it++)
					outEdges.emplace(*it);
			}
}