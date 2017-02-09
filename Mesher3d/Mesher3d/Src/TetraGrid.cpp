#include "Tetra.h"
#include "TetraGrid.h"

using namespace GCore;
using namespace M3d;

void TetraGrid::emplace(Tetra* tetra)
{
	TetGrid3d::emplace(tetra, tetra->box.lower, tetra->box.upper);
}

void TetraGrid::erase(Tetra* tetra)
{
	TetGrid3d::erase(tetra, tetra->box.lower, tetra->box.upper);
}

Tetra* TetraGrid::find(const Vec3d& point)const
{
	Vec3i ijk = getIJK(point);
	size_t idx = getIndex(ijk.x, ijk.y, ijk.z);
	
	const auto& data = nodeAt(idx).data;
	for (auto it = data.begin(); it != data.end(); it++)
	{
		if ((*it)->contains(point))
			return (*it);
	}
	return nullptr;
}

void TetraGrid::getTetras(size_t li, size_t lj, size_t lk,
	size_t ui, size_t uj, size_t uk,
	Tetraset & outTetras)const
{
	for (size_t i = li; i <= ui; i++)
		for (size_t j = lj; j <= uj; j++)
			for (size_t k = lk; k <= uk; k++)
			{
				size_t idx = getIndex(i, j, k);
				const auto& data = nodeAt(idx).data;

				for (auto it = data.begin(); it != data.end(); it++)
					outTetras.emplace(*it);
			}
}
