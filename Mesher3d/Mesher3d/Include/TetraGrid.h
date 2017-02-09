#pragma once

#include "Defines3d.h"
#include "Grid3D.h"

namespace M3d
{
	class Tetra;
	class TetraGrid :public Grid3D<Tetra*>
	{
	public:
		typedef Grid3D<Tetra*> TetGrid3d;

	public:
		TetraGrid(const GCore::Vec3d& start, const GCore::Vec3i& div, const GCore::Vec3d& delta)
			:TetGrid3d(start, div, delta) {}

		void emplace(Tetra* tetra);
		void erase(Tetra* tetra);

		Tetra* find(const GCore::Vec3d& point)const;

		void getTetras(size_t li, size_t lj, size_t lk,
			size_t ui, size_t uj, size_t uk, Tetraset & outTetras)const;


	};
}
