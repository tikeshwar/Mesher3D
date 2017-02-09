#pragma once

#include <vector>
#include "Defines.h"
#include "Vec.h"
#include "BBox.h"

namespace M3d
{
	struct NodeBox
	{
		GCore::Vec3d lower;
		GCore::Vec3d len;
	};

	template <typename T>
	class Node
	{
	public:
		Node() = default;
		Node(const NodeBox& _nodeBox)
			:nodeBox(_nodeBox) {}

		bool add(const T& t)
		{
			return data.emplace(t).second;
		}

		bool remove(const T& t)
		{
			return data.erase(t) > 0;
		}

	public:
		NodeBox nodeBox;
		std::unordered_set<T> data;
	};

	template <typename T>
	class Grid3D
	{
	public:
		typedef Node<T> Node3d;
		typedef std::vector<Node3d> NodeList;

	public:
		Grid3D(const GCore::Vec3d& start, const GCore::Vec3i& div, const GCore::Vec3d& delta)
			:mStart(start), mDiv(div), mDelta(delta)
		{
			mNodes.resize(mDiv.x * mDiv.y * mDiv.z);

			NodeBox nodebox;

			for (size_t i = 0; i < mDiv.x; i++)
				for (size_t j = 0; j < mDiv.y; j++)
					for (size_t k = 0; k < mDiv.z; k++)
					{
						nodebox.lower.x = mStart.x + i*mDelta.x;
						nodebox.lower.y = mStart.y + j*mDelta.y;
						nodebox.lower.z = mStart.z + k*mDelta.z;

						nodebox.len = mDelta;

						size_t idx = getIndex(i, j, k);
						mNodes[idx] = Node3d(nodebox);
					}
		}

		void emplace(T type, const GCore::Vec3d& lower, const GCore::Vec3d& upper)
		{
			GCore::Vec3i l = getIJK(lower);
			GCore::Vec3i u = getIJK(upper);

			for (size_t i = l.x; i <= u.x; i++)
				for (size_t j = l.y; j <= u.y; j++)
					for (size_t k = l.z; k <= u.z; k++)
					{
						size_t idx = getIndex(i, j, k);
						mNodes[idx].add(type);
					}

		}

		void erase(T type, const GCore::Vec3d& lower, const GCore::Vec3d& upper)
		{
			GCore::Vec3i l = getIJK(lower);
			GCore::Vec3i u = getIJK(upper);

			for (size_t i = l.x; i <= u.x; i++)
				for (size_t j = l.y; j <= u.y; j++)
					for (size_t k = l.z; k <= u.z; k++)
					{
						size_t idx = getIndex(i, j, k);
						mNodes[idx].remove(type);
					}
		}

		size_t getIndex(size_t i, size_t j, size_t k)const
		{
			return i*mDiv.y * mDiv.z + j*mDiv.z + k;
		}

		GCore::Vec3i getIJK(const GCore::Vec3d& point)const
		{
			auto vecLower = point - mStart;

			size_t x = (size_t)fmin(mDiv.x - 1, fmax(0, std::floor(vecLower.x / mDelta.x)));
			size_t y = (size_t)fmin(mDiv.y - 1, fmax(0, std::floor(vecLower.y / mDelta.y)));
			size_t z = (size_t)fmin(mDiv.z - 1, fmax(0, std::floor(vecLower.z / mDelta.z)));

			return GCore::Vec3i(x, y, z);
		}

		const Node3d& nodeAt(size_t idx)const
		{
			return mNodes[idx];
		}

		Node3d& nodeAt(size_t idx)
		{
			return mNodes[idx];
		}

	public:
		GCore::Vec3d mStart;
		GCore::Vec3i mDiv;
		GCore::Vec3d mDelta;

		size_t mSize;
	private:
		NodeList mNodes;
	};
}
