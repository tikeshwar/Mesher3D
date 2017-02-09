#pragma once

#include <unordered_set>
#include "BBox.h"

namespace GCore
{
	struct NodeBox
	{
		Vec2d lower;
		Vec2d len;
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
	class Grid2D
	{
	public:
		typedef Node<T> Node2d;
		typedef std::vector<Node2d> NodeList;

	public:
		Grid2D(const Vec2d& start, const Vec2i& div, const Vec2d& delta)
		{
			mStart = start;
			mDiv = div;
			mDelta = delta;

			mNodes.resize(mDiv.x * mDiv.y);

			NodeBox nodebox;
			for (size_t i = 0; i < mDiv.x; i++)
				for (size_t j = 0; j < mDiv.y; j++)
					{
						nodebox.lower.x = mStart.x + i*mDelta.x;
						nodebox.lower.y = mStart.y + j*mDelta.y;

						nodebox.len = mDelta;

						size_t idx = getIndex(i, j);
						mNodes[idx] = Node2d(nodebox);
					}
		}

		void emplace(T type, const Vec2d& lower, const Vec2d& upper)
		{
			Vec2i l = getIJ(lower);
			Vec2i u = getIJ(upper);

			for (size_t i = l.x; i <= u.x; i++)
				for (size_t j = l.y; j <= u.y; j++)
					{
						size_t idx = getIndex(i, j);
						mNodes[idx].add(type);
					}

		}

		void erase(T type, const Vec2d& lower, const Vec2d& upper)
		{
			Vec2i l = getIJ(lower);
			Vec2i u = getIJ(upper);

			for (size_t i = l.x; i <= u.x; i++)
				for (size_t j = l.y; j <= u.y; j++)
					{
						size_t idx = getIndex(i, j);
						mNodes[idx].remove(type);
					}
		}

		size_t getIndex(size_t i, size_t j)const
		{
			return i*mDiv.y + j;
		}

		Vec2i getIJ(const Vec2d& point)const
		{
			auto vecLower = point - mStart;

			size_t x = (size_t)fmin(mDiv.x - 1, fmax(0, std::floor(vecLower.x / mDelta.x)));
			size_t y = (size_t)fmin(mDiv.y - 1, fmax(0, std::floor(vecLower.y / mDelta.y)));

			return Vec2i(x, y);
		}

		const Node2d& nodeAt(size_t idx)const
		{
			return mNodes[idx];
		}

		Node2d& nodeAt(size_t idx)
		{
			return mNodes[idx];
		}

	public:
		Vec2d mStart;
		Vec2i mDiv;
		Vec2d mDelta;

		size_t mSize;
	private:
		NodeList mNodes;
	};
}
