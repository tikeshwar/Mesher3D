#pragma once

#include <unordered_set>
#include <queue>

namespace GCore
{
	template <typename T>
	class UniqueQueue
	{
	public:
		typedef std::unordered_set<T> Uniqueset;
		typedef std::queue<T> Queue;

	public:
		void push(const T& val)
		{
			if (mUniqueset.emplace(val).second)
				mQueue.push(val);
		}

		const T& pop()
		{
			T& val = mQueue.front();
			while (mUniqueset.erase(val) == 0)
			{
				if (!mQueue.empty())
				{
					val = mQueue.front();
					mQueue.pop();
				}
			}
				
			return val;
		}

		void erase(const T& val)
		{
			mUniqueset.erase(val);
		}

		bool empty()
		{
			return mUniqueset.empty();
		}

		size_t size()
		{
			return mUniqueset.size();
		}
	private:
		Uniqueset mUniqueset;
		Queue mQueue;
	};
}
