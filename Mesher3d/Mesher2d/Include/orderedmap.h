#pragma once

#include <map>

template <typename K, typename V>
class OrderedMap:public std::multimap<K, V>
{
	typedef std::multimap<K, V> MultiMap;
	typedef std::pair<K, V>	KVPair;
	typedef typename MultiMap::iterator MSIterator;
public:
	OrderedMap(){}
	~OrderedMap(){MultiMap::~multimap();}

	bool insert(K k, V v)
	{
		auto mItr = multimap::equal_range(k);
		if(mItr.first != multimap::end() && mItr.first == mItr.second)
		for(auto it = mItr.first; it != mItr.second; it++)
		{
			if(it->second == v)
				return false;
		}
		multimap::insert(KVPair(k,v));
		return true;
	}
	bool erase(K k, V v)
	{
		auto mItr = multimap::equal_range(k);
		for(auto it = mItr.first; it != mItr.second; it++)
		{
			if(it->second == v)
			{
				multimap::erase(it);
				return true;
			}
		}
		return false;
	}
	bool find(K k, V v)
	{
		auto mItr = multimap::equal_range(k);
		for(auto it = mItr.first; it != mItr.second; it++)
		{
			if(it->second == v)
				return true;
		}
		return false;
	}
};