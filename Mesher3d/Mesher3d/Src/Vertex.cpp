#include "Defines2d.h"
#include "Defines.h"
#include "Vec.h"
#include "Edge.h"

#include "Vertex.h"

using namespace GCore;
using namespace M3d;

bool Vertex::isConnected()
{
	bool connected = false;
	for (auto it = edges.begin(); it != edges.end(); it++)
		if ((*it) != nullptr)
		{
			connected = true;
			break;
		}
	return connected;
}

// angle in radians
bool Vertex::isAcute(double angleRad, Edgeset& outEdges)
{
	double threshold = cos(angleRad);

	for (auto sit = edges.begin(); sit != edges.end(); sit++)
	{
		Vec3d sitVec = this->coord.direction((*sit)->otherVertex(this)->coord);
		//double sitVecMag = sitVec.magnitude();

		auto it = sit;
		it++;
		for (; it != edges.end(); it++)
		{
			Vec3d itVec = this->coord.direction((*it)->otherVertex(this)->coord);
			//double itVecMag = itVec.magnitude();
			double cosVal = sitVec.dot(itVec);
			if (GCore::isGreater(cosVal, threshold))
			{
				outEdges.emplace(*sit);
				outEdges.emplace(*it);
			}
		}
	}
	return !outEdges.empty();
}

Edge* Vertex::shortestEdge()
{
	Edge* shortestEdge = nullptr;
	double minEdgelen = HUGE;
	for (auto sit = edges.begin(); sit != edges.end(); sit++)
	{
		double edgelen = (*sit)->length();
		if (GCore::isSmaller(edgelen, minEdgelen))
		{
			minEdgelen = edgelen;
			shortestEdge = *sit;
		}
	}
	return shortestEdge;
}
