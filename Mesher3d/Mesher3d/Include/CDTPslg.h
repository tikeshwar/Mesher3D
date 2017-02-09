#pragma once

#include <unordered_map>
#include "Tessellation.h"
#include "Pslg.h"

namespace M3d
{
	class EdgeGrid;
	class CDTPslg
	{
		typedef std::unordered_map<Vertex*, double> LFSMap;

	public:
		CDTPslg(const Pslg& pslg)
			:cdtPslg(pslg){}

		void createCDT();

		void writeToOff(const char* filename);

	private:
		void initGrid();
		void insertPslgInGrid();
		void calculateLFS();
		double getLocalFeatureSize(Vertex* vertex);

		void createProtectionBalls();
		void divideWeakEdges();
		bool isEdgeProtected(Edge* edge);

		void updatePslg();

		Edge* addEdge(const GCore::Vec3d& v1, const GCore::Vec3d& v2);
		void deleteEdge(Edge* edge);


	private:
		EdgeGrid* mEdgeGrid;
		Tessellation mTess;
		LFSMap mLfsMap;
		Edgeset mProtectedEdges;

	public:
		Pslg cdtPslg;
	};
}
