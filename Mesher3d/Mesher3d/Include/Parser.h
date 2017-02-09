#pragma once

#include "Pslg.h"

namespace M3d
{
	class Parser
	{
	public:
		typedef std::vector<size_t> Polygon;
		typedef std::vector<GCore::Vec3i> FaceList;

	public:
		virtual void parse(const char* filename) = 0;
		static void triangulate(const Pslg& pslg, const Polygon& polygon, FaceList& outFaces);

		Pslg pslg;
	};

	class PolyParser : public Parser
	{
	public:
		void parse(const char* filename);

	private:
		void parseNodes(std::ifstream& file);
		void parseFacets(std::ifstream& file);
		void parseHoles(std::ifstream& file);
		void parseRegions(std::ifstream& file);
	};

	class SMeshParser : public Parser
	{
	public:
		void parse(const char* filename);

	private:
		void parseNodes(std::ifstream& file);
		void parseFacets(std::ifstream& file);
		void parseHoles(std::ifstream& file);
		void parseRegions(std::ifstream& file);

	private:
		size_t mLargestIdx;
	};

	class OffParser :public Parser
	{
	public:
		void parse(const char* filename);

	private:
		void parseNodes(std::ifstream& file);
		void parseFacets(std::ifstream& file);

	private:
		size_t mNodeCount;
		size_t mFacetCount;
	};
}