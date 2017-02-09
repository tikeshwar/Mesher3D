#include <iostream>
#include <fstream>
#include <sstream>
#include "Parser2d.h"

using namespace M2d;

void PolyParser::parse(const char * filename)
{
	std::ifstream file(filename);
	if (!file.is_open())
		return;

	parseNodes(file);
	parseFacets(file);
	parseHoles(file);
	parseRegions(file);

	pslg.calculateBBox();
}

void PolyParser::parseNodes(std::ifstream & file)
{
	constexpr int BUFSIZE = 256;
	char line[BUFSIZE];

	// read the node count, dim, attr, bdrymarker
	int nodeCount = 0, dim = 0, attr = 0, bdrymarker = 0;
	while (file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> nodeCount >> dim >> attr >> bdrymarker;
		break;
	}

	// read all node coords with nodeIndex
	pslg.points.resize(nodeCount);

	int nodeIdx = 0;
	GCore::Vec2d coord;
	while (nodeIdx <= nodeCount && file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> nodeIdx >> coord.x >> coord.y;

		pslg.points[nodeIdx - 1] = coord;
		nodeIdx++;
	}
}

void PolyParser::parseFacets(std::ifstream & file)
{
	constexpr int BUFSIZE = 256;
	char line[BUFSIZE];

	// read the facetCount
	int facetCount = 0, bdrymarker = 0;
	while (file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> facetCount >> bdrymarker;
		break;
	}

	// read the facets
	//pslg.boundarySegments.resize(facetCount);
	int vertIdx = 0, segCount;
	GCore::Vec2i coord;
	while (vertIdx < facetCount && file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> segCount >> coord.x >> coord.y;
		pslg.boundarySegments.push_back(GCore::Vec2i(coord.x - 1, coord.y - 1));

		vertIdx++;
	}
}

void PolyParser::parseHoles(std::ifstream & file)
{
	constexpr int BUFSIZE = 256;
	char line[BUFSIZE];

	// read the facetCount
	int holeCount = 0;
	while (file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> holeCount;
		break;
	}

	if (holeCount > 0)
	{
		// read all node coords with nodeIndex
		pslg.holePoints.resize(holeCount);

		int nodeIdx = 0;
		GCore::Vec2d coord;
		while (nodeIdx <= holeCount && file.getline(line, BUFSIZE))
		{
			if (line[0] == '#' || !strlen(line))
				continue;

			std::stringstream ss(line);
			ss >> nodeIdx >> coord.x >> coord.y;

			pslg.holePoints[nodeIdx - 1] = coord;
			nodeIdx++;
		}
	}
}

void PolyParser::parseRegions(std::ifstream & /*file*/)
{
}





