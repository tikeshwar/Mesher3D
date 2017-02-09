#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "Defines.h"
#include "Mat4x4.h"
#include "Parser.h"
#include "Mesher2d.h"

using namespace M3d;

void Parser::triangulate(const Pslg& pslg, const Polygon& polygon, FaceList& outFaces)
{
	GCore::Mat4x4d xyMat;

	GCore::Vec3d center;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
		center = center + pslg.points[*it];
	center = center / (double)polygon.size();

	GCore::Vec3d norm = (pslg.points[polygon[2]] - pslg.points[polygon[1]]).cross(pslg.points[polygon[1]] - pslg.points[polygon[0]]);
	xyMat = xyMat* GCore::Mat4x4d::rotateBetweenAxes(norm, GCore::Vec3d(0, 0, 1))*GCore::Mat4x4d::translate(-center.x, -center.y, -center.z);

	M2d::Pslg pslg2d;
	for (auto it = polygon.begin(); it != polygon.end(); it++)
	{
		GCore::Vec3d transformedPt = xyMat.transform(pslg.points[*it]);
		pslg2d.points.push_back(GCore::Vec2d(transformedPt.x, transformedPt.y));
	}

	for (size_t i = 0; i < polygon.size(); i++)
		pslg2d.boundarySegments.push_back(GCore::Vec2i(i, (i + 1) % polygon.size()));

	pslg2d.calculateBBox();

	M2d::Mesher2d mesher2d(pslg2d);
	mesher2d.createDelaunay2D();
	mesher2d.getFacets(outFaces);
	mesher2d.writeToOff("cubeTet.off");
	system("start meshlab.exe cubeTet.off");
}

void PolyParser::parse(const char * filename)
{
	std::ifstream file(filename);
	if (!file.is_open())
		return;

	parseNodes(file);
	parseFacets(file);
	parseHoles(file);
	parseRegions(file);

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
	GCore::Vec3d coord;
	while (nodeIdx <= nodeCount && file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> nodeIdx >> coord.x >> coord.y >> coord.z;

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
	Polygon polygon;
	int vertIdx = 0, polygonCount = 0, vertCount = 0;
	while (file.getline(line, BUFSIZE) && vertIdx < facetCount)
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> polygonCount;

		ss = std::stringstream();
		file.getline(line, BUFSIZE);
		ss << line;
		ss >> vertCount;

		unsigned int val = 0;
		for (int i = 0; i < vertCount; i++)
		{
			ss >> val;
			polygon.push_back(val - 1);
		}
		vertIdx++;
		pslg.boundaryFaces.push_back(polygon);
		if (polygon.size() == 14)
		{
			//FaceList facets;
			//Parser::triangulate(pslg, polygon, facets);
			/*for (auto it = facets.begin(); it != facets.end(); it++)
				pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[(*it).x], polygon[(*it).y], polygon[(*it).z]));*/
		}
		/*else
		{
			pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[0], polygon[1], polygon[2]));
		}*/
		polygon.clear();

	}
}

void PolyParser::parseHoles(std::ifstream & file)
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

	if (nodeCount)
	{
		// read all node coords with nodeIndex
		pslg.points.resize(nodeCount);

		int nodeIdx = 0;
		GCore::Vec3d coord;
		while (nodeIdx <= nodeCount && file.getline(line, BUFSIZE))
		{
			if (line[0] == '#' || !strlen(line))
				continue;

			std::stringstream ss(line);
			ss >> nodeIdx >> coord.x >> coord.y >> coord.z;

			pslg.points[nodeIdx - 1] = coord;
			nodeIdx++;
		}
	}
}

void PolyParser::parseRegions(std::ifstream & /*file*/)
{
}



//-----------------------smesh----------------------------------

void SMeshParser::parse(const char * filename)
{
	mLargestIdx = 0;

	std::ifstream file(filename);
	if (!file.is_open())
		return;

	parseNodes(file);
	parseFacets(file);
	parseHoles(file);
	parseRegions(file);

}

void SMeshParser::parseNodes(std::ifstream & file)
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
	int tidx = 0;
	GCore::Vec3d coord;
	while (nodeIdx < nodeCount && file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> tidx >> coord.x >> coord.y >> coord.z;
		mLargestIdx = tidx;
		pslg.points[nodeIdx] = coord;
		nodeIdx++;
	}
}

void SMeshParser::parseFacets(std::ifstream & file)
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
	int vertIdx = 0, polygonCount = 0;
	while (file.getline(line, BUFSIZE) && vertIdx < facetCount)
	{
		Polygon polygon;
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> polygonCount;

		unsigned int val = 0;
		for (int i = 0; i < polygonCount; i++)
		{
			ss >> val;
			if (mLargestIdx == pslg.points.size())
				polygon.push_back(val - 1);
			else
				polygon.push_back(val);
		}
		vertIdx++;

	/*	FaceList facets;
		Parser::triangulate(pslg, polygon, facets);
		for (auto it = facets.begin(); it != facets.end(); it++)
			pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[(*it).x], polygon[(*it).y], polygon[(*it).z]));*/
		pslg.boundaryFaces.push_back(polygon);

	}
}

void SMeshParser::parseHoles(std::ifstream & /*file*/)
{
}

void SMeshParser::parseRegions(std::ifstream & /*file*/)
{
}

//--------------------OFF--------------------------------------

void OffParser::parse(const char* filename)
{
	std::ifstream file(filename);
	if (!file.is_open())
		return;

	parseNodes(file);
	parseFacets(file);
}

void OffParser::parseNodes(std::ifstream& file)
{
	constexpr int BUFSIZE = 256;
	char line[BUFSIZE];

	while (file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		if (std::string(line).find("OFF") != std::string::npos)
			break;
	}

	// read the node count, dim, attr, bdrymarker
	while (file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> mNodeCount >> mFacetCount;
		break;
	}

	// read all node coords with nodeIndex
	pslg.points.resize(mNodeCount);

	size_t nodeIdx = 0;
	GCore::Vec3d coord;
	while (nodeIdx < mNodeCount && file.getline(line, BUFSIZE))
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> coord.x >> coord.y >> coord.z;

		pslg.points[nodeIdx] = coord;
		nodeIdx++;
	}
}

void OffParser::parseFacets(std::ifstream& file)
{
	constexpr int BUFSIZE = 256;
	char line[BUFSIZE];

	// read the facets
	size_t vertIdx = 0, polygonCount;
	while (file.getline(line, BUFSIZE) && vertIdx < mFacetCount)
	{
		if (line[0] == '#' || !strlen(line))
			continue;

		std::stringstream ss(line);
		ss >> polygonCount;

		Polygon polygon;
		unsigned int val = 0;
		for (size_t i = 0; i < polygonCount; i++)
		{
			ss >> val;
			polygon.push_back(val);
		}
		vertIdx++;
		pslg.boundaryFaces.push_back(polygon);
		//if (polygon.size() == 3)
		//{
		//	pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[0], polygon[1], polygon[2]));
		//}
		//else if (polygon.size() == 4)
		//{
		//	pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[0], polygon[1], polygon[2]));
		//	pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[0], polygon[2], polygon[3]));
		//}
		//else if (polygon.size() > 4)
		//{
		//	FaceList facets;
		//	Parser::triangulate(pslg, polygon, facets);
		//	for (auto it = facets.begin(); it != facets.end(); it++)
		//		pslg.boundaryFaces.push_back(GCore::Vec3i(polygon[(*it).x], polygon[(*it).y], polygon[(*it).z]));
		//}
	}
}

