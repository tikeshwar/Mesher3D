#include <fstream>
#include <unordered_map>
#include "GeomTest.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Tetra.h"
#include "Helper.h"

using namespace M3d;
using namespace GCore;

bool M3d::Helper::locallyDelaunay(Face* face)
{
	if (face->tetra[0] == nullptr || face->tetra[1] == nullptr)
		return true;

	GeomTest::Pos pos = face->tetra[0]->isInsideCircumsphere(face->tetra[1]->opposite(face)->coord);
	if (pos == GeomTest::kOutside || pos == GeomTest::kOnBoundary)
		return true;
	return false;
}

void M3d::Helper::reflexEdges(Face* face, Edgeset& outReflexEdges)
{
	double angle[2] = { 0,0 };

	for (int i = 0; i < 3; i++)
	{
		Edge* edge = face->edge[i];

		for (int j = 0; j < 2; j++)
		{
			angle[j] = 0;

			Vec3d tetCenter = face->tetra[j]->centroid();

			Face* reflexFace = face->tetra[j]->opposite(face->opposite(edge));

			double o3d = GeomTest::orient3d(reflexFace->vertex[0]->coord, reflexFace->vertex[1]->coord,
				reflexFace->vertex[2]->coord, tetCenter);
			if (GCore::isZero(o3d))
			{
				angle[j] = 0;
			}
			else
			{
				double dot = reflexFace->refNormal(tetCenter).dot(face->refNormal(tetCenter));
				angle[j] = acos(dot);
				if (!std::isnormal(angle[j]))
					angle[j] = 0;
			}
		}

		if (GCore::isSmallerOrEqual(angle[0] + angle[1], 3.142))
			outReflexEdges.emplace(edge);
	}
}

bool M3d::Helper::checkIfZeroVolumeTetra(const Faceset& faceset, const GCore::Vec3d& point)
{
	for (auto it = faceset.begin(); it != faceset.end(); it++)
	{
		if ((*it)->vertex[0]->coord == point ||
			(*it)->vertex[1]->coord == point ||
			(*it)->vertex[2]->coord == point)
			continue;

		double vol = GeomTest::orient3d((*it)->vertex[0]->coord, (*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord, point);

		if (GCore::isZero(vol))
			return true;
	}
	return false;
}

void M3d::Helper::deleteTetras(Tessellation& tess, const Tetraset& tetraToDelete)
{
	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
		tess.deleteTetra(*it);
}


bool M3d::Helper::checkIfDelaunay(const Tetraset& tetraset)
{
	Faceset faces;
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
		for (int i = 0; i < 4; i++)
			faces.emplace((*it)->face[i]);

	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		Face* face = *it;
		if (face->tetra[0] == nullptr || face->tetra[1] == nullptr)
			continue;

		Vertex* oppVtx = face->tetra[1]->opposite(face);
		bool b1 = face->tetra[0]->isInsideCircumsphere(oppVtx->coord) != GeomTest::kOutside;
		oppVtx = face->tetra[0]->opposite(face);
		bool b2 = face->tetra[1]->isInsideCircumsphere(oppVtx->coord) != GeomTest::kOutside;
		if (b1^b2)
		{
			Tetraset tset;
			tset.emplace(face->tetra[0]);
			tset.emplace(face->tetra[1]);
			//visualize(tset, "cubeTet.off");
			//visualize(tetraset, "cubeTet.off");

			return false;
		}
	}
	return true;
}

int M3d::Helper::checkZeroVolume(const Tessellation& tess)
{
	int zeroVolTetras = 0;
	auto tetras = tess.getTetras();
	for (auto it = tetras.begin(); it != tetras.end(); it++)
	{
		double vol = GeomTest::orient3d((*it)->vertex[0]->coord, (*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord, (*it)->vertex[3]->coord);

		if (GCore::isZero(vol))
			zeroVolTetras++;
	}

	return zeroVolTetras;
}

bool M3d::Helper::checkFaceSanity(const Tessellation& tess)
{
	int bdryFaces = 0;
	auto faces = tess.getFaces();
	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		int tetraCount = 0;
		for (int i = 0; i < 2; i++)
		{
			if ((*it)->tetra[i])
				tetraCount++;
		}

		if (tetraCount != 2)
			bdryFaces++;
	}

	return bdryFaces == 4;
}

void M3d::Helper::visualize(const Tessellation& tess, const char* filename)
{
	tess.writeToOff(filename);
	system("start meshlab.exe cubeTet.off");

}

//void visualize(const Vertex* v1, const Vertex* v2, const char* filename)
//{
//	Tetraset tetras;
//
//	VertexList midVerticesV1, midVerticesV2;
//	EdgeList edgesV1, edgesV2;
//	findInBetweenVertices(v1, v2, midVerticesV1, edgesV1);
//	findInBetweenVertices(v2, v1, midVerticesV2, edgesV2);
//
//	std::list<const Vertex*> vertexArr;
//	for (auto it = midVerticesV1.begin(); it != midVerticesV1.end(); it++)
//		vertexArr.push_back(*it);
//	for (auto it = midVerticesV2.begin(); it != midVerticesV2.end(); it++)
//		vertexArr.push_back(*it);
//
//	vertexArr.push_back(v1);
//	vertexArr.push_back(v2);
//
//	for (auto vit = vertexArr.begin(); vit != vertexArr.end(); vit++)
//	{
//		//for (auto it = edgev[i]->edges.begin(); it != edgev[i]->edges.end(); it++)
//		{
//			Tetraset outTetras;
//			auto it = (*vit)->edges.begin();
//			(*it)->getSurroundingTetras(outTetras);
//			tetras.emplace(*outTetras.begin());
//
//			//break;
//		}
//	}
//	visualize(tetras, filename);
//}

void M3d::Helper::visualizeFaces(const Tessellation& tess, const Faceset& faceset, const char* filename)
{
	std::ofstream file;
	file.open(filename);

	const auto& vset = tess.getVertices();

	file << "OFF" << std::endl;
	file << vset.size() << " " << faceset.size() << " 0" << std::endl;

	std::unordered_map<Vertex*, int> v2iMap;

	int v = 0;
	for (auto const vertex : vset)
	{
		file << vertex->coord.x << " " << vertex->coord.y << " " << vertex->coord.z << std::endl;
		v2iMap.emplace(vertex, v++);
	}

	for (auto const face : faceset)
	{
		file << "3 "
			<< v2iMap[face->vertex[0]] << " "
			<< v2iMap[face->vertex[1]] << " "
			<< v2iMap[face->vertex[2]]
			<< std::endl;
	}

	file.close();
	system("start meshlab.exe cubeTet.off cubeTet.off");
}

void M3d::Helper::visualizeTetras(const Tessellation& tess, const Tetraset& tetraset, const char* filename)
{
	Faceset faceset;
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
	{
		for (int f = 0; f < 4; f++)
		{
			faceset.emplace((*it)->face[f]);
		}
	}

	M3d::Helper::visualizeFaces(tess, faceset, filename);
}

void M3d::Helper::getEdgeCoordList(const Edgeset& edgeset, EdgeCoordList& outEdgeCoordList)
{
	for (auto it = edgeset.begin(); it != edgeset.end(); it++)
	{
		outEdgeCoordList.push_back(EdgeTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord)
			);
	}
}

void M3d::Helper::getFaceCoordList(const Faceset& faceset, FaceCoordList& outFaceCoordList)
{
	for (auto it = faceset.begin(); it != faceset.end(); it++)
	{
		outFaceCoordList.push_back(FaceTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord)
			);
	}
}

void M3d::Helper::getTetraCoordList(const Tetraset& tetraset, TetraCoordList& outTetraCoordList)
{
	for (auto it = tetraset.begin(); it != tetraset.end(); it++)
	{
		outTetraCoordList.push_back(TetraTuple(
			(*it)->vertex[0]->coord,
			(*it)->vertex[1]->coord,
			(*it)->vertex[2]->coord,
			(*it)->vertex[3]->coord)
			);
	}
}


bool M3d::Helper::findCircumscribingTetras(const Tessellation& tess, const Vec3d& point, Tetraset& outTetraset)
{
	Tetra* seedTetra = tess.locateTetra(point);
	if (!seedTetra)
		return false;

	Tetraset tetraQueue;
	tetraQueue.emplace(seedTetra);

	Faceset hullfaces, nonhullfaces;

	while (!tetraQueue.empty())
	{
		auto it = tetraQueue.begin();
		Tetra* tetra = *it;
		tetraQueue.erase(it);

		if (tetra->isInsideCircumsphere(point) != GeomTest::kOutside)
		{
			outTetraset.emplace(tetra);
			for (int i = 0; i < 4; i++)
			{
				Tetra* otherTetra = tetra->face[i]->otherTetra(tetra);
				if (otherTetra && outTetraset.find(otherTetra) == outTetraset.end())
					tetraQueue.emplace(otherTetra);
			}
		}
	}

	return true;
}

void M3d::Helper::correctCavity(const GCore::Vec3d& point, Tetraset& outTetraset)
{
	Faceset hullfaces, facesToDelete;
	findHullFaces(outTetraset, hullfaces, facesToDelete);

	bool validCavity = true;
	for (auto it = hullfaces.begin(); it != hullfaces.end(); it++)
	{
		Face* face = *it;
		Tetra* nonHullTetra = nullptr;
		if (outTetraset.find(face->tetra[0]) != outTetraset.end())
			nonHullTetra = face->tetra[0];
		else
			nonHullTetra = face->tetra[1];

		Vec3d tetCenter = nonHullTetra->centroid();
		double o3dtet = GeomTest::orient3d(face->vertex[0]->coord, face->vertex[1]->coord,
			face->vertex[2]->coord, tetCenter);
		double o3dpoint = GeomTest::orient3d(face->vertex[0]->coord, face->vertex[1]->coord,
			face->vertex[2]->coord, point);

		if (GCore::isZero(o3dpoint*o3dtet) || (signbit(o3dtet) ^ signbit(o3dpoint)))
		{
			validCavity = false;

			Tetra* otherTetra = face->otherTetra(nonHullTetra);
			if (otherTetra)
				outTetraset.emplace(otherTetra);
			break;
		}
	}

	if (!validCavity)
		correctCavity(point, outTetraset);

}

void M3d::Helper::findDeletedPoints(const Vertexset& verticesToCheck, const Faceset& nonhullfaces, std::list<GCore::Vec3d>& outPoints)
{
	for (auto vit = verticesToCheck.begin(); vit != verticesToCheck.end(); vit++)
	{
		bool isBaldVertex = true;

		Faceset faces;
		for (auto eit = (*vit)->edges.begin(); eit != (*vit)->edges.end(); eit++)
		{
			faces.insert((*eit)->faces.begin(), (*eit)->faces.end());
		}

		for (auto fit = faces.begin(); fit != faces.end(); fit++)
		{
			if (nonhullfaces.find(*fit) == nonhullfaces.end())
			{
				isBaldVertex = false;
				break;
			}
		}

		if (isBaldVertex)
			outPoints.push_back((*vit)->coord);
	}
}

void M3d::Helper::findHullFaces(const Tetraset& tetraToDelete, Faceset& outFaceset, Faceset& outFacesToDelete)
{
	for (auto it = tetraToDelete.begin(); it != tetraToDelete.end(); it++)
	{
		Tetra* tetra = *it;
		for (int i = 0; i < 4; i++)
		{
			Face* face = tetra->face[i];

			size_t count = 0;
			for (int j = 0; j < 2; j++)
			{
				if (face->tetra[j] && tetraToDelete.find(face->tetra[j]) != tetraToDelete.end())
					count++;
			}

			if (count == 1)
				outFaceset.emplace(face);
			else
				outFacesToDelete.emplace(face);
		}
	}
}

void M3d::Helper::findAllCoords(const Tetraset& tetraToDelete, Coordset& outCoords)
{
	Vertexset vertexset;
	for (auto tit = tetraToDelete.begin(); tit != tetraToDelete.end(); tit++)
		for (int i = 0; i < 4; i++)
			vertexset.emplace((*tit)->vertex[i]);

	for (auto vit = vertexset.begin(); vit != vertexset.end(); vit++)
		outCoords.push_back((*vit)->coord);
}

void M3d::Helper::triangulateCavity(Tessellation& tess, const Tetraset& tetraToDelete, const GCore::Vec3d& point, Tetraset& outTetras)
{
	Faceset hullfaces, facesToDelete;
	findHullFaces(tetraToDelete, hullfaces, facesToDelete);

	if (gDebug)
		Helper::visualizeFaces(tess, hullfaces, "cubeTet.off");

	FaceCoordList faceCoordList;
	Helper::getFaceCoordList(hullfaces, faceCoordList);

	//if (checkIfZeroVolumeTetra(hullfaces, point))
	//{
	//	std::cout << "was trying to create zero vol tetra" << std::endl;
	//	return;
	//}

	Helper::deleteTetras(tess, tetraToDelete);

	if (gDebug)
		Helper::visualizeTetras(tess, tess.getTetras(), "cubeTet.off");

	Tetraset newTetras;
	fillvoid(tess, faceCoordList, point, outTetras);

	//bool sane = checkFaceSanity();
	//if (!sane)
	//{
	//	std::cout << gAngle0 << " " << gAngle1 << std::endl;
	//	//visualize(hullfaces, "cubeTet.off");
	//	//visualize(outTetras, "cubeTet.off");
	//}

	if (gDebug)
	{
		Helper::visualizeFaces(tess, hullfaces, "cubeTet.off");
		Helper::visualizeTetras(tess, outTetras, "cubeTet.off");
	}
}

void M3d::Helper::fillvoid(Tessellation& tess, const FaceCoordList& faceCoords, const GCore::Vec3d& point, Tetraset& newTetras)
{
	for (auto it = faceCoords.begin(); it != faceCoords.end(); it++)
	{
		const GCore::Vec3d& v1 = std::get<0>(*it);
		const GCore::Vec3d& v2 = std::get<1>(*it);
		const GCore::Vec3d& v3 = std::get<2>(*it);

		if (v1 == point || v2 == point || v3 == point)
			continue;
		//double vol = GeomTest::signedVolume(v1, v2, v3, point);
		//if (GCore::isZero<double>(fabs(vol)))
		//{
		//	std::cout << gi << std::endl;
		//}

		Tetra* tetra = tess.addTetra(v1, v2, v3, point);
		if (tetra)
			newTetras.emplace(tetra);

	}
}


