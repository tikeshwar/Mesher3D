#include <Vertex.h>
#include <Edge.h>

using namespace M2d;

Edge::Edge(Vertex* v1, Vertex* v2)
{
	vertex[0] = v1;
	vertex[1] = v2;

	triangle[0] = triangle[1] = ghostTriangle[0] = ghostTriangle[1] = nullptr;
}
Vertex*		Edge::otherVertex(const Vertex* inVertex)
{
	if (inVertex == vertex[0])
		return vertex[1];
	return vertex[0];
}
const Vertex*		Edge::otherVertex(const Vertex* inVertex)const
{
	if (inVertex == vertex[0])
		return vertex[1];
	return vertex[0];
}
Triangle*			Edge::otherTriangle(Triangle* inTriangle)
{
	if (triangle[0] == inTriangle)
		return triangle[1];
	else if (triangle[1] == inTriangle)
		return triangle[0];
	return nullptr;
}
const Triangle*		Edge::otherTriangle(const Triangle* inTriangle)const
{
	if (triangle[0] == inTriangle)
		return triangle[1];
	else if (triangle[1] == inTriangle)
		return triangle[0];
	return nullptr;
}
bool						Edge::addTriangle(Triangle* inTriangle)
{
	if (triangle[0] == nullptr)
	{
		triangle[0] = inTriangle;
		return true;
	}
	else if (triangle[1] == nullptr)
	{
		triangle[1] = inTriangle;
		return true;
	}
	return false;
}
bool						Edge::removeTriangle(Triangle* inTriangle)
{
	if (triangle[0] == inTriangle)
	{
		ghostTriangle[0] = inTriangle;
		triangle[0] = nullptr;
		return true;
	}
	else if (triangle[1] == inTriangle)
	{
		ghostTriangle[1] = inTriangle;
		triangle[1] = nullptr;
		return true;
	}
	return false;
}

bool					Edge::removeGhostTriangle(Triangle* inTriangle)
{
	if (ghostTriangle[0] == inTriangle)
	{
		ghostTriangle[0] = nullptr;
		return true;
	}
	else if (ghostTriangle[1] == inTriangle)
	{
		ghostTriangle[1] = nullptr;
		return true;
	}
	return false;
}

