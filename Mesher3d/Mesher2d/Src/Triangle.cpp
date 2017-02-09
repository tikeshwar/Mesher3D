#include <map>
#include "Tolerance.h"
#include "Edge.h"
#include "Triangle.h"

using namespace M2d;

void Triangle::findCircumCircle(const GCore::Vec2d & p1,
	const GCore::Vec2d & p2,
	const GCore::Vec2d & p3,
	GCore::Vec2d & outCenter,
	double & outRadius)
{
	GCore::Vec2d a = p1 - p3;
	GCore::Vec2d b = p2 - p3;
	GCore::Vec2d l = b*a.magnitudeSqr() - a*b.magnitudeSqr();
	double axbMagSqr = a.magnitudeSqr()*b.magnitudeSqr() - (a.dot(b))*(a.dot(b));
	outCenter = p3 + ((a*(l.dot(b)) - b*(l.dot(a))) / (2 * axbMagSqr));
	outRadius = (outCenter - p3).magnitude();
}
Triangle::Triangle(Vertex* v1, Vertex* v2, Vertex* v3, Edge* e1, Edge* e2, Edge* e3)
{
	vertex[0] = v1;
	vertex[1] = v2;
	vertex[2] = v3;

	edge[0] = e1;
	edge[1] = e2;
	edge[2] = e3;

	GCore::Vec2d center;
	double radius = 0;
	findCircumCircle(vertex[0]->coord, vertex[1]->coord, vertex[2]->coord, center, radius);
	circle = new GCore::Circle(center, radius);
}
Triangle::~Triangle()
{
	if (circle)
		delete circle;
}

inline bool			Triangle::isVertex(const Vertex* inVertex)const
{
	return (vertex[0] == inVertex || vertex[1] == inVertex || vertex[2] == inVertex);
}
inline bool			Triangle::isEdge(const Edge* inEdge)const
{
	return (edge[0] == inEdge || edge[1] == inEdge || edge[2] == inEdge);
}
void				Triangle::neighbours(Triangle* neighbors[3])
{
	neighbors[0] = edge[0]->otherTriangle(this);
	neighbors[1] = edge[1]->otherTriangle(this);
	neighbors[2] = edge[2]->otherTriangle(this);
}
double				Triangle::angleAt(const Vertex* inVertex)const
{
	const Edge* tedge = opposite(inVertex);
	GCore::Vec2d v1 = inVertex->coord - tedge->vertex[0]->coord;
	GCore::Vec2d v2 = inVertex->coord - tedge->vertex[1]->coord;
	v1.normalize();
	v2.normalize();
	return acos(v1.dot(v2));
}
Edge*				Triangle::opposite(const Vertex* inVertex)
{
	if (vertex[0] == inVertex)return edge[0];
	if (vertex[1] == inVertex)return edge[1];
	if (vertex[2] == inVertex)return edge[2];
	return nullptr;
}
const Edge*			Triangle::opposite(const Vertex* inVertex)const
{
	if (vertex[0] == inVertex)return edge[0];
	if (vertex[1] == inVertex)return edge[1];
	if (vertex[2] == inVertex)return edge[2];
	return nullptr;
}
Vertex*				Triangle::opposite(const Edge* inEdge)
{
	if (edge[0] == inEdge)return vertex[0];
	if (edge[1] == inEdge)return vertex[1];
	if (edge[2] == inEdge)return vertex[2];
	return nullptr;
}
const Vertex*		Triangle::opposite(const Edge* inEdge)const
{
	if (edge[0] == inEdge)return vertex[0];
	if (edge[1] == inEdge)return vertex[1];
	if (edge[2] == inEdge)return vertex[2];
	return nullptr;
}


inline bool				Triangle::inCircumcircle(const GCore::Vec2d & inPoint)const
{
	return circle->isInside(inPoint);
}
inline bool				Triangle::inCircumcircle(const GCore::Vec2d & inPoint)
{
	return circle->isInside(inPoint);
}

inline GCore::Vec2d		Triangle::centroid()const
{
	return (vertex[0]->coord + vertex[1]->coord + vertex[2]->coord) / 3.0;
}
inline double		Triangle::area()const
{
	return fabs(0.5*(vertex[0]->coord.x*vertex[1]->coord.y - vertex[0]->coord.y*vertex[1]->coord.x));
}

const Edge*			Triangle::smallestEdge()const
{
	typedef std::map<double, const Edge*> EdgeMap;
	EdgeMap edgeMap;
	edgeMap.insert(EdgeMap::value_type(edge[0]->lengthSqr(), edge[0]));
	edgeMap.insert(EdgeMap::value_type(edge[1]->lengthSqr(), edge[1]));
	edgeMap.insert(EdgeMap::value_type(edge[2]->lengthSqr(), edge[2]));

	return edgeMap.begin()->second;
}
const Edge*			Triangle::largestEdge()const
{
	typedef std::map<double, const Edge*> EdgeMap;
	EdgeMap edgeMap;
	edgeMap.insert(EdgeMap::value_type(edge[0]->lengthSqr(), edge[0]));
	edgeMap.insert(EdgeMap::value_type(edge[1]->lengthSqr(), edge[1]));
	edgeMap.insert(EdgeMap::value_type(edge[2]->lengthSqr(), edge[2]));

	return (--edgeMap.end())->second;
}
double				Triangle::smallestAngle()const
{
	double a0 = angleAt(vertex[0]);
	double a1 = angleAt(vertex[1]);
	double a2 = angleAt(vertex[2]);
	return std::fmin(a0, std::fmin(a1, a2));
}
double				Triangle::largestAngle()const
{
	double a0 = angleAt(vertex[0]);
	double a1 = angleAt(vertex[1]);
	double a2 = angleAt(vertex[2]);
	return std::fmax(a0, std::fmax(a1, a2));
}
GCore::Box2d				Triangle::boundingBox()const
{
	GCore::Box2d box;
	box.extend(vertex[0]->coord);
	box.extend(vertex[1]->coord);
	box.extend(vertex[2]->coord);
	return box;
}
bool	Triangle::isInside(const GCore::Vec2d & inPoint)const
{
	GCore::Vec2d A = vertex[0]->coord - vertex[2]->coord;
	GCore::Vec2d B = vertex[1]->coord - vertex[2]->coord;
	GCore::Vec2d C = inPoint - vertex[2]->coord;

	double D = A.x*B.y - A.y*B.x;
	double D1 = C.x*B.y - C.y*B.x;
	double D2 = A.x*C.y - A.y*C.x;

	double t1 = D1 / D, t2 = D2 / D, t3 = 1 - t1 - t2;
	if (GCore::isPositive<double>(t1) && GCore::isPositive<double>(t2) && GCore::isPositive<double>(t3)
		&& GCore::isSmallerOrEqual<double>(t1, 1) && GCore::isSmallerOrEqual<double>(t2, 1) && GCore::isSmallerOrEqual<double>(t3, 1))
		return true;
	return false;
}