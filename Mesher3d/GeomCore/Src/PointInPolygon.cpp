#include "GeomTest.h"
#include "PointInPolygon.h"
#include "BBox.h"

using namespace GCore;

void PointInPolygon::transformToXY()
{
	Box3d box;
	for (size_t i = 0; i < mPoints.size(); i++)
		box.extend(mPoints[i]);

	mXYMat.setIdentity();

	// get the normal
	Vec3d zAxis(0, 0, 1);
	Vec3d norm = (mPoints[1] - mPoints[0]).cross(mPoints[2] - mPoints[0]);
	if (GCore::isZero(norm.magnitudeSqr()))
	{
		size_t n = mPoints.size();
		for (size_t i = 1; i < n; i++)
		{
			norm = (mPoints[(i + 1) % n] - mPoints[i]).cross(
				mPoints[(i + 2) % n] - mPoints[i]);
			if (!GCore::isZero(norm.magnitudeSqr()))
				break;
		}
	}

	if (!GCore::isZero(norm.magnitudeSqr()))
	{
		norm.normalize();

		Vec3d rotAxis = norm.cross(zAxis);
		if (!isZero(rotAxis.magnitudeSqr()))
		{
			rotAxis.normalize();
			double rotAngle = acos(zAxis.dot(norm));
			mXYMat = Mat4x4d::rotate(rotAngle, rotAxis.x, rotAxis.y, rotAxis.z);
		}
	}

	Vec3d center = box.center();
	mXYMat = mXYMat*Mat4x4d::translate(-center.x, -center.y, -center.z);
	for (size_t i = 0; i < mPoints.size(); i++)
		mPoints[i] = mXYMat.transform(mPoints[i]);

	for (size_t i = 0; i < mPoints.size(); i++)
		mBox.extend(mPoints[i].xy());
}

int PointInPolygon::cnPnPoly(const Vec3d& point)const
{
	Vec3d P = mXYMat.transform(point);

	size_t n = mPoints.size();
	const std::vector<Vec3d>& V = mPoints;

	int    cn = 0;    // the  crossing number counter

					  // loop through all edges of the polygon
	for (size_t i = 0; i<n; i++) {    // edge from V[i]  to V[i+1]
		if (((V[i].y <= P.y) && (V[i + 1].y > P.y))     // an upward crossing
			|| ((V[i].y > P.y) && (V[i + 1].y <= P.y))) { // a downward crossing
														  // compute  the actual edge-ray intersect x-coordinate
			double vt = (P.y - V[i].y) / (V[i + 1].y - V[i].y);
			if (P.x < V[i].x + vt * (V[i + 1].x - V[i].x)) // P.x < intersect
				++cn;   // a valid crossing of y=P.y right of P.x
		}
	}
	return (cn & 1);    // 0 if even (out), and 1 if  odd (in)

}

int PointInPolygon::wnPnPoly(const Vec3d& point)const
{
	Vec3d P = mXYMat.transform(point);
	//if (!isZero(P.z))
	//	return 0;

	size_t n = mPoints.size();
	const std::vector<Vec3d>& V = mPoints;

	int    wn = 0;    // the  winding number counter

					  // loop through all edges of the polygon
	for (size_t i = 0; i < n; i++) {   // edge from V[i] to  V[i+1]
		if (V[i].y <= P.y) {          // start y <= P.y
			if (V[(i + 1) % n].y  > P.y)      // an upward crossing
				if (isLeft(V[i], V[(i + 1) % n], P) > 0)  // P left of  edge
					++wn;            // have  a valid up intersect
		}
		else {                        // start y > P.y (no test needed)
			if (V[(i + 1) % n].y <= P.y)     // a downward crossing
				if (isLeft(V[i], V[(i + 1) % n], P) < 0)  // P right of  edge
					--wn;            // have  a valid down intersect
		}
	}
	return wn;
}

int PointInPolygon::orientTest(const Vec3d& point)const
{
	Vec2d P = mXYMat.transform(point).xy();
	if (!mBox.contains(P))
		return 0;

	Vec2d farpt = mBox.lower - mBox.diagonal();
	Vec2d perturbedpt(0.000001, 0.0000001);

	size_t n = mPoints.size();

	std::vector<Vec2d> xyPoints(n);
	for (size_t i = 0; i < n; i++)
	{
		xyPoints[i] = mPoints[i].xy();
	}

	size_t intersectionCount = 0;
	for (size_t i = 0; i < n; i++)
	{
		Vec2d pi = xyPoints[i];
		Vec2d pj = xyPoints[(i + 1) % n];

		double o2dpi = GeomTest::orient2d(P, farpt, pi);
		if (GCore::isZero(o2dpi))
		{
			if (pi == P)
			{
				return 0;
			}
			else
			{
				pi = pi + perturbedpt;
				o2dpi = GeomTest::orient2d(P, farpt, pi);
			}

		}

		double o2dpj = GeomTest::orient2d(P, farpt, pj);
		if (GCore::isZero(o2dpj))
		{
			if (pj == P)
			{
				return 0;
			}
			else
			{
				pj = pj + perturbedpt;
				o2dpj = GeomTest::orient2d(P, farpt, pj);
			}

		}

		if (signbit(o2dpi*o2dpj))
		{
			double o2dP = GeomTest::orient2d(pi, pj, P);
			if (GCore::isZero(o2dP))
				//o2dP = GeomTest::orient2d(pi, pj, P + perturbedpt);
				return 0;

			double o2dfarpt = GeomTest::orient2d(pi, pj, farpt);
			if (GCore::isZero(o2dfarpt))
				//o2dfarpt = GeomTest::orient2d(pi, pj, farpt + perturbedpt);
				continue;

			if (signbit(o2dP*o2dfarpt))
				intersectionCount++;
		}
	}
	return intersectionCount % 2;
}