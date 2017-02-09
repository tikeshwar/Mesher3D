#pragma once

#include "Defines.h"
#include "Vec.h"
#include "Mat4x4.h"

namespace GCore
{
	//https://people.eecs.berkeley.edu/~jrs/meshpapers/robnotes.pdf

	class GeomTest
	{
	public:

		enum Pos
		{
			kInside,
			kOnBoundary,
			kOutside,
			kParallel,
			kNonParallel,
			kIntersecting,
			kOnVertex,
			kOnEdge,
			kOnFace,
			kOnSameside,
			kSkew
		};

		/* a positive value if the points a, b, and c are arranged in counterclockwise order,
		   a negative value if the points are in clockwise order, and
		   zero if the points are collinear*/
		static double orient2d(const Vec2d& a, const Vec2d& b, const Vec2d& c)
		{
			return  (a.x - c.x)*(b.y - c.y) - (a.y - c.y)*(b.x - c.x);
		}

		/*  You can apply a right-hand rule: orient your right hand with fingers
			curled to follow the circular sequence bcd
			zero if points are coplanar*/
		static double orient3d(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d)
		{
			return (a - d).dot((b - d).cross(c - d));
		}

		/* If orient2d is positive, then incircle is positive if d lies inside the circle
			negative of d lies outside the circle
			zero if on the circle, otherwise orient2d is negative, the signs are reversed*/
		static double inCircle(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d)
		{
			Vec3d t(a.x - d.x, a.y - d.y, (a.x - d.x)*(a.x - d.x) + (a.y - d.y)*(a.y - d.y));
			Vec3d u(b.x - d.x, b.y - d.y, (b.x - d.x)*(b.x - d.x) + (b.y - d.y)*(b.y - d.y));
			Vec3d v(c.x - d.x, c.y - d.y, (c.x - d.x)*(c.x - d.x) + (c.y - d.y)*(c.y - d.y));
			return t.dot(u.cross(v));
		}

		/* If orient3d is positive, then incircle is positive if d lies inside the circle
			negative of d lies outside the circle
			zero if on the circle, otherwise orient3d is negative, the signs are reversed*/
		static double inSphere(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d, const Vec3d& e)
		{
			Vec3d t = a - e, u = b - e, v = c - e, w = d - e;
			double tmag = t.dot(t), umag = u.dot(u), vmag = v.dot(v), wmag = w.dot(w);

			return t.x*Vec3d(u.y, u.z, umag).dot(Vec3d(v.y, v.z, vmag).cross(Vec3d(w.y, w.z, wmag)))
				- t.y*Vec3d(u.x, u.z, umag).dot(Vec3d(v.x, v.z, vmag).cross(Vec3d(w.x, w.z, wmag)))
				+ t.z*Vec3d(u.x, u.y, umag).dot(Vec3d(v.x, v.y, vmag).cross(Vec3d(w.x, w.y, wmag)))
				- tmag*u.dot(v.cross(w));
		}

		static double signedArea2d(const Vec2d& a, const Vec2d& b, const Vec2d& c)
		{
			return orient2d(a, b, c) / 2.0;
		}

		static double unSignedArea3d(const Vec3d& a, const Vec3d& b, const Vec3d& c)
		{
			return (a - c).cross(b - c).magnitude() / 2.0;
		}

		static double signedVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d)
		{
			return orient3d(a, b, c, d) / 6.0;
		}

		static void findCircumcircle2d(const Vec2d& a, const Vec2d& b, const Vec2d& c, Vec2d& outCenter, double& outRadius)
		{
			Vec2d r = a - c, s = b - c;
			double area4 = signedArea2d(a, b, c)*2.0;
			outCenter.x = c.x + (r.dot(r)*s.y - s.dot(s)*r.y) / area4;
			outCenter.y = c.y + (r.x*s.dot(s) - s.x*r.dot(r)) / area4;
			outRadius = sqrt((b - c).magnitudeSqr()*(c - a).magnitudeSqr()*(a - b).magnitudeSqr()) / area4;
		}

		static void findCircumcircle3d(const Vec3d& a, const Vec3d& b, const Vec3d& c, Vec3d& outCenter, double& outRadius)
		{
			Vec3d r = a - c, s = b - c;
			double area4 = unSignedArea3d(a, b, c);
			double areaSqr8 = 8.0*area4*area4;
			Vec3d radialExp = ((s*r.dot(r) - r*s.dot(s)).cross(r.cross(s))) / areaSqr8;
			outCenter = c + radialExp;
			outRadius = radialExp.magnitude();
		}

		static void findCircumsphere(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d, Vec3d& outCenter, double& outRadius)
		{
			Vec3d t = a - d, u = b - d, v = c - d;
			double vol = signedVolume(a, b, c, d);
			double vol12 = 12.0*vol;
			Vec3d radialExp = (u.cross(v)*t.dot(t) + v.cross(t)*u.dot(u) + t.cross(u)*v.dot(v)) / vol12;
			outCenter = d + radialExp;
			outRadius = radialExp.magnitude();
		}

		static Pos lineXPlane(const Vec3d& a, const Vec3d& b, const Vec3d& c,
			const Vec3d& p, const Vec3d& q, double& t)
		{
			Vec3d norm = (a - c).cross(b - c);
			norm.normalize();

			Vec3d dir = p.direction(q);
			if (GCore::isZero<double>(dir.dot(norm)))
				return kParallel;

			t = (a - p).dot(norm) / dir.dot(norm);
			return kNonParallel;
		}

		struct IntersectionInfo
		{
			bool isIntersecting;
			Vec3d intersectionPt;
			Pos intersectionPos;
		};

		static void lineSegXTriangle(const Vec3d& a, const Vec3d& b, const Vec3d& c,
			const Vec3d& p, const Vec3d& q, IntersectionInfo& outIntersectionInfo)
		{
			outIntersectionInfo.isIntersecting = false;
			outIntersectionInfo.intersectionPos = kOutside;

			double t = 0;
			if (lineXPlane(a, b, c, p, q, t) == kNonParallel)
			{
				Vec3d projectedPt = p + p.direction(q)*t;
				outIntersectionInfo.intersectionPt = projectedPt;

				double A = unSignedArea3d(a, b, c);
				double A1 = unSignedArea3d(projectedPt, a, b);
				double A2 = unSignedArea3d(projectedPt, b, c);
				double A3 = unSignedArea3d(projectedPt, c, a);

				if (isEqual<double>(A, A1 + A2 + A3))
				{
					outIntersectionInfo.isIntersecting = true;
					outIntersectionInfo.intersectionPt = projectedPt;

					if (isEqual(A, A1) || isEqual(A, A2) || isEqual(A, A3))
						outIntersectionInfo.intersectionPos = kOnVertex;
					else if (isEqual(A, A1 + A2) || isEqual(A, A2 + A3) || isEqual(A, A3 + A1))
						outIntersectionInfo.intersectionPos = kOnEdge;
					else
						outIntersectionInfo.intersectionPos = kInside;
				}
			}
			else
				outIntersectionInfo.intersectionPos = kParallel;
		}

		static Pos linexline(const Vec3d& p1, const Vec3d& p2, const Vec3d& q1, const Vec3d& q2,
			double& t1, double& t2)
		{
			double vol = orient3d(p1, p2, q1, q2);
			if (!isZero(vol))
				return Pos::kSkew;

			// as they are planar, lets transform to the xy
			GCore::Mat4x4d xyMat;

			GCore::Vec3d norm = (p2 - p1).cross(q1 - p1);
			norm.normalize();
			GCore::Vec3d center = (p1 + p2) / 2.0;
			xyMat = xyMat* GCore::Mat4x4d::rotateBetweenAxes(norm, GCore::Vec3d(0, 0, 1))*GCore::Mat4x4d::translate(-center.x, -center.y, -center.z);

			Vec3d p12d = xyMat.transform(p1);
			Vec3d p22d = xyMat.transform(p2);
			Vec3d q12d = xyMat.transform(q1);
			Vec3d q22d = xyMat.transform(q2);

			Vec3d dir1 = p12d.direction(p22d);
			Vec3d dir2 = q12d.direction(q22d);

			Vec3d P = q12d - p12d;
			double D = -dir1.x*dir2.y + dir1.y*dir2.x;
			if (isZero(D))
				return Pos::kParallel;

			double Dt1 = -P.x*dir2.y + P.y*dir2.x;
			double Dt2 = P.y*dir1.x - P.x*dir1.y;

			t1 = Dt1 / D;
			t2 = Dt2 / D;

			return Pos::kIntersecting;
		}
	};
}
