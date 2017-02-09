#include "GeomTest.h"
#include "Mat4x4.h"
#include "Sphere.h"

using namespace GCore;

Sphere::Sphere(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d)
{
	GeomTest::findCircumsphere(a, b, c, d, center, radius);

	//Mat4x4d m11, m12, m13, m14;
	//double a11[] = { a.x, a.y, a.z, 1, b.x, b.y, b.z, 1, c.x, c.y, c.z, 1, d.x, d.y, d.z, 1 };
	//double a12[] = { a.magnitudeSqr(), a.y, a.z, 1, b.magnitudeSqr(), b.y, b.z, 1, c.magnitudeSqr(), c.y, c.z, 1, d.magnitudeSqr(), d.y, d.z, 1 };
	//double a13[] = { a.magnitudeSqr(), a.x, a.z, 1, b.magnitudeSqr(), b.x, b.z, 1, c.magnitudeSqr(), c.x, c.z, 1, d.magnitudeSqr(), d.x, d.z, 1 };
	//double a14[] = { a.magnitudeSqr(), a.x, a.y, 1, b.magnitudeSqr(), b.x, b.y, 1, c.magnitudeSqr(), c.x, c.y, 1, d.magnitudeSqr(), d.x, d.y, 1 };

	//m11.setMatrix(a11);
	//m12.setMatrix(a12);
	//m13.setMatrix(a13);
	//m14.setMatrix(a14);

	//double d11 = m11.determinant();
	//double d12 = m12.determinant();
	//double d13 = m13.determinant();
	//double d14 = m14.determinant();

	//center.x = 0.5 * d12 / d11;
	//center.y = -0.5 * d13 / d11;
	//center.z = 0.5 * d14 / d11;

	//radius = (center - a).magnitude();
}

Box3d Sphere::boundingBox()const
{
	Box3d bbox;
	Vec3d radialVec(radius, radius, radius);
	bbox.extend(radialVec + center);
	bbox.extend(center - radialVec);
	return bbox;
}
