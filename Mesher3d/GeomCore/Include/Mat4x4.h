#pragma once

#include <cmath>
#include "Vec.h"

namespace GCore
{

	template <typename T = float>
	class Mat4x4
	{
	public:

		Mat4x4()
		{
			mElem[0] = 1; mElem[1] = 0; mElem[2] = 0; mElem[3] = 0;
			mElem[4] = 0; mElem[5] = 1; mElem[6] = 0; mElem[7] = 0;
			mElem[8] = 0; mElem[9] = 0; mElem[10] = 1; mElem[11] = 0;
			mElem[12] = 0; mElem[13] = 0; mElem[14] = 0; mElem[15] = 1;
		}

		Mat4x4(T* mat)
		{
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mElem[i * 4 + j] = mat[i * 4 + j];
		}

		void setMatrix(T* mat)
		{
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					mElem[i * 4 + j] = mat[i * 4 + j];
		}

		T& operator()(int row, int col)
		{
			return mElem[row * 4 + col];
		}

		const T& operator()(int row, int col)const
		{
			return mElem[row * 4 + col];
		}

		T determinant()
		{
			return mElem[0] * det(0, 0) - mElem[1] * det(0, 1) + mElem[2] * det(0, 2) - mElem[3] * det(0, 3);
		}

		T det(int r, int c)
		{
			T tElem[9];
			int k = 0;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
				{
					if (i == r || j == c)
						continue;
					tElem[k++] = mElem[i * 4 + j];
				}
			// 0 1 2
			// 3 4 5
			// 6 7 8
			return tElem[0] * (tElem[4] * tElem[8] - tElem[5] * tElem[7])
				- tElem[1] * (tElem[3] * tElem[8] - tElem[5] * tElem[6])
				+ tElem[2] * (tElem[3] * tElem[7] - tElem[4] * tElem[6]);
		}

		Mat4x4 operator*(const Mat4x4& mat)const
		{
			Mat4x4 r;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
				{
					T sum = 0;
					for (int k = 0; k < 4; k++)
						sum += (*this)(i, k)*mat(k, j);
					r(i, j) = sum;
				}
			return r;
		}

		void setIdentity()
		{
			for (int i = 0; i < 4; i++)
				mElem[4 * i + i] = 1;
		}

		Vec3<T> transform(const Vec3<T>& vec3)const
		{
			Vec3<T> rVec3;
			rVec3.x = vec3.x*mElem[0] + vec3.y*mElem[1] + vec3.z*mElem[2] + mElem[3];
			rVec3.y = vec3.x*mElem[4] + vec3.y*mElem[5] + vec3.z*mElem[6] + mElem[7];
			rVec3.z = vec3.x*mElem[8] + vec3.y*mElem[9] + vec3.z*mElem[10] + mElem[11];
			return rVec3;
		}

		static Mat4x4 orthographicProj(T left, T right, T bottom, T top, T near, T far)
		{
			Mat4x4 r;
			r(0, 0) = 2 / (right - left);
			r(1, 1) = 2 / (top - bottom);
			r(2, 2) = -2 / (far - near);
			r(3, 0) = -(right + left) / (right - left);
			r(3, 1) = -(top + bottom) / (top - bottom);
			r(3, 2) = -(far + near) / (far - near);
			return r;
		}

		static Mat4x4 frustumProj(T left, T right, T bottom, T top, T near, T far)
		{
			Mat4x4 r;
			r(0, 0) = 2 * near / (right - left);
			r(1, 1) = 2 * near / (top - bottom);
			r(2, 0) = (right + left) / (right - left);
			r(2, 1) = (top + bottom) / (top - bottom);
			r(2, 2) = -(far + near) / (far - near);
			r(2, 3) = -1;
			r(3, 2) = -2 * far*near / (far - near);
			r(3, 3) = 0;
			return r;
		}

		static Mat4x4 perspectiveProj(T fov, T aspect, T near, T far)
		{
			fov = PI*fov / 180.0;
			T tanfov = tan(fov / 2.0);

			Mat4x4 r;
			r(0, 0) = 1 / (aspect*tanfov);
			r(1, 1) = 1 / tanfov;
			r(2, 2) = -(far + near) / (far - near);
			r(2, 3) = -1;
			r(3, 2) = -2 * far*near / (far - near);
			r(3, 3) = 0;
			return r;
		}

		static Mat4x4 lookAt(const Vec3<T>& eye, const Vec3<T>& at, const Vec3<T>& up)
		{
			Vec3<T> f = at - eye;
			f.normalize();
			Vec3<T> s = f.cross(up);
			s.normalize();
			Vec3<T> u = s.cross(f);
			u.normalize();

			Mat4x4 r;
			r(0, 0) = s.x;
			r(1, 0) = s.y;
			r(2, 0) = s.z;
			r(0, 1) = u.x;
			r(1, 1) = u.y;
			r(2, 1) = u.z;
			r(0, 2) = -f.x;
			r(1, 2) = -f.y;
			r(2, 2) = -f.z;
			r(3, 0) = -s.dot(eye);
			r(3, 1) = -u.dot(eye);
			r(3, 2) = f.dot(eye);
			return r;
		}

		static Mat4x4 rotate(T angle, T x, T y, T z)
		{
			T cosv = cos(angle);
			T sinv = sin(angle);
			Mat4x4 r;

			r(0, 0) = x*x*(1 - cosv) + cosv;
			r(0, 1) = x*y*(1 - cosv) - z*sinv;
			r(0, 2) = x*z*(1 - cosv) + y*sinv;

			r(1, 0) = x*y*(1 - cosv) + z*sinv;
			r(1, 1) = y*y*(1 - cosv) + cosv;
			r(1, 2) = y*z*(1 - cosv) - x*sinv;

			r(2, 0) = x*z*(1 - cosv) - y*sinv;
			r(2, 1) = y*z*(1 - cosv) + x*sinv;
			r(2, 2) = z*z*(1 - cosv) + cosv;

			return r;

		}

		static Mat4x4 scale(T scalex, T scaley, T scalez)
		{
			Mat4x4 r;
			r(0, 0) = scalex;
			r(1, 1) = scaley;
			r(2, 2) = scalez;
			return r;
		}

		static Mat4x4 translate(T translatex, T translatey, T translatez)
		{
			Mat4x4 r;
			r(0, 3) = translatex;
			r(1, 3) = translatey;
			r(2, 3) = translatez;
			return r;
		}

		static Mat4x4 rotateBetweenAxes(const Vec3<T>& from, const Vec3<T>& to)
		{
			Mat4x4 xyMat;

			const Vec3<T> nfrom = from.normalize();
			const Vec3<T> nto = to.normalize();

			Vec3<T> rotAxis = nfrom.cross(nto);
			if (!isZero<T>(rotAxis.magnitude()))
			{
				rotAxis.normalize();
				double rotAngle = std::acos(nfrom.dot(nto));
				if(!std::isinf(rotAngle))
					xyMat = GCore::Mat4x4d::rotate(rotAngle, rotAxis.x, rotAxis.y, rotAxis.z)*xyMat;
			}

			return xyMat;
		}

		T* matrix()const
		{
			return mElem;
		}

		T* matrix()
		{
			return mElem;
		}

	private:
		T mElem[16];
	};
}
