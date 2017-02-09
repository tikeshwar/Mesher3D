#pragma once

#include "Defines.h"
#include <cmath>

namespace GCore
{
	template <typename T = double>
	constexpr bool isEqual(T left, T right)
	{
		return std::fabs(left - right) < TOL;
	}

	template <typename T = double>
	constexpr bool isGreater(T left, T right)
	{
		return left - right > TOL;
	}

	template <typename T = double>
	constexpr bool isGreaterOrEqual(T left, T right)
	{
		return isGreater(left, right) || isEqual(left, right);
	}

	template <typename T = double>
	constexpr bool isSmaller(T left, T right)
	{
		return right - left > TOL;
	}


	template <typename T = double>
	constexpr bool isSmallerOrEqual(T left, T right)
	{
		return isSmaller(left, right) || isEqual(left, right);
	}

	template <typename T = double>
	constexpr bool isZero(T num)
	{
		return fabs(num) < ZERO_TOL;
	}

	template <typename T = double>
	constexpr bool isPositive(T num)
	{
		return isZero(num) ? true : num > TOL;
	}

	template <typename T = double>
	constexpr bool isNegative(T num)
	{
		return isZero(num) ? true : -num > TOL;
	}
}
