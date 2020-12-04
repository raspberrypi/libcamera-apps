/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * pwl.hpp - piecewise linear functions interface
 */
#pragma once

#include <math.h>

#include <functional>
#include <vector>

class Pwl
{
public:
	struct Interval {
		Interval(double _start, double _end) : start(_start), end(_end) {}
		double start, end;
		bool Contains(double value) { return value >= start && value <= end; }
		double Clip(double value) { return value < start ? start : (value > end ? end : value); }
		double Len() const { return end - start; }
	};
	struct Point {
		Point() : x(0), y(0) {}
		Point(double _x, double _y) : x(_x), y(_y) {}
		double x, y;
		Point operator-(Point const &p) const { return Point(x - p.x, y - p.y); }
		Point operator+(Point const &p) const { return Point(x + p.x, y + p.y); }
		double operator%(Point const &p) const { return x * p.x + y * p.y; }
		Point operator*(double f) const { return Point(x * f, y * f); }
		Point operator/(double f) const { return Point(x / f, y / f); }
		double Len2() const { return x * x + y * y; }
		double Len() const { return sqrt(Len2()); }
	};
	Pwl() {}
	Pwl(std::vector<Point> const &points) : points_(points) {}
	void Append(double x, double y, const double eps = 1e-6);
	void Prepend(double x, double y, const double eps = 1e-6);
	Interval Domain() const;
	Interval Range() const;
	bool Empty() const;
	// Evaluate Pwl, optionally supplying an initial guess for the
	// "span". The "span" may be optionally be updated.  If you want to know
	// the "span" value but don't have an initial guess you can set it to
	// -1.
	double Eval(double x, int *span_ptr = nullptr, bool update_span = true) const;
	// Find perpendicular closest to xy, starting from span+1 so you can
	// call it repeatedly to check for multiple closest points (set span to
	// -1 on the first call). Also returns "pseudo" perpendiculars; see
	// PerpType enum.
	enum class PerpType {
		NotFound, // no perpendicular found
		Start, // start of Pwl is closest point
		End, // end of Pwl is closest point
		Vertex, // vertex of Pwl is closest point
		Perpendicular // true perpendicular found
	};
	PerpType Invert(Point const &xy, Point &perp, int &span, const double eps = 1e-6) const;
	// Compose two Pwls together, doing "this" first and "other" after.
	Pwl Compose(Pwl const &other, const double eps = 1e-6) const;
	// Apply function to (x,y) values at every control point.
	void Map(std::function<void(double x, double y)> f) const;
	// Apply function to (x, y0, y1) values wherever either Pwl has a
	// control point.
	static void Map2(Pwl const &pwl0, Pwl const &pwl1,
					 std::function<void(double x, double y0, double y1)> f);
	// Combine two Pwls, meaning we create a new Pwl where the y values are
	// given by running f wherever either has a knot.
	static Pwl Combine(Pwl const &pwl0, Pwl const &pwl1,
					   std::function<double(double x, double y0, double y1)> f,
					   const double eps = 1e-6);
	// Make "this" match (at least) the given domain. Any extension my be
	// clipped or linear.
	void MatchDomain(Interval const &domain, bool clip = true, const double eps = 1e-6);
	// Generate a LUT for this funciton.
	template <typename T> std::vector<T> GenerateLut() const
	{
		int end = Domain().end + 1, span = 0;
		std::vector<T> lut(end);
		for (int x = 0; x < end; x++)
			lut[x] = Eval(x, &span);
		return lut;
	}
	Pwl &operator*=(double d);
	void Debug(FILE *fp = stdout) const;

private:
	int findSpan(double x, int span) const;
	std::vector<Point> points_;
};
