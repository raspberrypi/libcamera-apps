/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * pwl.cpp - piecewise linear functions
 */

#include <cassert>
#include <stdexcept>

#include "pwl.hpp"

void Pwl::Append(double x, double y, const double eps)
{
	if (points_.empty() || points_.back().x + eps < x)
		points_.push_back(Point(x, y));
}

void Pwl::Prepend(double x, double y, const double eps)
{
	if (points_.empty() || points_.front().x - eps > x)
		points_.insert(points_.begin(), Point(x, y));
}

Pwl::Interval Pwl::Domain() const
{
	return Interval(points_[0].x, points_[points_.size() - 1].x);
}

Pwl::Interval Pwl::Range() const
{
	double lo = points_[0].y, hi = lo;
	for (auto &p : points_)
		lo = std::min(lo, p.y), hi = std::max(hi, p.y);
	return Interval(lo, hi);
}

bool Pwl::Empty() const
{
	return points_.empty();
}

double Pwl::Eval(double x, int *span_ptr, bool update_span) const
{
	int span = findSpan(x, span_ptr && *span_ptr != -1 ? *span_ptr : points_.size() / 2 - 1);
	if (span_ptr && update_span)
		*span_ptr = span;
	return points_[span].y +
		(x - points_[span].x) * (points_[span + 1].y - points_[span].y) /
		       (points_[span + 1].x - points_[span].x);
}

int Pwl::findSpan(double x, int span) const
{
	// Pwls are generally small, so linear search may well be faster than
	// binary, though could review this if large PWls start turning up.
	int last_span = points_.size() - 2;
	// some algorithms may call us with span pointing directly at the last
	// control point
	span = std::max(0, std::min(last_span, span));
	while (span < last_span && x >= points_[span + 1].x)
		span++;
	while (span && x < points_[span].x)
		span--;
	return span;
}

Pwl::PerpType Pwl::Invert(Point const &xy, Point &perp, int &span, const double eps) const
{
	assert(span >= -1);
	bool prev_off_end = false;
	for (span = span + 1; span < (int)points_.size() - 1; span++) {
		Point span_vec = points_[span + 1] - points_[span];
		double t = ((xy - points_[span]) % span_vec) / span_vec.Len2();
		if (t < -eps) // off the start of this span
		{
			if (span == 0) {
				perp = points_[span];
				return PerpType::Start;
			} else if (prev_off_end) {
				perp = points_[span];
				return PerpType::Vertex;
			}
		} else if (t > 1 + eps) // off the end of this span
		{
			if (span == (int)points_.size() - 2) {
				perp = points_[span + 1];
				return PerpType::End;
			}
			prev_off_end = true;
		} else // a true perpendicular
		{
			perp = points_[span] + span_vec * t;
			return PerpType::Perpendicular;
		}
	}
	return PerpType::NotFound;
}

Pwl Pwl::Compose(Pwl const &other, const double eps) const
{
	double this_x = points_[0].x, this_y = points_[0].y;
	int this_span = 0, other_span = other.findSpan(this_y, 0);
	Pwl result({ { this_x, other.Eval(this_y, &other_span, false) } });
	while (this_span != (int)points_.size() - 1) {
		double dx = points_[this_span + 1].x - points_[this_span].x,
		       dy = points_[this_span + 1].y - points_[this_span].y;
		if (abs(dy) > eps &&
		    other_span + 1 < (int)other.points_.size() &&
		    points_[this_span + 1].y >=
			    other.points_[other_span + 1].x + eps) {
			// next control point in result will be where this
			// function's y reaches the next span in other
			this_x = points_[this_span].x +
				 (other.points_[other_span + 1].x -
				  points_[this_span].y) * dx / dy;
			this_y = other.points_[++other_span].x;
		} else if (abs(dy) > eps && other_span > 0 &&
			   points_[this_span + 1].y <=
				   other.points_[other_span - 1].x - eps) {
			// next control point in result will be where this
			// function's y reaches the previous span in other
			this_x = points_[this_span].x +
				 (other.points_[other_span + 1].x -
				  points_[this_span].y) * dx / dy;
			this_y = other.points_[--other_span].x;
		} else {
			// we stay in the same span in other
			this_span++;
			this_x = points_[this_span].x,
			this_y = points_[this_span].y;
		}
		result.Append(this_x, other.Eval(this_y, &other_span, false), eps);
	}
	return result;
}

void Pwl::Map(std::function<void(double x, double y)> f) const
{
	for (auto &pt : points_)
		f(pt.x, pt.y);
}

void Pwl::Map2(Pwl const &pwl0, Pwl const &pwl1,
			   std::function<void(double x, double y0, double y1)> f)
{
	int span0 = 0, span1 = 0;
	double x = std::min(pwl0.points_[0].x, pwl1.points_[0].x);
	f(x, pwl0.Eval(x, &span0, false), pwl1.Eval(x, &span1, false));
	while (span0 < (int)pwl0.points_.size() - 1 || span1 < (int)pwl1.points_.size() - 1) {
		if (span0 == (int)pwl0.points_.size() - 1)
			x = pwl1.points_[++span1].x;
		else if (span1 == (int)pwl1.points_.size() - 1)
			x = pwl0.points_[++span0].x;
		else if (pwl0.points_[span0 + 1].x > pwl1.points_[span1 + 1].x)
			x = pwl1.points_[++span1].x;
		else
			x = pwl0.points_[++span0].x;
		f(x, pwl0.Eval(x, &span0, false), pwl1.Eval(x, &span1, false));
	}
}

Pwl Pwl::Combine(Pwl const &pwl0, Pwl const &pwl1,
				 std::function<double(double x, double y0, double y1)> f, const double eps)
{
	Pwl result;
	Map2(pwl0, pwl1, [&](double x, double y0, double y1) {
		result.Append(x, f(x, y0, y1), eps);
	});
	return result;
}

void Pwl::MatchDomain(Interval const &domain, bool clip, const double eps)
{
	int span = 0;
	Prepend(domain.start, Eval(clip ? points_[0].x : domain.start, &span), eps);
	span = points_.size() - 2;
	Append(domain.end, Eval(clip ? points_.back().x : domain.end, &span), eps);
}

Pwl &Pwl::operator*=(double d)
{
	for (auto &pt : points_)
		pt.y *= d;
	return *this;
}

void Pwl::Debug(FILE *fp) const
{
	fprintf(fp, "Pwl {\n");
	for (auto &p : points_)
		fprintf(fp, "\t(%g, %g)\n", p.x, p.y);
	fprintf(fp, "}\n");
}
