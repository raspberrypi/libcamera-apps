/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * histogram.hpp - histogram calculation interface
 */
#pragma once

#include <stdint.h>
#include <vector>
#include <cassert>

// A simple histogram class, for use in particular to find "quantiles" and
// averages between "quantiles".

class Histogram
{
public:
	template<typename T> Histogram(T *histogram, int num)
	{
		assert(num);
		cumulative_.reserve(num + 1);
		cumulative_.push_back(0);
		for (int i = 0; i < num; i++)
			cumulative_.push_back(cumulative_.back() +
					      histogram[i]);
	}
	uint32_t Bins() const { return cumulative_.size() - 1; }
	uint64_t Total() const { return cumulative_[cumulative_.size() - 1]; }
	// Cumulative frequency up to a (fractional) point in a bin.
	uint64_t CumulativeFreq(double bin) const;
	// Return the (fractional) bin of the point q (0 <= q <= 1) through the
	// histogram. Optionally provide limits to help.
	double Quantile(double q, int first = -1, int last = -1) const;
	// Return the average histogram bin value between the two quantiles.
	double InterQuantileMean(double q_lo, double q_hi) const;

private:
	std::vector<uint64_t> cumulative_;
};
