/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * histogram.cpp - histogram calculations
 */
#include <math.h>
#include <stdio.h>

#include "histogram.hpp"

uint64_t Histogram::CumulativeFreq(double bin) const
{
	if (bin <= 0)
		return 0;
	else if (bin >= Bins())
		return Total();
	int b = (int)bin;
	return cumulative_[b] +
	       (bin - b) * (cumulative_[b + 1] - cumulative_[b]);
}

double Histogram::Quantile(double q, int first, int last) const
{
	if (first == -1)
		first = 0;
	if (last == -1)
		last = cumulative_.size() - 2;
	assert(first <= last);
	uint64_t items = q * Total();
	while (first < last) // binary search to find the right bin
	{
		int middle = (first + last) / 2;
		if (cumulative_[middle + 1] > items)
			last = middle; // between first and middle
		else
			first = middle + 1; // after middle
	}
	assert(items >= cumulative_[first] && items <= cumulative_[last + 1]);
	double frac = cumulative_[first + 1] == cumulative_[first] ? 0
		      : (double)(items - cumulative_[first]) /
				  (cumulative_[first + 1] - cumulative_[first]);
	return first + frac;
}

double Histogram::InterQuantileMean(double q_lo, double q_hi) const
{
	assert(q_hi > q_lo);
	double p_lo = Quantile(q_lo);
	double p_hi = Quantile(q_hi, (int)p_lo);
	double sum_bin_freq = 0, cumul_freq = 0;
	for (double p_next = floor(p_lo) + 1.0; p_next <= ceil(p_hi);
	     p_lo = p_next, p_next += 1.0) {
		int bin = floor(p_lo);
		double freq = (cumulative_[bin + 1] - cumulative_[bin]) *
			      (std::min(p_next, p_hi) - p_lo);
		sum_bin_freq += bin * freq;
		cumul_freq += freq;
	}
	// add 0.5 to give an average for bin mid-points
	return sum_bin_freq / cumul_freq + 0.5;
}
