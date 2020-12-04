/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * hdr.hpp - class to help HDR image processing
 */
#pragma once

#include "pwl.hpp"
#include "histogram.hpp"

struct HdrImage
{
	HdrImage() : width(0), height(0), dynamic_range(0) {}
	HdrImage(int w, int h, int num_pixels)
		: width(w), height(h), pixels(num_pixels), dynamic_range(0) {}
	int width;
	int height;
	std::vector<int16_t> pixels;
	int dynamic_range; // 1 more than the maximum pixel value
	int16_t &P(unsigned int offset) { return pixels[offset]; }
	int16_t P(unsigned int offset) const { return pixels[offset]; }
	void Clear() { std::fill(pixels.begin(), pixels.end(), 0); }
	void Accumulate(void const *image_buffer, int stride);
	struct LpFilterConfig {
		double strength;
		Pwl threshold;
	};
	HdrImage LpFilter(LpFilterConfig const &config) const;
	struct TonemapConfig {
		Pwl tonemap;
		Pwl pos_strength;
		Pwl neg_strength;
	};
	void Tonemap(HdrImage const &lp, TonemapConfig const &config);
	std::vector<uint8_t> Extract(int stride) const;
	Histogram CalculateHistogram() const;
	void Scale(double factor);
};

