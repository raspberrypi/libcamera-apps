/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * hdr.cpp - class to help HDR image processing
 */

#include <thread>
#include <iostream>

#include "hdr.hpp"

static void add_Y_pixels(int16_t *dest, uint8_t const *src, int width, int stride, int height)
{
	for (int y = 0; y < height; y++, src += stride)
	{
		for (int x = 0; x < width; x++)
			*(dest++) += src[x];
	}
}

// Add the new image buffer to this "accumulator" image. We just add them as
// we don't have the horsepower to do any fancy alignment or anything.
// Actually, spreading it across a few threads doesn't seem to help much, though
// compiling with "gcc -mfpu=neon-fp-armv8 -ftree-vectorize" gives a big
// improvement.

void HdrImage::Accumulate(void const *image_buffer, int stride)
{
	int16_t *dest = &P(0);
	uint8_t const *src = (uint8_t *)image_buffer;
	int width2 = width / 2, height2 = height / 2, stride2 = stride / 2;
	std::thread thread1(add_Y_pixels, dest, src, width, stride, height2);
	std::thread thread2(add_Y_pixels, dest + width * height2,
						src + stride * height2, width, stride, height2);

	dest += width * height;
	src += stride * height;

	// U and V components
	for (int y = 0; y < height; y++, src += stride2)
	{
		for (int x = 0; x < width2; x++)
			*(dest++) += src[x] - 128;
	}

	dynamic_range += 256;

	thread1.join();
	thread2.join();
}

// Forward pass of the IIR low pass filter.

static void forward_pass(std::vector<double> &fwd_pixels, std::vector<double> &fwd_weight_sums,
						 HdrImage const &in,
						 std::vector<double> &weights, std::vector<double> &threshold,
						 int width, int height, int size, double strength)

{
	// (Should probably initialise the top/left elements of fwd_pixels/fwd_weight_sums...)
	for (int y = size; y < height; y++)
	{
		unsigned int off = y * width + size;
		for (int x = size; x < width; x++, off++)
		{
			int pixel = in.P(off);
			double thresh = threshold[pixel];
			double pixel_wt_sum = pixel * strength, wt_sum = strength;
			std::vector<int> deltas = { -width - 1, -width, -width + 1, -1 };
			for (auto delta : deltas)
			{
				double P = fwd_pixels[off + delta];
				int idx = abs(P - pixel) * 10 / thresh;
				double wt = idx >= weights.size() ? 0 : weights[idx];
				pixel_wt_sum += wt * P;
				wt_sum += wt;
			}
			fwd_pixels[off] = pixel_wt_sum / wt_sum;
			fwd_weight_sums[off] = wt_sum;
		}
	}
}

// Low pass IIR filter. We perform a forwards and a reverse pass, finally combining
// the results to get a smoothed but vaguely edge-preserving version of the
// accumulator image. You could imagine implementing alternative (more sophisticated)
// filters.

HdrImage HdrImage::LpFilter(LpFilterConfig const &config) const
{
	// Cache threshold values, computing them would be slow.
	std::vector<double> threshold = config.threshold.GenerateLut<double>();

	// Cache values of e^(-x^2) for 0 <= x <= 3, it will be much quicker
	std::vector<double> weights(31);
	for (int d = 0; d <= 30; d++)
		weights[d] = exp(-d * d / 100.0);

	int size = 1;
	double strength = config.strength;

	// Forward pass.
	std::vector<double> fwd_weight_sums(width * height);
	std::vector<double> fwd_pixels(width * height);

	HdrImage out(width, height, width * height);
	out.dynamic_range = dynamic_range;

	// Run the forward pass in other thread, so that the two passes run in parallel.
	std::thread fwd_pass(forward_pass, std::ref(fwd_pixels), std::ref(fwd_weight_sums),
						 std::ref(*this), std::ref(weights), std::ref(threshold),
						 width, height, size, strength);

	// Reverse pass, but otherwise the same as the forward pass. There could be a small
	// saving in omitting it, but it's not huge given that they run in parallel.
	std::vector<double> rev_weight_sums(width * height);
	std::vector<double> rev_pixels(width * height);
	// (Should probably initialise the bottom/right elements of rev_pixels/rev_weight_sums...)
	for (int y = height - 1 - size; y >= 0; y--)
	{
		unsigned int off = y * width + width - 1 - size;
		for (int x = width - 1 - size; x >= 0; x--, off--)
		{
			int pixel = P(off);
			double thresh = threshold[pixel];
			double pixel_wt_sum = pixel * strength, wt_sum = strength;
			std::vector<int> deltas = { width + 1, width, width - 1, 1 };
			for (auto delta : deltas)
			{
				double P = rev_pixels[off + delta];
				int idx = abs(P - pixel) * 10 / thresh;
				double wt = idx >= weights.size() ? 0 : weights[idx];
				pixel_wt_sum += wt * P;
				wt_sum += wt;
			}
			rev_pixels[off] = pixel_wt_sum / wt_sum;
			rev_weight_sums[off] = wt_sum;
		}
	}

	fwd_pass.join();

	// Combine.
	for (int y = 0; y < height; y++)
	{
		unsigned int off = y * width;
		for (int x = 0; x < width; x++, off++)
			out.P(off) = (fwd_pixels[off] * fwd_weight_sums[off] + rev_pixels[off] * rev_weight_sums[off]) / (fwd_weight_sums[off] + rev_weight_sums[off]);
	}

	return out;
}

Histogram HdrImage::CalculateHistogram() const
{
	std::vector<uint32_t> bins(dynamic_range);
	std::fill(bins.begin(), bins.end(), 0);
	for (int i = 0; i < width * height; i++)
		bins[P(i)]++;
	return Histogram(&bins[0], dynamic_range);
}

// The idea is that we tonemap the low pass image, and then add back the high pass
// signal (the difference between the original and the lp image), with some amount
// of gain.

void HdrImage::Tonemap(HdrImage const &lp, TonemapConfig const &config)
{
	// Make LUTs for the all the Pwls, it'll be much quicker.
	std::vector<int> tonemap_lut = config.tonemap.GenerateLut<int>();
	std::vector<double> pos_strength_lut = config.pos_strength.GenerateLut<double>();
	std::vector<double> neg_strength_lut = config.neg_strength.GenerateLut<double>();

	int maxval = dynamic_range - 1;
	for (int y = 0; y < height; y++)
	{
		unsigned int off_Y = y * width;
		unsigned int off_U = y * width / 4 + width * height;
		unsigned int off_V = off_U + width * height / 4;
		for (int x = 0; x < width; x++, off_Y++)
		{
			int Y_lp_orig = lp.P(off_Y), Y_hp = P(off_Y) - Y_lp_orig;
			int Y_lp_mapped = tonemap_lut[Y_lp_orig];
			double strength = (Y_hp > 0 ? pos_strength_lut : neg_strength_lut)[Y_lp_orig];
			int Y_final = std::clamp(Y_lp_mapped + (int)(strength * Y_hp), 0, maxval);
			P(off_Y) = Y_final;
			if (!(x & 1) && !(y & 1))
			{
				float f = (Y_final + 1) / (float)(Y_lp_orig + 1);
				//f /= sqrt(sqrt(sqrt(f))); // if you feel the colours are a bit strong?
				int U = P(off_U), V = P(off_V);
				P(off_U) = U * f;
				P(off_V) = V * f;
				off_U++, off_V++;
			}
		}
	}
}

// Extract the HDR image into an 8bpp buffer with the given line-to-line stride.

std::vector<uint8_t> HdrImage::Extract(int stride) const
{
	std::vector<uint8_t> dest(stride * height * 3 / 2);
	double ratio = dynamic_range / 256;
	const int16_t *Y_ptr = &pixels[0];
	const int16_t *U_ptr = Y_ptr + width * height, *V_ptr = U_ptr + width * height / 4;
	uint8_t *dest_y = &dest[0];
	uint8_t *dest_u = dest_y + stride * height, *dest_v = dest_u + stride * height / 4;

	for (int y = 0; y < height; y++, dest_y += stride)
	{
		for (int x = 0; x < width; x++)
		{
			dest_y[x] = *(Y_ptr++) / ratio;
			if (!(x & 1) && !(y & 1))
			{
				int U = *(U_ptr++) / ratio;
				int V = *(V_ptr++) / ratio;
				dest_u[x / 2] = std::clamp(U + 128, 0, 255);
				dest_v[x / 2] = std::clamp(V + 128, 0, 255);
			}
		}
		if (!(y & 1))
			dest_u += stride / 2, dest_v += stride / 2;
	}

	return dest;
}

// Simple linear scaling of the image by the given factor.

void HdrImage::Scale(double factor)
{
	for (unsigned int i = 0; i < pixels.size(); i++)
		pixels[i] *= factor;
	dynamic_range *= factor;
}
