/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_hdr.cpp - simple libcamera HDR app.
 */

// WARNING: there are many reasons why I wouldn't normally do HDR like this,
// and I won't go into them all. Still, we don't really have the APIs to do it
// in the Bayer domain, so munging together fully processed YUV images is what
// we're going to try and do. The signals are all non-linear therefore, meaning
// that the whole thing is a bit of a dog's dinner. Still, this is demo code,
// just for a bit of fun, and is not formally supported. You have been warned!

// To try it out, just run the executable (libcamera-hdr) with no arguments.
// You'll get a short exposure version of the image at the preview resolution
// (short.jpg), and an HDR version at the full capture resolution (hdr.jpg).

#include "still_options.hpp"
#include "libcamera_app.hpp"
#include "hdr.hpp"

using namespace std::placeholders;
class LibcameraHdr : public LibcameraApp
{
public:
	LibcameraHdr() : LibcameraApp(std::make_unique<StillOptions>()) {}

	StillOptions *GetOptions() const { return static_cast<StillOptions *>(options_.get()); }
};

using BufferMap = LibcameraHdr::BufferMap;
using Stream = LibcameraHdr::Stream;
using ControlList = libcamera::ControlList;

// In jpeg.cpp:
void jpeg_save(std::vector<void *> const &mem, int w, int h, int stride,
			   libcamera::PixelFormat const &pixel_format,
			   libcamera::ControlList const &metadata,
			   std::string const &filename,
			   std::string const &cam_name,
			   StillOptions const *options);

struct HdrConfig
{
	unsigned int num_frames;   // capture and combine this many frames
	HdrImage::LpFilterConfig lp_filter;
	double fixed_q;            // fixed point in tonemap
	Pwl q50_curve;             // where tonemap should move the q50 point
	double q25_factor;         // how to adjust the q25 point relative to the q50 one
	HdrImage::TonemapConfig tonemap;
	Pwl exposure_adjust;       // exposure adjustment for dark images
};
static HdrConfig HDR_CONFIG =
{
	8, // num_frames, 1 to 16 should work
	{ // lp_filter
		0.2, // strength
		Pwl({ { 0,    10 }, // threshold
			  { 2048, 2048 * 0.1 },
			  { 4095, 2048 * 0.1 } })
	},
	0.03, // fixed_q
	Pwl({ { 0,    400 }, // q50_curve
		  { 30,   500 },
		  { 100,  600 },
		  { 200,  800 },
		  { 300,  1000 },
		  { 2048, 2048 },
		  { 4095, 3072 } }),
	0.667, // q25_factor
	{ // tonemap
		Pwl(), // tonemap (gets filled in later)
		Pwl({ { 0, 6.0 }, // pos_strength
			  { 1024, 2.0 },
			  { 4095, 2.0 }	}),
		Pwl({ { 0, 4.0 }, // neg_strength
			  { 1024, 1.5 },
			  { 4095, 1.5 }	})
	},
	Pwl({ { 0,   2.0 }, // exposure_adjust
		  { 2.0, 1.5 },
		  { 8.0, 1.0 } })
};

static constexpr int PREVIEW_FRAMES = 60;

// Create some kind of a tonemap to apply to the image.

static Pwl create_tonemap(HdrImage const &im, HdrConfig &hdr_config)
{
	int maxval = im.dynamic_range - 1;
	Histogram histogram = im.CalculateHistogram();
	// The "fixed_q" point won't be moved, that allows us to keep some degree of
	// contrast at the bottom of the dynamic range.
	double q_fixed = histogram.Quantile(hdr_config.fixed_q);
	double target_fixed = q_fixed;
	// The "q50" point will get shifted according to the Pwl in the config.
	double q50 = histogram.Quantile(0.5);
	double target50 = hdr_config.q50_curve.Eval(q50);
	// We allow the "q25" (lower quartile point) so be moved relative to the q50 one.
	double q25 = histogram.Quantile(0.25);
	double target25 = target50 * hdr_config.q25_factor;

	Pwl tonemap;
	tonemap.Append(0, 0);
	tonemap.Append(q_fixed, target_fixed);
	tonemap.Append(q25, target25);
	tonemap.Append(q50, target50);
	tonemap.Append(maxval, maxval);

	return tonemap;
}

static double get_exposure_adjustment(void *mem, int w, int h, int stride, Pwl const &exposure_adjust)
{
	// Bit of a bodge, really. The "highlight" metering method really does stop almost everything
	// from blowing out, but if the 10% point of the histogram is crazily low, then we're better
	// off overall boosting the exposure a bit.
	uint32_t bins[256] = {};
	uint8_t *ptr = (uint8_t *)mem;
	for (int j = 0; j < h; j++)
	{
		int off = j * stride;
		for (int i = 0; i < w; i++, off++)
			bins[ptr[off]]++;
	}
	Histogram hist(bins, 256);
	double q10 = hist.Quantile(0.1);
	return exposure_adjust.Eval(exposure_adjust.Domain().Clip(q10));
}

// The main event loop for the application.

static void event_loop(LibcameraHdr &app)
{
	StillOptions const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();
	int w = 0, h = 0, stride = 0;
	HdrImage acc;
	Stream *stream = nullptr;

	// We're going to meter for the highlights. To tune the behaviour, check out the
	// "highlight" constraint mode in the json file.
	ControlList controls;
	controls.set(controls::AeConstraintMode, controls::ConstraintHighlight);
	app.SetControls(controls);
	app.StartCamera();
	app.SetPreviewDoneCallback(std::bind(&LibcameraHdr::QueueRequest, &app, _1));

	for (unsigned int count = 0; ; count++)
	{
		LibcameraHdr::Msg msg = app.Wait();
		if (msg.type == LibcameraHdr::MsgType::Quit)
			return;
		else if (msg.type != LibcameraHdr::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequest completed_request = std::get<CompletedRequest>(msg.payload);
		BufferMap &buffers = completed_request.buffers;
		ControlList &metadata = completed_request.metadata;

		if (count < PREVIEW_FRAMES)
			app.ShowPreview(completed_request, app.ViewfinderStream());
		else if (count == PREVIEW_FRAMES)
		{
			float exposure_time, ag, dg, colour_gains[2];
			FrameInfo frame_info(metadata);

			app.StopCamera();

			// Save this image, why not.
			std::cout << "Save short.jpg" << std::endl;
			int vf_w, vf_h, vf_stride;
			Stream *vf_stream = app.ViewfinderStream(&vf_w, &vf_h, &vf_stride);
			jpeg_save(app.Mmap(buffers[vf_stream]), vf_w, vf_h, vf_stride,
					  vf_stream->configuration().pixelFormat,
					  metadata, "short.jpg", app.CameraId(), options);

			// This will boost the exposure, allowing a bit more stuff to blow out,
			// if there's really tons of stuff right at the bottom of the histogram.
			float exp_adjust = get_exposure_adjustment(app.Mmap(buffers[vf_stream])[0],
													   vf_w, vf_h, vf_stride,
													   HDR_CONFIG.exposure_adjust);

			// Now restart us in stills capture mode. Triple-buffering is required
			// for us not to drop frames. If you run out of memory doing this, check
			// the "Tips" in the README.md for how to increase it.
			app.Teardown();
			app.ConfigureStill(LibcameraHdr::FLAG_STILL_TRIPLE_BUFFER);
			stream = app.StillStream(&w, &h, &stride);
			acc = HdrImage(w, h, w * h * 3 / 2); // YUV420
			acc.Clear();

			// We expose for the highlights as before, but accumulate multiple frames.
			ControlList controls;
			controls.set(controls::ExposureTime, (int32_t)(frame_info.exposure_time * exp_adjust));
			controls.set(controls::AnalogueGain, frame_info.analogue_gain * frame_info.digital_gain);
			controls.set(controls::ColourGains, frame_info.colour_gains);
			app.SetControls(controls);
			app.StartCamera();
		}
		else
		{
			// Add to accumulator image. This will only work well for static scenes.
			std::cout << "Accumulate image " << count - PREVIEW_FRAMES << std::endl;
			acc.Accumulate(app.Mmap(buffers[stream])[0], stride);

			if (count == PREVIEW_FRAMES + HDR_CONFIG.num_frames)
			{
				app.StopCamera();
				std::cout << "HDR processing starting" << std::endl;

				// Values are tuned for 16 frames, so scale acc up to match.
				acc.Scale(16.0 / HDR_CONFIG.num_frames);
				HdrImage lp = acc.LpFilter(HDR_CONFIG.lp_filter);

				HDR_CONFIG.tonemap.tonemap = create_tonemap(lp, HDR_CONFIG);
				acc.Tonemap(lp, HDR_CONFIG.tonemap);

				std::vector<uint8_t> output = acc.Extract(stride);

				std::cout << "Save hdr.jpg" << std::endl;
				jpeg_save({ &output[0] }, w, h, stride, stream->configuration().pixelFormat, metadata, "hdr.jpg",
						  app.CameraId(), options);
				return;
			}

			app.ShowPreview(completed_request, stream);
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraHdr app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();
			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
    }
	return 0;
}
