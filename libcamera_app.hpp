/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_app.hpp - base class for libcamera apps.
 */

#pragma once

#include <sys/mman.h>

#include <iostream>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <variant>
#include <thread>
#include <condition_variable>

#include "null_preview.hpp"
#include "egl_preview.hpp"
#include "drm_preview.hpp"
#include "frame_info.hpp"

// Crikey, X11/Xlib.h actually contains "#define Status int". Since when was that OK?
#undef Status

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>

namespace controls = libcamera::controls;
namespace properties = libcamera::properties;

struct CompletedRequest
{
	using BufferMap = libcamera::Request::BufferMap;
	using ControlList = libcamera::ControlList;
	CompletedRequest() {}
	CompletedRequest(unsigned int seq, BufferMap const &b, ControlList const &m)
		: sequence(seq), buffers(b), metadata(m) {}
	unsigned int sequence;
	BufferMap buffers;
	ControlList metadata;
	float framerate;
};

typedef std::function<void(CompletedRequest &)> PreviewDoneCallback;

template <class OPTIONS>
class LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;
	using CameraManager = libcamera::CameraManager;
	using Camera = libcamera::Camera;
	using CameraConfiguration = libcamera::CameraConfiguration;
	using FrameBufferAllocator = libcamera::FrameBufferAllocator;
	using StreamRole = libcamera::StreamRole;
	using StreamRoles = libcamera::StreamRoles;
	using PixelFormat = libcamera::PixelFormat;
	using StreamConfiguration = libcamera::StreamConfiguration;
	using BufferMap = Request::BufferMap;
	using Size = libcamera::Size;
	using Rectangle = libcamera::Rectangle;
	struct QuitPayload {};
	enum class MsgType
	{
		RequestComplete,
		Quit
	};
	typedef std::variant<CompletedRequest, QuitPayload> MsgPayload;
	struct Msg
	{
		Msg(MsgType const &t, MsgPayload const &p) : type(t), payload(p) {}
		MsgType type;
		MsgPayload payload;
	};

	// Program options are left as public; we use Boost to parse them.
	OPTIONS options;

	// Some flags that can be used to give hints to the camera configuration.
	static constexpr unsigned int FLAG_STILL_NONE   =  0;
	static constexpr unsigned int FLAG_STILL_BGR    =  1;  // supply BGR images, not YUV
	static constexpr unsigned int FLAG_STILL_RGB    =  2;  // supply RGB images, not YUV
	static constexpr unsigned int FLAG_STILL_RAW    =  4;  // request raw image stream
	static constexpr unsigned int FLAG_STILL_DOUBLE_BUFFER = 8;  // double-buffer stream
	static constexpr unsigned int FLAG_STILL_TRIPLE_BUFFER = 16; // triple-buffer stream
	static constexpr unsigned int FLAG_STILL_BUFFER_MASK   = 24; // mask for buffer flags

	static constexpr unsigned int FLAG_VIDEO_NONE   =  0;
	static constexpr unsigned int FLAG_VIDEO_RAW    =  1;  // request raw image stream

	LibcameraApp() : preview_thread_(&LibcameraApp::previewThread, this) {}
	virtual ~LibcameraApp()
	{
		{
			std::lock_guard<std::mutex> lock(preview_item_mutex_);
			preview_abort_ = true;
			preview_cond_var_.notify_one();
		}
		preview_thread_.join();
		if (options.verbose && !options.help)
			std::cout << "Closing Libcamera application" << "(frames displayed "
					  << preview_frames_displayed_ << ", dropped " << preview_frames_dropped_
					  << ")" << std::endl;
		StopCamera();
		Teardown();
		CloseCamera();
	}
	std::string const &CameraId() const { return camera_->id(); }
	void OpenCamera()
	{
		// Make a preview window.
		if (options.nopreview)
			preview_ = std::make_unique<NullPreview>(options);
		else
		{
			try
			{
				preview_ = std::make_unique<EglPreview>(options);
				if (options.verbose)
					std::cout << "Made X/EGL preview window" << std::endl;
			}
			catch(std::exception const &e)
			{
				try
				{
					preview_ = std::make_unique<DrmPreview>(options);
					if (options.verbose)
						std::cout << "Made DRM preview window" << std::endl;
				}
				catch (std::exception const &e)
				{
					std::cout << "Preview window unavailable" << std::endl;
					preview_ = std::make_unique<NullPreview>(options);
				}
			}
		}
		preview_->SetDoneCallback(std::bind(&LibcameraApp::previewDoneCallback, this, std::placeholders::_1));

		if (options.verbose)
			std::cout << "Opening camera..." << std::endl;

		camera_manager_ = std::make_unique<CameraManager>();
		int ret = camera_manager_->start();
		if (ret)
			throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));

		if (camera_manager_->cameras().size() == 0)
			throw std::runtime_error("no cameras available");

		std::string const &cam_id = camera_manager_->cameras()[0]->id();
		camera_ = camera_manager_->get(cam_id);
		if (!camera_)
			throw std::runtime_error("failed to find camera " + cam_id);

		if (camera_->acquire())
			throw std::runtime_error("failed to acquire camera " + cam_id);
		camera_acquired_ = true;

		if (options.verbose)
			std::cout << "Acquired camera " << cam_id << std::endl;
	}
	void CloseCamera()
	{
		preview_.reset();

		if (camera_acquired_)
			camera_->release();
		camera_acquired_ = false;

		camera_.reset();

		camera_manager_.reset();

		if (options.verbose && !options.help)
			std::cout << "Camera closed" << std::endl;
	}
	void ConfigureViewfinder()
	{
		if (options.verbose)
			std::cout << "Configuring viewfinder..." << std::endl;

		configuration_ = camera_->generateConfiguration({ StreamRole::Viewfinder });
		if (!configuration_)
			throw std::runtime_error("failed to generate viewfinder configuration");

		Size size(1280, 960);
		if (options.viewfinder_width && options.viewfinder_height)
			size = Size(options.viewfinder_width, options.viewfinder_height);
		else if (camera_->properties().contains(properties::PixelArrayActiveAreas))
		{
			// The idea here is that most sensors will have a 2x2 binned mode that
			// we can pick up. If it doesn't, well, you can always specify the size
			// you want exactly with the viewfinder_width/height options.
			size = camera_->properties().get(properties::PixelArrayActiveAreas)[0].size() / 2;
			// If width and height were given, we might be switching to capture
			// afterwards - so try to match the field of view.
			if (options.width && options.height)
				size = size.boundedToAspectRatio(Size(options.width, options.height));
			size.alignDownTo(2, 2); // YUV420 will want to be even
			if (options.verbose)
				std::cout << "Viewfinder size chosen is " << size.toString() << std::endl;
		}

		// Now we get to override any of the default settings from the options.
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
		configuration_->at(0).size = size;
		configuration_->transform = options.transform;

		configureDenoise(options.denoise == "auto" ? "cdn_off" : options.denoise);
		setupCapture();

		viewfinder_stream_ = configuration_->at(0).stream();

		if (options.verbose)
			std::cout << "Viewfinder setup complete" << std::endl;
	}
	void ConfigureStill(unsigned int flags = FLAG_STILL_NONE)
	{
		if (options.verbose)
			std::cout << "Configuring still capture..." << std::endl;

		// Will add a raw capture stream once that works properly.
		StreamRoles stream_roles;
		if (flags & FLAG_STILL_RAW)
			stream_roles = { StreamRole::StillCapture, StreamRole::Raw };
		else
			stream_roles = { StreamRole::StillCapture };
		configuration_ = camera_->generateConfiguration(stream_roles);
		if (!configuration_)
			throw std::runtime_error("failed to generate still capture configuration");

		// Now we get to override any of the default settings from the options.
		if (flags & FLAG_STILL_BGR)
			configuration_->at(0).pixelFormat = libcamera::formats::BGR888;
		else if (flags & FLAG_STILL_RGB)
			configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
		else
			configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
		if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_DOUBLE_BUFFER)
			configuration_->at(0).bufferCount = 2;
		else if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_TRIPLE_BUFFER)
			configuration_->at(0).bufferCount = 3;
		if (options.width)
			configuration_->at(0).size.width = options.width;
		if (options.height)
			configuration_->at(0).size.height = options.height;
		if ((flags & FLAG_STILL_RAW) && !options.rawfull)
		{
			configuration_->at(1).size.width = configuration_->at(0).size.width;
			configuration_->at(1).size.height = configuration_->at(0).size.height;
			configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;
		}
		configuration_->transform = options.transform;

		configureDenoise(options.denoise == "auto" ? "cdn_hq" : options.denoise);
		setupCapture();

		still_stream_ = configuration_->at(0).stream();
		raw_stream_ = configuration_->at(1).stream();

		if (options.verbose)
			std::cout << "Still capture setup complete" << std::endl;
	}
	void ConfigureVideo(unsigned int flags = FLAG_VIDEO_NONE)
	{
		if (options.verbose)
			std::cout << "Configuring video..." << std::endl;

		StreamRoles stream_roles;
		if (flags & FLAG_VIDEO_RAW)
			stream_roles = { StreamRole::VideoRecording, StreamRole::Raw };
		else
			stream_roles = { StreamRole::VideoRecording };
		configuration_ = camera_->generateConfiguration(stream_roles);
		if (!configuration_)
			throw std::runtime_error("failed to generate video configuration");

		// Now we get to override any of the default settings from the options.
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
		configuration_->at(0).bufferCount = 6; // 6 buffers is better than 4
		if (options.width)
			configuration_->at(0).size.width = options.width;
		if (options.height)
			configuration_->at(0).size.height = options.height;
		if (flags & FLAG_VIDEO_RAW)
		{
			if (!options.rawfull)
			{
				configuration_->at(1).size.width = configuration_->at(0).size.width;
				configuration_->at(1).size.height = configuration_->at(0).size.height;
			}
			configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;
		}
		configuration_->transform = options.transform;

		configureDenoise(options.denoise == "auto" ? "cdn_fast" : options.denoise);
		setupCapture();

		video_stream_ = configuration_->at(0).stream();
		raw_stream_ = configuration_->at(1).stream();

		if (options.verbose)
			std::cout << "Video setup complete" << std::endl;
	}
	void Teardown()
	{
		if (options.verbose && !options.help)
			std::cout << "Tearing down requests, buffers and configuration" << std::endl;

		for (auto &iter : mapped_buffers_)
		{
			assert(iter.first->planes().size() == iter.second.size());
			for (unsigned i = 0; i < iter.first->planes().size(); i++)
				munmap(iter.second[i], iter.first->planes()[i].length);
		}
		mapped_buffers_.clear();

		delete allocator_;
		allocator_ = nullptr;

		configuration_.reset();

		frame_buffers_.clear();

		viewfinder_stream_ = nullptr;
		still_stream_ = nullptr;
		raw_stream_ = nullptr;
		video_stream_ = nullptr;
	}
	void StartCamera()
	{
		// This makes all the Request objects that we shall need.
		makeRequests();

		// Build a list of initial controls that we must set in the camera before starting it.
		// We don't overwrite anything the application may have set before calling us.
		if (!controls_.contains(controls::ScalerCrop) &&
			options.roi_width != 0 && options.roi_height != 0)
		{
			Rectangle sensor_area = camera_->properties().get(properties::ScalerCropMaximum);
			int x = options.roi_x * sensor_area.width;
			int y = options.roi_y * sensor_area.height;
			int w = options.roi_width * sensor_area.width;
			int h = options.roi_height * sensor_area.height;
			Rectangle crop(x, y, w, h);
			crop.translateBy(sensor_area.topLeft());
			if (options.verbose)
				std::cout << "Using crop " << crop.toString() << std::endl;
			controls_.set(controls::ScalerCrop, crop);
		}

		// Framerate is a bit weird. If it was set programmatically, we go with that, but
		// otherwise it applies only to preview/video modes. For stills capture we set it
		// as long as possible so that we get whatever the exposure profile wants.
		if (!controls_.contains(controls::FrameDurationLimits))
		{
			if (still_stream_)
				controls_.set(controls::FrameDurationLimits, { INT64_C(100), INT64_C(1000000000) });
			else if (options.framerate > 0)
			{
				int64_t frame_time = 1000000 / options.framerate; // in us
				controls_.set(controls::FrameDurationLimits, { frame_time, frame_time });
			}
		}

		if (!controls_.contains(controls::ExposureTime) && options.shutter)
			controls_.set(controls::ExposureTime, options.shutter);
		if (!controls_.contains(controls::AnalogueGain) && options.gain)
			controls_.set(controls::AnalogueGain, options.gain);
		if (!controls_.contains(controls::AeMeteringMode))
			controls_.set(controls::AeMeteringMode, options.metering_index);
		if (!controls_.contains(controls::AeExposureMode))
			controls_.set(controls::AeExposureMode, options.exposure_index);
		if (!controls_.contains(controls::ExposureValue))
			controls_.set(controls::ExposureValue, options.ev);
		if (!controls_.contains(controls::AwbMode))
			controls_.set(controls::AwbMode, options.awb_index);
		if (!controls_.contains(controls::ColourGains) &&
			options.awb_gain_r && options.awb_gain_b)
			controls_.set(controls::ColourGains, { options.awb_gain_r, options.awb_gain_b });
		if (!controls_.contains(controls::Brightness))
			controls_.set(controls::Brightness, options.brightness);
		if (!controls_.contains(controls::Contrast))
			controls_.set(controls::Contrast, options.contrast);
		if (!controls_.contains(controls::Saturation))
			controls_.set(controls::Saturation, options.saturation);
		if (!controls_.contains(controls::Sharpness))
			controls_.set(controls::Sharpness, options.sharpness);

		if (camera_->start(&controls_))
			throw std::runtime_error("failed to start camera");
		controls_.clear();
		camera_started_ = true;
		last_timestamp_ = 0;

		camera_->requestCompleted.connect(this, &LibcameraApp::requestComplete);

		for (std::unique_ptr<Request> &request : requests_)
		{
			if (camera_->queueRequest(request.get()) < 0)
				throw std::runtime_error("Failed to queue request");
		}

		if (options.verbose)
			std::cout << "Camera started!" << std::endl;
	}
	void StopCamera()
	{
		{
			// We don't want QueueRequest to run asynchronously while we stop the camera.
			std::lock_guard<std::mutex> lock(camera_stop_mutex_);
			if (camera_started_)
			{
				if (camera_->stop())
					throw std::runtime_error("failed to stop camera");
				camera_started_ = false;
			}
		}

		if (camera_)
			camera_->requestCompleted.disconnect(this, &LibcameraApp::requestComplete);

		msg_queue_.Clear();

		if (preview_)
			preview_->Reset();

		while (!free_requests_.empty())
			free_requests_.pop();

		requests_.clear();

		controls_.clear(); // no need for mutex here

		if (options.verbose && !options.help)
			std::cout << "Camera stopped!" << std::endl;
	}
	Msg Wait() { return msg_queue_.Wait(); }
	static void StreamDimensions(Stream const *stream, int *w, int *h, int *stride)
	{
		StreamConfiguration const &cfg = stream->configuration();
		if (w)
			*w = cfg.size.width;
		if (h)
			*h = cfg.size.height;
		if (stride)
			*stride = cfg.stride;
	}
	Stream *ViewfinderStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const
	{
		StreamDimensions(viewfinder_stream_, w, h, stride);
		return viewfinder_stream_;
	}
	Stream *StillStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const
	{
		StreamDimensions(still_stream_, w, h, stride);
		return still_stream_;
	}
	Stream *RawStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const
	{
		StreamDimensions(raw_stream_, w, h, stride);
		return raw_stream_;
	}
	Stream *VideoStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const
	{
		StreamDimensions(video_stream_, w, h, stride);
		return video_stream_;
	}
	void QueueRequest(CompletedRequest const &completed_request)
	{
		// This function may run asynchronously so needs protection from the
		// camera stopping at the same time.
		std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);
		if (!camera_started_)
			return;

		Request *request = nullptr;
		{
			std::lock_guard<std::mutex> lock(free_requests_mutex_);
			if (!free_requests_.empty())
			{
				request = free_requests_.front();
				free_requests_.pop();
			}
		}
		if (!request)
		{
			std::cout << "WARNING: could not make request!" << std::endl;
			return;
		}

		for (auto const &p : completed_request.buffers)
		{
			if (request->addBuffer(p.first, p.second) < 0)
				throw std::runtime_error("failed to add buffer to request in QueueRequest");
		}

		{
			std::lock_guard<std::mutex> lock(control_mutex_);
			request->controls() = std::move(controls_);
		}

		if (camera_->queueRequest(request) < 0)
			throw std::runtime_error("failed to queue request");
	}
	void PostMessage(MsgType &t, MsgPayload &p) { msg_queue_.Post(Msg(t, p)); }
	std::vector<void *> Mmap(FrameBuffer *buffer) const
	{
		auto item = mapped_buffers_.find(buffer);
		if (item == mapped_buffers_.end())
			return std::vector<void *>();
		return item->second;
	}
	void SetPreviewDoneCallback(PreviewDoneCallback preview_done_callback)
	{
		preview_done_callback_ = preview_done_callback;
	}
	void ShowPreview(CompletedRequest &completed_request, Stream *stream)
	{
		PreviewItem item(std::move(completed_request), stream);
		{
			std::lock_guard<std::mutex> lock(preview_item_mutex_);
			if (!preview_item_.stream)
				preview_item_ = std::move(item);
			preview_cond_var_.notify_one();
		}
		// If we couldn't display this frame we must still return it through the callback
		if (item.stream)
		{
			preview_frames_dropped_++;
			if (preview_done_callback_)
				preview_done_callback_(item.completed_request);
		}
	}
	void SetControls(ControlList &controls)
	{
		std::lock_guard<std::mutex> lock(control_mutex_);
		controls_ = std::move(controls);
	}

private:
	template <typename T>
	class MessageQueue
	{
	public:
		template <typename U>
		void Post(U &&msg)
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_.push(std::forward<U>(msg));
			cond_.notify_one();
		}
		T Wait()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cond_.wait(lock, [this] { return !queue_.empty(); } );
			T msg = std::move(queue_.front());
			queue_.pop();
			return msg;
		}
		void Clear()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_ = {};
		}
	private:
		std::queue<T> queue_;
		std::mutex mutex_;
		std::condition_variable cond_;
	};
	struct PreviewItem
	{
		PreviewItem() : stream(nullptr) {}
		PreviewItem(CompletedRequest &&b, Stream *s) : completed_request(b), stream(s) {}
		PreviewItem &operator=(PreviewItem &&other)
		{
			completed_request = std::move(other.completed_request);
			stream = other.stream;
			other.stream = nullptr;
			return *this;
		}
		CompletedRequest completed_request;
		Stream *stream;
	};

	void setupCapture()
	{
		// First finish setting up the configuration.

		CameraConfiguration::Status validation = configuration_->validate();
		if (validation == CameraConfiguration::Invalid)
			throw std::runtime_error("failed to valid stream configurations");
		else if (validation == CameraConfiguration::Adjusted)
			std::cout << "Stream configuration adjusted" << std::endl;

		if (camera_->configure(configuration_.get()) < 0)
			throw std::runtime_error("failed to configure streams");
		if (options.verbose)
			std::cout << "Camera streams configured" << std::endl;

		// Next allocate all the buffers we need, mmap them and store them on a free list.

		allocator_ = new FrameBufferAllocator(camera_);
		for (StreamConfiguration &config : *configuration_)
		{
			Stream *stream = config.stream();

			if (allocator_->allocate(stream) < 0)
				throw std::runtime_error("failed to allocate capture buffers");

			for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream))
			{
				for (unsigned i = 0; i < buffer->planes().size(); i++)
				{
					const FrameBuffer::Plane &plane = buffer->planes()[i];
					void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,plane.fd.fd(), 0);
					mapped_buffers_[buffer.get()].push_back(memory);
				}
				frame_buffers_[stream].push(buffer.get());
			}
		}
		if (options.verbose)
			std::cout << "Buffers allocated and mapped" << std::endl;

		// The requests will be made when StartCamera() is called.
	}
	void makeRequests()
	{
		auto free_buffers(frame_buffers_);
		while (true)
		{
			for (StreamConfiguration &config : *configuration_)
			{
				Stream *stream = config.stream();
				if (stream == configuration_->at(0).stream())
				{
					if (free_buffers[stream].empty())
					{
						if (options.verbose)
							std::cout << "Requests created" << std::endl;
						return;
					}
					std::unique_ptr<Request> request = camera_->createRequest();
					if (!request)
						throw std::runtime_error("failed to make request");
					requests_.push_back(std::move(request));
				}
				else if (free_buffers[stream].empty())
					throw std::runtime_error("concurrent streams need matching numbers of buffers");

				FrameBuffer *buffer = free_buffers[stream].front();
				free_buffers[stream].pop();
				if (requests_.back()->addBuffer(stream, buffer) < 0)
					throw std::runtime_error("failed to add buffer to request");
			}
		}
	}
	void requestComplete(Request *request)
	{
		if (request->status() == Request::RequestCancelled)
			return;

		CompletedRequest payload(request->buffers().begin()->second->metadata().sequence,
								 request->buffers(), request->metadata());
		{
			request->reuse();
			std::lock_guard<std::mutex> lock(free_requests_mutex_);
			free_requests_.push(request);
		}

		// We calculate the instantaneous framerate in case anyone wants it.
		uint64_t timestamp = payload.buffers.begin()->second->metadata().timestamp;
		if (last_timestamp_ == 0 || last_timestamp_ == timestamp)
			payload.framerate = 0;
		else
			payload.framerate = 1e9 / (timestamp - last_timestamp_);
		last_timestamp_ = timestamp;

		msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(payload)));
	}
	void previewDoneCallback(int fd)
	{
		CompletedRequest completed_request;
		{
			std::lock_guard<std::mutex> lock(preview_mutex_);
			auto it = preview_completed_requests_.find(fd);
			if (it == preview_completed_requests_.end())
				throw std::runtime_error("previewDoneCallback: missing fd " + std::to_string(fd));
			completed_request = std::move(it->second);
			preview_completed_requests_.erase(it);
		}
		if (preview_done_callback_)
			preview_done_callback_(completed_request);
	}
	void previewThread()
	{
		while (true)
		{
			PreviewItem item;
			while (!item.stream)
			{
				std::unique_lock<std::mutex> lock(preview_item_mutex_);
				if (preview_abort_)
					return;
				else if (preview_item_.stream)
					item = std::move(preview_item_);
				else
					preview_cond_var_.wait(lock);
			}

			int w, h, stride;
			StreamDimensions(item.stream, &w, &h, &stride);
			FrameBuffer *buffer = item.completed_request.buffers[item.stream];

			// Fill the frame info with the ControlList items and ancillary bits.
			FrameInfo frame_info(item.completed_request.metadata);
			frame_info.fps = item.completed_request.framerate;
			frame_info.sequence = item.completed_request.sequence;

			int fd = buffer->planes()[0].fd.fd();
			size_t size = buffer->planes()[0].length;
			{
				std::lock_guard<std::mutex> lock(preview_mutex_);
				preview_completed_requests_[fd] = std::move(item.completed_request);
			}
			if (preview_->Quit())
			{
				if (options.verbose)
					std::cout << "Preview window has quit" << std::endl;
				msg_queue_.Post(Msg(MsgType::Quit, QuitPayload()));
			}
			preview_frames_displayed_++;
			preview_->Show(fd, size, w, h, stride);
			if (!options.info_text.empty())
			{
				std::string s = frame_info.ToString(options.info_text);
				preview_->SetInfoText(s);
			}
		}
	}
	void configureDenoise(const std::string &denoise_mode)
	{
		using namespace libcamera::controls::draft;

		static const std::map<std::string, NoiseReductionModeEnum> denoise_table = {
			{ "off", NoiseReductionModeOff },
			{ "cdn_off", NoiseReductionModeMinimal },
			{ "cdn_fast", NoiseReductionModeFast },
			{ "cdn_hq", NoiseReductionModeHighQuality }
		};
		NoiseReductionModeEnum denoise;

		auto const mode = denoise_table.find(denoise_mode);
		if (mode == denoise_table.end())
			throw std::runtime_error("Invalid denoise mode " + denoise_mode);
		denoise = mode->second;

		controls_.set(NoiseReductionMode, denoise);
	}

	std::unique_ptr<CameraManager> camera_manager_;
	std::shared_ptr<Camera> camera_;
	bool camera_acquired_ = false;
	std::unique_ptr<CameraConfiguration> configuration_;
	std::map<FrameBuffer *, std::vector<void *>> mapped_buffers_;
	Stream *viewfinder_stream_ = nullptr;
	Stream *still_stream_ = nullptr;
	Stream *raw_stream_ = nullptr;
	Stream *video_stream_ = nullptr;
	FrameBufferAllocator *allocator_ = nullptr;
	std::map<Stream *, std::queue<FrameBuffer *>> frame_buffers_;
	std::mutex free_requests_mutex_;
	std::queue<Request *> free_requests_;
	std::vector<std::unique_ptr<Request>> requests_;
	bool camera_started_ = false;
	std::mutex camera_stop_mutex_;
	MessageQueue<Msg> msg_queue_;
	// Related to the preview window.
	std::unique_ptr<Preview> preview_;
	PreviewDoneCallback preview_done_callback_;
	std::map<int, CompletedRequest> preview_completed_requests_;
	std::mutex preview_mutex_;
	std::mutex preview_item_mutex_;
	PreviewItem preview_item_;
	std::condition_variable preview_cond_var_;
	bool preview_abort_ = false;
	uint32_t preview_frames_displayed_ = 0;
	uint32_t preview_frames_dropped_ = 0;
	std::thread preview_thread_;
	// For setting camera controls.
	std::mutex control_mutex_;
	ControlList controls_;
	// Other:
	uint64_t last_timestamp_;
};
