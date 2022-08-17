/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings
 */

#include "py_main.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/libcamera.h>

#include <pybind11/functional.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "py_camera_manager.h"
#include "py_helpers.h"

namespace py = pybind11;

using namespace libcamera;

namespace libcamera {

LOG_DEFINE_CATEGORY(Python)

}

/*
 * Note: global C++ destructors can be ran on this before the py module is
 * destructed.
 */
static std::weak_ptr<PyCameraManager> gCameraManager;

void init_py_enums(py::module &m);
void init_py_controls_generated(py::module &m);
void init_py_formats_generated(py::module &m);
void init_py_geometry(py::module &m);
void init_py_properties_generated(py::module &m);

PYBIND11_MODULE(_libcamera, m)
{
	init_py_enums(m);
	init_py_controls_generated(m);
	init_py_geometry(m);
	init_py_properties_generated(m);

	/* Forward declarations */

	/*
	 * We need to declare all the classes here so that Python docstrings
	 * can be generated correctly.
	 * https://pybind11.readthedocs.io/en/latest/advanced/misc.html#avoiding-c-types-in-docstrings
	 */

	auto pyCameraManager = py::class_<PyCameraManager>(m, "CameraManager");
	auto pyCamera = py::class_<Camera>(m, "Camera");
	auto pyCameraConfiguration = py::class_<CameraConfiguration>(m, "CameraConfiguration");
	auto pyCameraConfigurationStatus = py::enum_<CameraConfiguration::Status>(pyCameraConfiguration, "Status");
	auto pyStreamConfiguration = py::class_<StreamConfiguration>(m, "StreamConfiguration");
	auto pyStreamFormats = py::class_<StreamFormats>(m, "StreamFormats");
	auto pyFrameBufferAllocator = py::class_<FrameBufferAllocator>(m, "FrameBufferAllocator");
	auto pyFrameBuffer = py::class_<FrameBuffer>(m, "FrameBuffer");
	auto pyFrameBufferPlane = py::class_<FrameBuffer::Plane>(pyFrameBuffer, "Plane");
	auto pyStream = py::class_<Stream>(m, "Stream");
	auto pyControlId = py::class_<ControlId>(m, "ControlId");
	auto pyControlInfo = py::class_<ControlInfo>(m, "ControlInfo");
	auto pyRequest = py::class_<Request>(m, "Request");
	auto pyRequestStatus = py::enum_<Request::Status>(pyRequest, "Status");
	auto pyRequestReuse = py::enum_<Request::ReuseFlag>(pyRequest, "Reuse");
	auto pyFrameMetadata = py::class_<FrameMetadata>(m, "FrameMetadata");
	auto pyFrameMetadataStatus = py::enum_<FrameMetadata::Status>(pyFrameMetadata, "Status");
	auto pyFrameMetadataPlane = py::class_<FrameMetadata::Plane>(pyFrameMetadata, "Plane");
	auto pyTransform = py::class_<Transform>(m, "Transform");
	auto pyColorSpace = py::class_<ColorSpace>(m, "ColorSpace");
	auto pyColorSpacePrimaries = py::enum_<ColorSpace::Primaries>(pyColorSpace, "Primaries");
	auto pyColorSpaceTransferFunction = py::enum_<ColorSpace::TransferFunction>(pyColorSpace, "TransferFunction");
	auto pyColorSpaceYcbcrEncoding = py::enum_<ColorSpace::YcbcrEncoding>(pyColorSpace, "YcbcrEncoding");
	auto pyColorSpaceRange = py::enum_<ColorSpace::Range>(pyColorSpace, "Range");
	auto pyPixelFormat = py::class_<PixelFormat>(m, "PixelFormat");

	init_py_formats_generated(m);

	/* Global functions */
	m.def("log_set_level", &logSetLevel);

	/* Classes */
	pyCameraManager
		.def_static("singleton", []() {
			std::shared_ptr<PyCameraManager> cm = gCameraManager.lock();

			if (!cm) {
				cm = std::make_shared<PyCameraManager>();
				gCameraManager = cm;
			}

			return cm;
		})

		.def_property_readonly("version", &PyCameraManager::version)
		.def("get", &PyCameraManager::get, py::keep_alive<0, 1>())
		.def_property_readonly("cameras", &PyCameraManager::cameras)

		.def_property_readonly("event_fd", &PyCameraManager::eventFd)
		.def("get_ready_requests", &PyCameraManager::getReadyRequests);

	pyCamera
		.def_property_readonly("id", &Camera::id)
		.def("acquire", &Camera::acquire)
		.def("release", &Camera::release)
		.def("start", [](Camera &self,
		                 const std::unordered_map<const ControlId *, py::object> &controls) {
			/* \todo What happens if someone calls start() multiple times? */

			auto cm = gCameraManager.lock();
			ASSERT(cm);

			self.requestCompleted.connect(cm.get(), &PyCameraManager::handleRequestCompleted);

			ControlList controlList(self.controls());

			for (const auto& [id, obj]: controls) {
				auto val = pyToControlValue(obj, id->type());
				controlList.set(id->id(), val);
			}

			int ret = self.start(&controlList);
			if (ret) {
				self.requestCompleted.disconnect();
				return ret;
			}

			return 0;
		}, py::arg("controls") = std::unordered_map<const ControlId *, py::object>())

		.def("stop", [](Camera &self) {
			int ret = self.stop();
			if (ret)
				return ret;

			self.requestCompleted.disconnect();

			return 0;
		})

		.def("__str__", [](Camera &self) {
			return "<libcamera.Camera '" + self.id() + "'>";
		})

		/* Keep the camera alive, as StreamConfiguration contains a Stream* */
		.def("generate_configuration", &Camera::generateConfiguration, py::keep_alive<0, 1>())
		.def("configure", &Camera::configure)

		.def("create_request", &Camera::createRequest, py::arg("cookie") = 0)

		.def("queue_request", [](Camera &self, Request *req) {
			py::object py_req = py::cast(req);

			/*
			 * Increase the reference count, will be dropped in
			 * CameraManager.get_ready_requests().
			 */

			py_req.inc_ref();

			int ret = self.queueRequest(req);
			if (ret)
				py_req.dec_ref();

			return ret;
		})

		.def_property_readonly("streams", [](Camera &self) {
			py::set set;
			for (auto &s : self.streams()) {
				py::object py_self = py::cast(self);
				py::object py_s = py::cast(s);
				py::detail::keep_alive_impl(py_s, py_self);
				set.add(py_s);
			}
			return set;
		})

		.def_property_readonly("controls", [](Camera &self) {
			/* Convert ControlInfoMap to std container */

			std::unordered_map<const ControlId *, ControlInfo> ret;

			for (const auto &[k, cv] : self.controls())
				ret[k] = cv;

			return ret;
		})

		.def_property_readonly("properties", [](Camera &self) {
			/* Convert ControlList to std container */

			std::unordered_map<const ControlId *, py::object> ret;

			for (const auto &[k, cv] : self.properties()) {
				const ControlId *id = properties::properties.at(k);
				py::object ob = controlValueToPy(cv);
				ret[id] = ob;
			}

			return ret;
		});

	pyCameraConfiguration
		.def("__iter__", [](CameraConfiguration &self) {
			return py::make_iterator<py::return_value_policy::reference_internal>(self);
		}, py::keep_alive<0, 1>())
		.def("__len__", [](CameraConfiguration &self) {
			return self.size();
		})
		.def("validate", &CameraConfiguration::validate)
		.def("at", py::overload_cast<unsigned int>(&CameraConfiguration::at),
		     py::return_value_policy::reference_internal)
		.def_property_readonly("size", &CameraConfiguration::size)
		.def_property_readonly("empty", &CameraConfiguration::empty)
		.def_readwrite("transform", &CameraConfiguration::transform);

	pyCameraConfigurationStatus
		.value("Valid", CameraConfiguration::Valid)
		.value("Adjusted", CameraConfiguration::Adjusted)
		.value("Invalid", CameraConfiguration::Invalid);

	pyStreamConfiguration
		.def("__str__", &StreamConfiguration::toString)
		.def_property_readonly("stream", &StreamConfiguration::stream,
				       py::return_value_policy::reference_internal)
		.def_readwrite("size", &StreamConfiguration::size)
		.def_readwrite("pixel_format", &StreamConfiguration::pixelFormat)
		.def_readwrite("stride", &StreamConfiguration::stride)
		.def_readwrite("frame_size", &StreamConfiguration::frameSize)
		.def_readwrite("buffer_count", &StreamConfiguration::bufferCount)
		.def_property_readonly("formats", &StreamConfiguration::formats,
				       py::return_value_policy::reference_internal)
		.def_readwrite("color_space", &StreamConfiguration::colorSpace);

	pyStreamFormats
		.def_property_readonly("pixel_formats", &StreamFormats::pixelformats)
		.def("sizes", &StreamFormats::sizes)
		.def("range", &StreamFormats::range);

	pyFrameBufferAllocator
		.def(py::init<std::shared_ptr<Camera>>(), py::keep_alive<1, 2>())
		.def("allocate", &FrameBufferAllocator::allocate)
		.def_property_readonly("allocated", &FrameBufferAllocator::allocated)
		/* Create a list of FrameBuffers, where each FrameBuffer has a keep-alive to FrameBufferAllocator */
		.def("buffers", [](FrameBufferAllocator &self, Stream *stream) {
			py::object py_self = py::cast(self);
			py::list l;
			for (auto &ub : self.buffers(stream)) {
				py::object py_buf = py::cast(ub.get(), py::return_value_policy::reference_internal, py_self);
				l.append(py_buf);
			}
			return l;
		});

	pyFrameBuffer
		.def(py::init<std::vector<FrameBuffer::Plane>, unsigned int>(),
		     py::arg("planes"), py::arg("cookie") = 0)
		.def_property_readonly("metadata", &FrameBuffer::metadata, py::return_value_policy::reference_internal)
		.def_property_readonly("planes", &FrameBuffer::planes)
		.def_property("cookie", &FrameBuffer::cookie, &FrameBuffer::setCookie);

	pyFrameBufferPlane
		.def(py::init())
		.def(py::init([](int fd, unsigned int offset, unsigned int length) {
			auto p = FrameBuffer::Plane();
			p.fd = SharedFD(fd);
			p.offset = offset;
			p.length = length;
			return p;
		}), py::arg("fd"), py::arg("offset"), py::arg("length"))
		.def_property("fd",
			[](const FrameBuffer::Plane &self) {
				return self.fd.get();
			},
			[](FrameBuffer::Plane &self, int fd) {
				self.fd = SharedFD(fd);
			})
		.def_readwrite("offset", &FrameBuffer::Plane::offset)
		.def_readwrite("length", &FrameBuffer::Plane::length);

	pyStream
		.def_property_readonly("configuration", &Stream::configuration);

	pyControlId
		.def_property_readonly("id", &ControlId::id)
		.def_property_readonly("name", &ControlId::name)
		.def_property_readonly("type", &ControlId::type)
		.def("__str__", [](const ControlId &self) { return self.name(); })
		.def("__repr__", [](const ControlId &self) {
			return py::str("libcamera.ControlId({}, {}, {})")
				.format(self.id(), self.name(), self.type());
		});

	pyControlInfo
		.def_property_readonly("min", [](const ControlInfo &self) {
			return controlValueToPy(self.min());
		})
		.def_property_readonly("max", [](const ControlInfo &self) {
			return controlValueToPy(self.max());
		})
		.def_property_readonly("default", [](const ControlInfo &self) {
			return controlValueToPy(self.def());
		})
		.def_property_readonly("values", [](const ControlInfo &self) {
			py::list l;
			for (const auto &v : self.values())
				l.append(controlValueToPy(v));
			return l;
		})
		.def("__str__", &ControlInfo::toString)
		.def("__repr__", [](const ControlInfo &self) {
			return py::str("libcamera.ControlInfo({})")
				.format(self.toString());
		});

	pyRequest
		/* \todo Fence is not supported, so we cannot expose addBuffer() directly */
		.def("add_buffer", [](Request &self, const Stream *stream, FrameBuffer *buffer) {
			return self.addBuffer(stream, buffer);
		}, py::keep_alive<1, 3>()) /* Request keeps Framebuffer alive */
		.def_property_readonly("status", &Request::status)
		.def_property_readonly("buffers", &Request::buffers)
		.def_property_readonly("cookie", &Request::cookie)
		.def_property_readonly("has_pending_buffers", &Request::hasPendingBuffers)
		.def("set_control", [](Request &self, const ControlId &id, py::object value) {
			self.controls().set(id.id(), pyToControlValue(value, id.type()));
		})
		.def_property_readonly("metadata", [](Request &self) {
			/* Convert ControlList to std container */

			std::unordered_map<const ControlId *, py::object> ret;

			for (const auto &[key, cv] : self.metadata()) {
				const ControlId *id = controls::controls.at(key);
				py::object ob = controlValueToPy(cv);
				ret[id] = ob;
			}

			return ret;
		})
		/*
		 * \todo As we add a keep_alive to the fb in addBuffers(), we
		 * can only allow reuse with ReuseBuffers.
		 */
		.def("reuse", [](Request &self) { self.reuse(Request::ReuseFlag::ReuseBuffers); })
		.def("__str__", &Request::toString);

	pyRequestStatus
		.value("Pending", Request::RequestPending)
		.value("Complete", Request::RequestComplete)
		.value("Cancelled", Request::RequestCancelled);

	pyRequestReuse
		.value("Default", Request::ReuseFlag::Default)
		.value("ReuseBuffers", Request::ReuseFlag::ReuseBuffers);

	pyFrameMetadata
		.def_readonly("status", &FrameMetadata::status)
		.def_readonly("sequence", &FrameMetadata::sequence)
		.def_readonly("timestamp", &FrameMetadata::timestamp)
		.def_property_readonly("planes", [](const FrameMetadata &self) {
			/* Convert from Span<> to std::vector<> */
			/* Note: this creates a copy */
			std::vector<FrameMetadata::Plane> v(self.planes().begin(), self.planes().end());
			return v;
		});

	pyFrameMetadataStatus
		.value("Success", FrameMetadata::FrameSuccess)
		.value("Error", FrameMetadata::FrameError)
		.value("Cancelled", FrameMetadata::FrameCancelled);

	pyFrameMetadataPlane
		.def_readwrite("bytes_used", &FrameMetadata::Plane::bytesused);

	pyTransform
		.def(py::init([](int rotation, bool hflip, bool vflip, bool transpose) {
			bool ok;

			Transform t = transformFromRotation(rotation, &ok);
			if (!ok)
				throw std::invalid_argument("Invalid rotation");

			if (hflip)
				t ^= Transform::HFlip;
			if (vflip)
				t ^= Transform::VFlip;
			if (transpose)
				t ^= Transform::Transpose;
			return t;
		}), py::arg("rotation") = 0, py::arg("hflip") = false,
		    py::arg("vflip") = false, py::arg("transpose") = false)
		.def(py::init([](Transform &other) { return other; }))
		.def("__str__", [](Transform &self) {
			return "<libcamera.Transform '" + std::string(transformToString(self)) + "'>";
		})
		.def_property("hflip",
			      [](Transform &self) {
				      return !!(self & Transform::HFlip);
			      },
			      [](Transform &self, bool hflip) {
				      if (hflip)
					      self |= Transform::HFlip;
				      else
					      self &= ~Transform::HFlip;
			      })
		.def_property("vflip",
			      [](Transform &self) {
				      return !!(self & Transform::VFlip);
			      },
			      [](Transform &self, bool vflip) {
				      if (vflip)
					      self |= Transform::VFlip;
				      else
					      self &= ~Transform::VFlip;
			      })
		.def_property("transpose",
			      [](Transform &self) {
				      return !!(self & Transform::Transpose);
			      },
			      [](Transform &self, bool transpose) {
				      if (transpose)
					      self |= Transform::Transpose;
				      else
					      self &= ~Transform::Transpose;
			      })
		.def("inverse", [](Transform &self) { return -self; })
		.def("invert", [](Transform &self) {
			self = -self;
		})
		.def("compose", [](Transform &self, Transform &other) {
			self = self * other;
		});

	pyColorSpace
		.def(py::init([](ColorSpace::Primaries primaries,
				 ColorSpace::TransferFunction transferFunction,
				 ColorSpace::YcbcrEncoding ycbcrEncoding,
				 ColorSpace::Range range) {
			return ColorSpace(primaries, transferFunction, ycbcrEncoding, range);
		}), py::arg("primaries"), py::arg("transferFunction"),
		    py::arg("ycbcrEncoding"), py::arg("range"))
		.def(py::init([](ColorSpace &other) { return other; }))
		.def("__str__", [](ColorSpace &self) {
			return "<libcamera.ColorSpace '" + self.toString() + "'>";
		})
		.def_readwrite("primaries", &ColorSpace::primaries)
		.def_readwrite("transferFunction", &ColorSpace::transferFunction)
		.def_readwrite("ycbcrEncoding", &ColorSpace::ycbcrEncoding)
		.def_readwrite("range", &ColorSpace::range)
		.def_static("Raw", []() { return ColorSpace::Raw; })
		.def_static("Srgb", []() { return ColorSpace::Srgb; })
		.def_static("Sycc", []() { return ColorSpace::Sycc; })
		.def_static("Smpte170m", []() { return ColorSpace::Smpte170m; })
		.def_static("Rec709", []() { return ColorSpace::Rec709; })
		.def_static("Rec2020", []() { return ColorSpace::Rec2020; });

	pyColorSpacePrimaries
		.value("Raw", ColorSpace::Primaries::Raw)
		.value("Smpte170m", ColorSpace::Primaries::Smpte170m)
		.value("Rec709", ColorSpace::Primaries::Rec709)
		.value("Rec2020", ColorSpace::Primaries::Rec2020);

	pyColorSpaceTransferFunction
		.value("Linear", ColorSpace::TransferFunction::Linear)
		.value("Srgb", ColorSpace::TransferFunction::Srgb)
		.value("Rec709", ColorSpace::TransferFunction::Rec709);

	pyColorSpaceYcbcrEncoding
		.value("Null", ColorSpace::YcbcrEncoding::None)
		.value("Rec601", ColorSpace::YcbcrEncoding::Rec601)
		.value("Rec709", ColorSpace::YcbcrEncoding::Rec709)
		.value("Rec2020", ColorSpace::YcbcrEncoding::Rec2020);

	pyColorSpaceRange
		.value("Full", ColorSpace::Range::Full)
		.value("Limited", ColorSpace::Range::Limited);

	pyPixelFormat
		.def(py::init<>())
		.def(py::init<uint32_t, uint64_t>())
		.def(py::init<>([](const std::string &str) {
			return PixelFormat::fromString(str);
		}))
		.def_property_readonly("fourcc", &PixelFormat::fourcc)
		.def_property_readonly("modifier", &PixelFormat::modifier)
		.def(py::self == py::self)
		.def("__str__", &PixelFormat::toString)
		.def("__repr__", [](const PixelFormat &self) {
			return "libcamera.PixelFormat('" + self.toString() + "')";
		});
}
