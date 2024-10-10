/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings
 */

#include "py_main.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/libcamera.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
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
 * This is a holder class used only for the Camera class, for the sole purpose
 * of avoiding the compilation issue with Camera's private destructor.
 *
 * pybind11 requires a public destructor for classes held with shared_ptrs, even
 * in cases where the public destructor is not strictly needed. The current
 * understanding is that there are the following options to solve the problem:
 *
 * - Use pybind11 'smart_holder' branch. The downside is that 'smart_holder'
 *   is not the mainline branch, and not available in distributions.
 * - https://github.com/pybind/pybind11/pull/2067
 * - Make the Camera destructor public
 * - Something like the PyCameraSmartPtr here, which adds a layer, hiding the
 *   issue.
 */
template<typename T>
class PyCameraSmartPtr
{
public:
	using element_type = T;

	PyCameraSmartPtr()
	{
	}

	explicit PyCameraSmartPtr(T *)
	{
		throw std::runtime_error("invalid SmartPtr constructor call");
	}

	explicit PyCameraSmartPtr(std::shared_ptr<T> p)
		: ptr_(p)
	{
	}

	T *get() const { return ptr_.get(); }

	operator std::shared_ptr<T>() const { return ptr_; }

private:
	std::shared_ptr<T> ptr_;
};

PYBIND11_DECLARE_HOLDER_TYPE(T, PyCameraSmartPtr<T>)

/*
 * Note: global C++ destructors can be ran on this before the py module is
 * destructed.
 */
static std::weak_ptr<PyCameraManager> gCameraManager;

PYBIND11_MODULE(_libcamera, m)
{
	init_py_enums(m);
	init_py_controls_generated(m);
	init_py_geometry(m);
	init_py_properties_generated(m);
	init_py_color_space(m);
	init_py_transform(m);

	/* Forward declarations */

	/*
	 * We need to declare all the classes here so that Python docstrings
	 * can be generated correctly.
	 * https://pybind11.readthedocs.io/en/latest/advanced/misc.html#avoiding-c-types-in-docstrings
	 */

	auto pyCameraManager = py::class_<PyCameraManager, std::shared_ptr<PyCameraManager>>(m, "CameraManager");
	auto pyCamera = py::class_<Camera, PyCameraSmartPtr<Camera>>(m, "Camera");
	auto pySensorConfiguration = py::class_<SensorConfiguration>(m, "SensorConfiguration");
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

		.def_property_readonly_static("version", [](py::object /* self */) { return PyCameraManager::version(); })
		.def("get", &PyCameraManager::get, py::keep_alive<0, 1>())
		.def_property_readonly("cameras", &PyCameraManager::cameras)

		.def_property_readonly("event_fd", &PyCameraManager::eventFd)
		.def("get_ready_requests", &PyCameraManager::getReadyRequests);

	pyCamera
		.def_property_readonly("id", &Camera::id)
		.def("acquire", [](Camera &self) {
			int ret = self.acquire();
			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to acquire camera");
		})
		.def("release", [](Camera &self) {
			int ret = self.release();
			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to release camera");
		})
		.def("start", [](Camera &self,
		                 const std::unordered_map<const ControlId *, py::object> &controls) {
			/* \todo What happens if someone calls start() multiple times? */

			auto cm = gCameraManager.lock();
			ASSERT(cm);

			self.requestCompleted.connect(cm.get(), &PyCameraManager::handleRequestCompleted);

			ControlList controlList(self.controls());

			for (const auto &[id, obj] : controls) {
				auto val = pyToControlValue(obj, id->type());
				controlList.set(id->id(), val);
			}

			int ret = self.start(&controlList);
			if (ret) {
				self.requestCompleted.disconnect();
				throw std::system_error(-ret, std::generic_category(),
							"Failed to start camera");
			}
		}, py::arg("controls") = std::unordered_map<const ControlId *, py::object>())

		.def("stop", [](Camera &self) {
			int ret = self.stop();

			self.requestCompleted.disconnect();

			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to stop camera");
		})

		.def("__str__", [](Camera &self) {
			return "<libcamera.Camera '" + self.id() + "'>";
		})

		/* Keep the camera alive, as StreamConfiguration contains a Stream* */
		.def("generate_configuration", [](Camera &self, const std::vector<StreamRole> &roles) {
			return self.generateConfiguration(roles);
		}, py::keep_alive<0, 1>())

		.def("configure", [](Camera &self, CameraConfiguration *config) {
			int ret = self.configure(config);
			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to configure camera");
		})

		.def("create_request", [](Camera &self, uint64_t cookie) {
			std::unique_ptr<Request> req = self.createRequest(cookie);
			if (!req)
				throw std::system_error(ENOMEM, std::generic_category(),
							"Failed to create request");
			return req;
		}, py::arg("cookie") = 0)

		.def("queue_request", [](Camera &self, Request *req) {
			py::object py_req = py::cast(req);

			/*
			 * Increase the reference count, will be dropped in
			 * CameraManager.get_ready_requests().
			 */

			py_req.inc_ref();

			int ret = self.queueRequest(req);
			if (ret) {
				py_req.dec_ref();
				throw std::system_error(-ret, std::generic_category(),
							"Failed to queue request");
			}
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

	pySensorConfiguration
		.def(py::init<>())
		.def_readwrite("bit_depth", &SensorConfiguration::bitDepth)
		.def_readwrite("analog_crop", &SensorConfiguration::analogCrop)
		.def_property(
			"binning",
			[](SensorConfiguration &self) {
				return py::make_tuple(self.binning.binX, self.binning.binY);
			},
			[](SensorConfiguration &self, py::object value) {
				auto vec = value.cast<std::vector<unsigned int>>();
				if (vec.size() != 2)
					throw std::runtime_error("binning requires iterable of 2 values");
				self.binning.binX = vec[0];
				self.binning.binY = vec[1];
			})
		.def_property(
			"skipping",
			[](SensorConfiguration &self) {
				return py::make_tuple(self.skipping.xOddInc, self.skipping.xEvenInc,
						      self.skipping.yOddInc, self.skipping.yEvenInc);
			},
			[](SensorConfiguration &self, py::object value) {
				auto vec = value.cast<std::vector<unsigned int>>();
				if (vec.size() != 4)
					throw std::runtime_error("skipping requires iterable of 4 values");
				self.skipping.xOddInc = vec[0];
				self.skipping.xEvenInc = vec[1];
				self.skipping.yOddInc = vec[2];
				self.skipping.yEvenInc = vec[3];
			})
		.def_readwrite("output_size", &SensorConfiguration::outputSize)
		.def("is_valid", &SensorConfiguration::isValid);

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
		.def_readwrite("sensor_config", &CameraConfiguration::sensorConfig)
		.def_readwrite("orientation", &CameraConfiguration::orientation);

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
		.def(py::init<PyCameraSmartPtr<Camera>>(), py::keep_alive<1, 2>())
		.def("allocate", [](FrameBufferAllocator &self, Stream *stream) {
			int ret = self.allocate(stream);
			if (ret < 0)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to allocate buffers");
			return ret;
		})
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
		.def_property_readonly("vendor", &ControlId::vendor)
		.def_property_readonly("type", &ControlId::type)
		.def_property_readonly("isArray", &ControlId::isArray)
		.def_property_readonly("size", &ControlId::size)
		.def("__str__", [](const ControlId &self) { return self.name(); })
		.def("__repr__", [](const ControlId &self) {
			std::string sizeStr = "";
			if (self.isArray()) {
				sizeStr = "[";
				size_t size = self.size();
				if (size == std::numeric_limits<size_t>::max())
					sizeStr += "n";
				else
					sizeStr += std::to_string(size);
				sizeStr += "]";
			}
			return py::str("libcamera.ControlId({}, {}.{}{}, {})")
				.format(self.id(), self.vendor(), self.name(), sizeStr, self.type());
		})
		.def("enumerators", &ControlId::enumerators);

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
			int ret = self.addBuffer(stream, buffer);
			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to add buffer");
		}, py::keep_alive<1, 3>()) /* Request keeps Framebuffer alive */
		.def_property_readonly("status", &Request::status)
		.def_property_readonly("buffers", &Request::buffers)
		.def_property_readonly("cookie", &Request::cookie)
		.def_property_readonly("sequence", &Request::sequence)
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
