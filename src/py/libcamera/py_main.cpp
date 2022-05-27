/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings
 */

/*
 * \todo Add bindings for the ControlInfo class
 */

#include <mutex>
#include <stdexcept>
#include <sys/eventfd.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include <libcamera/libcamera.h>

#include <pybind11/functional.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

using namespace libcamera;

template<typename T>
static py::object valueOrTuple(const ControlValue &cv)
{
	if (cv.isArray()) {
		const T *v = reinterpret_cast<const T *>(cv.data().data());
		auto t = py::tuple(cv.numElements());

		for (size_t i = 0; i < cv.numElements(); ++i)
			t[i] = v[i];

		return std::move(t);
	}

	return py::cast(cv.get<T>());
}

static py::object controlValueToPy(const ControlValue &cv)
{
	switch (cv.type()) {
	case ControlTypeBool:
		return valueOrTuple<bool>(cv);
	case ControlTypeByte:
		return valueOrTuple<uint8_t>(cv);
	case ControlTypeInteger32:
		return valueOrTuple<int32_t>(cv);
	case ControlTypeInteger64:
		return valueOrTuple<int64_t>(cv);
	case ControlTypeFloat:
		return valueOrTuple<float>(cv);
	case ControlTypeString:
		return py::cast(cv.get<std::string>());
	case ControlTypeRectangle: {
		const Rectangle *v = reinterpret_cast<const Rectangle *>(cv.data().data());
		return py::cast(v);
	}
	case ControlTypeSize: {
		const Size *v = reinterpret_cast<const Size *>(cv.data().data());
		return py::cast(v);
	}
	case ControlTypeNone:
	default:
		throw std::runtime_error("Unsupported ControlValue type");
	}
}

template<typename T>
static ControlValue controlValueMaybeArray(const py::object &ob)
{
	if (py::isinstance<py::list>(ob) || py::isinstance<py::tuple>(ob)) {
		std::vector<T> vec = ob.cast<std::vector<T>>();
		return ControlValue(Span<const T>(vec));
	}

	return ControlValue(ob.cast<T>());
}

static ControlValue pyToControlValue(const py::object &ob, ControlType type)
{
	switch (type) {
	case ControlTypeBool:
		return ControlValue(ob.cast<bool>());
	case ControlTypeByte:
		return controlValueMaybeArray<uint8_t>(ob);
	case ControlTypeInteger32:
		return controlValueMaybeArray<int32_t>(ob);
	case ControlTypeInteger64:
		return controlValueMaybeArray<int64_t>(ob);
	case ControlTypeFloat:
		return controlValueMaybeArray<float>(ob);
	case ControlTypeString:
		return ControlValue(ob.cast<std::string>());
	case ControlTypeRectangle:
		return ControlValue(ob.cast<Rectangle>());
	case ControlTypeSize:
		return ControlValue(ob.cast<Size>());
	case ControlTypeNone:
	default:
		throw std::runtime_error("Control type not implemented");
	}
}

static std::weak_ptr<CameraManager> gCameraManager;
static int gEventfd;
static std::mutex gReqlistMutex;
static std::vector<Request *> gReqList;

static void handleRequestCompleted(Request *req)
{
	{
		std::lock_guard guard(gReqlistMutex);
		gReqList.push_back(req);
	}

	uint64_t v = 1;
	size_t s = write(gEventfd, &v, 8);
	/*
	 * We should never fail, and have no simple means to manage the error,
	 * so let's use LOG(Fatal).
	 */
	if (s != 8)
		LOG(Fatal) << "Unable to write to eventfd";
}

void init_py_enums(py::module &m);
void init_py_control_enums_generated(py::module &m);
void init_py_formats_generated(py::module &m);
void init_py_geometry(py::module &m);

PYBIND11_MODULE(_libcamera, m)
{
	init_py_enums(m);
	init_py_control_enums_generated(m);
	init_py_geometry(m);

	/* Forward declarations */

	/*
	 * We need to declare all the classes here so that Python docstrings
	 * can be generated correctly.
	 * https://pybind11.readthedocs.io/en/latest/advanced/misc.html#avoiding-c-types-in-docstrings
	 */

	auto pyCameraManager = py::class_<CameraManager>(m, "CameraManager");
	auto pyCamera = py::class_<Camera>(m, "Camera");
	auto pyCameraConfiguration = py::class_<CameraConfiguration>(m, "CameraConfiguration");
	auto pyCameraConfigurationStatus = py::enum_<CameraConfiguration::Status>(pyCameraConfiguration, "Status");
	auto pyStreamConfiguration = py::class_<StreamConfiguration>(m, "StreamConfiguration");
	auto pyStreamFormats = py::class_<StreamFormats>(m, "StreamFormats");
	auto pyFrameBufferAllocator = py::class_<FrameBufferAllocator>(m, "FrameBufferAllocator");
	auto pyFrameBuffer = py::class_<FrameBuffer>(m, "FrameBuffer");
	auto pyStream = py::class_<Stream>(m, "Stream");
	auto pyControlId = py::class_<ControlId>(m, "ControlId");
	auto pyRequest = py::class_<Request>(m, "Request");
	auto pyRequestStatus = py::enum_<Request::Status>(pyRequest, "Status");
	auto pyRequestReuse = py::enum_<Request::ReuseFlag>(pyRequest, "Reuse");
	auto pyFrameMetadata = py::class_<FrameMetadata>(m, "FrameMetadata");
	auto pyFrameMetadataStatus = py::enum_<FrameMetadata::Status>(pyFrameMetadata, "Status");
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
			std::shared_ptr<CameraManager> cm = gCameraManager.lock();
			if (cm)
				return cm;

			int fd = eventfd(0, 0);
			if (fd == -1)
				throw std::system_error(errno, std::generic_category(),
							"Failed to create eventfd");

			cm = std::shared_ptr<CameraManager>(new CameraManager, [](auto p) {
				close(gEventfd);
				gEventfd = -1;
				delete p;
			});

			gEventfd = fd;
			gCameraManager = cm;

			int ret = cm->start();
			if (ret)
				throw std::system_error(-ret, std::generic_category(),
							"Failed to start CameraManager");

			return cm;
		})

		.def_property_readonly("version", &CameraManager::version)

		.def_property_readonly("efd", [](CameraManager &) {
			return gEventfd;
		})

		.def("get_ready_requests", [](CameraManager &) {
			std::vector<Request *> v;

			{
				std::lock_guard guard(gReqlistMutex);
				swap(v, gReqList);
			}

			std::vector<py::object> ret;

			for (Request *req : v) {
				py::object o = py::cast(req);
				/* Decrease the ref increased in Camera.queue_request() */
				o.dec_ref();
				ret.push_back(o);
			}

			return ret;
		})

		.def("get", py::overload_cast<const std::string &>(&CameraManager::get), py::keep_alive<0, 1>())

		/* Create a list of Cameras, where each camera has a keep-alive to CameraManager */
		.def_property_readonly("cameras", [](CameraManager &self) {
			py::list l;

			for (auto &c : self.cameras()) {
				py::object py_cm = py::cast(self);
				py::object py_cam = py::cast(c);
				py::detail::keep_alive_impl(py_cam, py_cm);
				l.append(py_cam);
			}

			return l;
		});

	pyCamera
		.def_property_readonly("id", &Camera::id)
		.def("acquire", &Camera::acquire)
		.def("release", &Camera::release)
		.def("start", [](Camera &self, py::dict controls) {
			/* \todo What happens if someone calls start() multiple times? */

			self.requestCompleted.connect(handleRequestCompleted);

			const ControlInfoMap &controlMap = self.controls();
			ControlList controlList(controlMap);
			for (const auto& [hkey, hval]: controls) {
				auto key = hkey.cast<std::string>();

				auto it = std::find_if(controlMap.begin(), controlMap.end(),
						       [&key](const auto &kvp) {
								return kvp.first->name() == key;
						       });

				if (it == controlMap.end())
					throw std::runtime_error("Control " + key + " not found");

				const auto &id = it->first;
				auto obj = py::cast<py::object>(hval);

				controlList.set(id->id(), pyToControlValue(obj, id->type()));
			}

			int ret = self.start(&controlList);
			if (ret) {
				self.requestCompleted.disconnect(handleRequestCompleted);
				return ret;
			}

			return 0;
		}, py::arg("controls") = py::dict())

		.def("stop", [](Camera &self) {
			int ret = self.stop();
			if (ret)
				return ret;

			self.requestCompleted.disconnect(handleRequestCompleted);

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

		.def("find_control", [](Camera &self, const std::string &name) {
			const auto &controls = self.controls();

			auto it = std::find_if(controls.begin(), controls.end(),
					       [&name](const auto &kvp) {
							return kvp.first->name() == name;
					       });

			if (it == controls.end())
				throw std::runtime_error("Control '" + name + "' not found");

			return it->first;
		}, py::return_value_policy::reference_internal)

		.def_property_readonly("controls", [](Camera &self) {
			py::dict ret;

			for (const auto &[id, ci] : self.controls()) {
				ret[id->name().c_str()] = std::make_tuple<py::object>(controlValueToPy(ci.min()),
										      controlValueToPy(ci.max()),
										      controlValueToPy(ci.def()));
			}

			return ret;
		})

		.def_property_readonly("properties", [](Camera &self) {
			py::dict ret;

			for (const auto &[key, cv] : self.properties()) {
				const ControlId *id = properties::properties.at(key);
				py::object ob = controlValueToPy(cv);

				ret[id->name().c_str()] = ob;
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
		/* \todo implement FrameBuffer::Plane properly */
		.def(py::init([](std::vector<std::tuple<int, unsigned int>> planes, unsigned int cookie) {
			std::vector<FrameBuffer::Plane> v;
			for (const auto &t : planes)
				v.push_back({ SharedFD(std::get<0>(t)), FrameBuffer::Plane::kInvalidOffset, std::get<1>(t) });
			return new FrameBuffer(v, cookie);
		}))
		.def_property_readonly("metadata", &FrameBuffer::metadata, py::return_value_policy::reference_internal)
		.def_property_readonly("num_planes", [](const FrameBuffer &self) {
			return self.planes().size();
		})
		.def("length", [](FrameBuffer &self, uint32_t idx) {
			const FrameBuffer::Plane &plane = self.planes()[idx];
			return plane.length;
		})
		.def("fd", [](FrameBuffer &self, uint32_t idx) {
			const FrameBuffer::Plane &plane = self.planes()[idx];
			return plane.fd.get();
		})
		.def("offset", [](FrameBuffer &self, uint32_t idx) {
			const FrameBuffer::Plane &plane = self.planes()[idx];
			return plane.offset;
		})
		.def_property("cookie", &FrameBuffer::cookie, &FrameBuffer::setCookie);

	pyStream
		.def_property_readonly("configuration", &Stream::configuration);

	pyControlId
		.def_property_readonly("id", &ControlId::id)
		.def_property_readonly("name", &ControlId::name)
		.def_property_readonly("type", &ControlId::type);

	pyRequest
		/* \todo Fence is not supported, so we cannot expose addBuffer() directly */
		.def("add_buffer", [](Request &self, const Stream *stream, FrameBuffer *buffer) {
			return self.addBuffer(stream, buffer);
		}, py::keep_alive<1, 3>()) /* Request keeps Framebuffer alive */
		.def_property_readonly("status", &Request::status)
		.def_property_readonly("buffers", &Request::buffers)
		.def_property_readonly("cookie", &Request::cookie)
		.def_property_readonly("has_pending_buffers", &Request::hasPendingBuffers)
		.def("set_control", [](Request &self, ControlId &id, py::object value) {
			self.controls().set(id.id(), pyToControlValue(value, id.type()));
		})
		.def_property_readonly("metadata", [](Request &self) {
			py::dict ret;

			for (const auto &[key, cv] : self.metadata()) {
				const ControlId *id = controls::controls.at(key);
				py::object ob = controlValueToPy(cv);

				ret[id->name().c_str()] = ob;
			}

			return ret;
		})
		/*
		 * \todo As we add a keep_alive to the fb in addBuffers(), we
		 * can only allow reuse with ReuseBuffers.
		 */
		.def("reuse", [](Request &self) { self.reuse(Request::ReuseFlag::ReuseBuffers); });

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
		/* \todo Implement FrameMetadata::Plane properly */
		.def_property_readonly("bytesused", [](FrameMetadata &self) {
			std::vector<unsigned int> v;
			v.resize(self.planes().size());
			transform(self.planes().begin(), self.planes().end(), v.begin(), [](const auto &p) { return p.bytesused; });
			return v;
		});

	pyFrameMetadataStatus
		.value("Success", FrameMetadata::FrameSuccess)
		.value("Error", FrameMetadata::FrameError)
		.value("Cancelled", FrameMetadata::FrameCancelled);

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
		.def_static("Jpeg", []() { return ColorSpace::Jpeg; })
		.def_static("Srgb", []() { return ColorSpace::Srgb; })
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
