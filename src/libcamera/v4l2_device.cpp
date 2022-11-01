/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.cpp - Common base for V4L2 video devices and subdevices
 */

#include "libcamera/internal/v4l2_device.h"

#include <fcntl.h>
#include <iomanip>
#include <limits.h>
#include <map>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

#include <linux/v4l2-mediabus.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/sysfs.h"

/**
 * \file v4l2_device.h
 * \brief Common base for V4L2 devices and subdevices
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(V4L2)

/**
 * \class V4L2Device
 * \brief Base class for V4L2VideoDevice and V4L2Subdevice
 *
 * The V4L2Device class groups together the functions and fields common to
 * both the V4L2VideoDevice and V4L2Subdevice classes, and provides a base
 * class with functions to open and close the device node associated with the
 * device and to perform IOCTL system calls on it.
 *
 * The V4L2Device class cannot be instantiated directly, as its constructor
 * is protected. Users should instead create instances of one the derived
 * classes to model either a V4L2 video device or a V4L2 subdevice.
 */

/**
 * \brief Construct a V4L2Device
 * \param[in] deviceNode The device node filesystem path
 *
 * Initialize the file descriptor to -1 and store the \a deviceNode to be used
 * at open() time, and the \a logTag to prefix log messages with.
 */
V4L2Device::V4L2Device(const std::string &deviceNode)
	: deviceNode_(deviceNode), fdEventNotifier_(nullptr),
	  frameStartEnabled_(false)
{
}

/**
 * \brief Destroy a V4L2Device
 */
V4L2Device::~V4L2Device()
{
}

/**
 * \brief Open a V4L2 device node
 * \param[in] flags Access mode flags
 *
 * Open the device node path with the provided access mode \a flags and
 * initialize the file descriptor, which was initially set to -1.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::open(unsigned int flags)
{
	if (isOpen()) {
		LOG(V4L2, Error) << "Device already open";
		return -EBUSY;
	}

	UniqueFD fd(syscall(SYS_openat, AT_FDCWD, deviceNode_.c_str(), flags));
	if (!fd.isValid()) {
		int ret = -errno;
		LOG(V4L2, Error) << "Failed to open V4L2 device '"
				 << deviceNode_ << "': "
				 << strerror(-ret);
		return ret;
	}

	setFd(std::move(fd));

	return 0;
}

/**
 * \brief Set the file descriptor of a V4L2 device
 * \param[in] fd The file descriptor handle
 *
 * This function allows a device to provide an already opened file descriptor
 * referring to the V4L2 device node, instead of opening it with open(). This
 * can be used for V4L2 M2M devices where a single video device node is used for
 * both the output and capture devices, or when receiving an open file
 * descriptor in a context that doesn't have permission to open the device node
 * itself.
 *
 * This function and the open() function are mutually exclusive, only one of the
 * two shall be used for a V4L2Device instance.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::setFd(UniqueFD fd)
{
	if (isOpen())
		return -EBUSY;

	fd_ = std::move(fd);

	fdEventNotifier_ = new EventNotifier(fd_.get(), EventNotifier::Exception);
	fdEventNotifier_->activated.connect(this, &V4L2Device::eventAvailable);
	fdEventNotifier_->setEnabled(false);

	listControls();

	return 0;
}

/**
 * \brief Close the device node
 *
 * Reset the file descriptor to -1
 */
void V4L2Device::close()
{
	if (!isOpen())
		return;

	delete fdEventNotifier_;

	fd_.reset();
}

/**
 * \fn V4L2Device::isOpen()
 * \brief Check if the V4L2 device node is open
 * \return True if the V4L2 device node is open, false otherwise
 */

/**
 * \fn V4L2Device::controls()
 * \brief Retrieve the supported V4L2 controls and their information
 * \return A map of the V4L2 controls supported by the device
 */

/**
 * \brief Read controls from the device
 * \param[in] ids The list of controls to read, specified by their ID
 *
 * This function reads the value of all controls contained in \a ids, and
 * returns their values as a ControlList.
 *
 * If any control in \a ids is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), or if any other error occurs
 * during validation of the requested controls, no control is read and this
 * function returns an empty control list.
 *
 * \return The control values in a ControlList on success, or an empty list on
 * error
 */
ControlList V4L2Device::getControls(const std::vector<uint32_t> &ids)
{
	if (ids.empty())
		return {};

	ControlList ctrls{ controls_ };

	for (uint32_t id : ids) {
		const auto iter = controls_.find(id);
		if (iter == controls_.end()) {
			LOG(V4L2, Error)
				<< "Control " << utils::hex(id) << " not found";
			return {};
		}

		ctrls.set(id, {});
	}

	std::vector<v4l2_ext_control> v4l2Ctrls(ids.size());
	memset(v4l2Ctrls.data(), 0, sizeof(v4l2_ext_control) * ctrls.size());

	unsigned int i = 0;
	for (auto &ctrl : ctrls) {
		unsigned int id = ctrl.first;
		const struct v4l2_query_ext_ctrl &info = controlInfo_[id];

		v4l2_ext_control &v4l2Ctrl = v4l2Ctrls[i++];
		v4l2Ctrl.id = id;

		if (info.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
			ControlType type;

			switch (info.type) {
			case V4L2_CTRL_TYPE_U8:
				type = ControlTypeByte;
				break;

			default:
				LOG(V4L2, Error)
					<< "Unsupported payload control type "
					<< info.type;
				return {};
			}

			ControlValue &value = ctrl.second;
			value.reserve(type, true, info.elems);
			Span<uint8_t> data = value.data();

			v4l2Ctrl.p_u8 = data.data();
			v4l2Ctrl.size = data.size();
		}
	}

	struct v4l2_ext_controls v4l2ExtCtrls = {};
	v4l2ExtCtrls.which = V4L2_CTRL_WHICH_CUR_VAL;
	v4l2ExtCtrls.controls = v4l2Ctrls.data();
	v4l2ExtCtrls.count = v4l2Ctrls.size();

	int ret = ioctl(VIDIOC_G_EXT_CTRLS, &v4l2ExtCtrls);
	if (ret) {
		unsigned int errorIdx = v4l2ExtCtrls.error_idx;

		/* Generic validation error. */
		if (errorIdx == 0 || errorIdx >= v4l2Ctrls.size()) {
			LOG(V4L2, Error) << "Unable to read controls: "
					 << strerror(-ret);
			return {};
		}

		/* A specific control failed. */
		const unsigned int id = v4l2Ctrls[errorIdx].id;
		LOG(V4L2, Error) << "Unable to read control " << utils::hex(id)
				 << ": " << strerror(-ret);

		v4l2Ctrls.resize(errorIdx);
	}

	updateControls(&ctrls, v4l2Ctrls);

	return ctrls;
}

/**
 * \brief Write controls to the device
 * \param[in] ctrls The list of controls to write
 *
 * This function writes the value of all controls contained in \a ctrls, and
 * stores the values actually applied to the device in the corresponding
 * \a ctrls entry.
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is read-only, if any other error
 * occurs during validation of the requested controls, no control is written and
 * this function returns -EINVAL.
 *
 * If an error occurs while writing the controls, the index of the first
 * control that couldn't be written is returned. All controls below that index
 * are written and their values are updated in \a ctrls, while all other
 * controls are not written and their values are not changed.
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */
int V4L2Device::setControls(ControlList *ctrls)
{
	if (ctrls->empty())
		return 0;

	std::vector<v4l2_ext_control> v4l2Ctrls(ctrls->size());
	memset(v4l2Ctrls.data(), 0, sizeof(v4l2_ext_control) * ctrls->size());

	for (auto [ctrl, i] = std::pair(ctrls->begin(), 0u); i < ctrls->size(); ctrl++, i++) {
		const unsigned int id = ctrl->first;
		const auto iter = controls_.find(id);
		if (iter == controls_.end()) {
			LOG(V4L2, Error)
				<< "Control " << utils::hex(id) << " not found";
			return -EINVAL;
		}
		v4l2_ext_control &v4l2Ctrl = v4l2Ctrls[i];
		v4l2Ctrl.id = id;

		/* Set the v4l2_ext_control value for the write operation. */
		ControlValue &value = ctrl->second;
		switch (iter->first->type()) {
		case ControlTypeInteger32: {
			if (value.isArray()) {
				Span<uint8_t> data = value.data();
				v4l2Ctrl.p_u32 = reinterpret_cast<uint32_t *>(data.data());
				v4l2Ctrl.size = data.size();
			} else {
				v4l2Ctrl.value = value.get<int32_t>();
			}

			break;
		}

		case ControlTypeInteger64:
			v4l2Ctrl.value64 = value.get<int64_t>();
			break;

		case ControlTypeByte: {
			if (!value.isArray()) {
				LOG(V4L2, Error)
					<< "Control " << utils::hex(id)
					<< " requires an array value";
				return -EINVAL;
			}

			Span<uint8_t> data = value.data();
			v4l2Ctrl.p_u8 = data.data();
			v4l2Ctrl.size = data.size();

			break;
		}

		default:
			/* \todo To be changed to support strings. */
			v4l2Ctrl.value = value.get<int32_t>();
			break;
		}
	}

	struct v4l2_ext_controls v4l2ExtCtrls = {};
	v4l2ExtCtrls.which = V4L2_CTRL_WHICH_CUR_VAL;
	v4l2ExtCtrls.controls = v4l2Ctrls.data();
	v4l2ExtCtrls.count = v4l2Ctrls.size();

	int ret = ioctl(VIDIOC_S_EXT_CTRLS, &v4l2ExtCtrls);
	if (ret) {
		unsigned int errorIdx = v4l2ExtCtrls.error_idx;

		/* Generic validation error. */
		if (errorIdx == 0 || errorIdx >= v4l2Ctrls.size()) {
			LOG(V4L2, Error) << "Unable to set controls: "
					 << strerror(-ret);
			return -EINVAL;
		}

		/* A specific control failed. */
		const unsigned int id = v4l2Ctrls[errorIdx].id;
		LOG(V4L2, Error) << "Unable to set control " << utils::hex(id)
				 << ": " << strerror(-ret);

		v4l2Ctrls.resize(errorIdx);
		ret = errorIdx;
	}

	updateControls(ctrls, v4l2Ctrls);

	return ret;
}

/**
 * \brief Retrieve the v4l2_query_ext_ctrl information for the given control
 * \param[in] id The V4L2 control id
 * \return A pointer to the v4l2_query_ext_ctrl structure for the given
 * control, or a null pointer if not found
 */
const struct v4l2_query_ext_ctrl *V4L2Device::controlInfo(uint32_t id) const
{
	const auto it = controlInfo_.find(id);
	if (it == controlInfo_.end())
		return nullptr;

	return &it->second;
}

/**
 * \brief Retrieve the device path in sysfs
 *
 * This function returns the sysfs path to the physical device backing the V4L2
 * device. The path is guaranteed to be an absolute path, without any symbolic
 * link.
 *
 * It includes the sysfs mount point prefix
 *
 * \return The device path in sysfs
 */
std::string V4L2Device::devicePath() const
{
	std::string devicePath = sysfs::charDevPath(deviceNode_) + "/device";

	char *realPath = realpath(devicePath.c_str(), nullptr);
	if (!realPath) {
		LOG(V4L2, Fatal)
			<< "Can not resolve device path for " << devicePath;
		return {};
	}

	std::string path{ realPath };
	free(realPath);

	return path;
}

/**
 * \brief Enable or disable frame start event notification
 * \param[in] enable True to enable frame start events, false to disable them
 *
 * This function enables or disables generation of frame start events. Once
 * enabled, the events are signalled through the frameStart signal.
 *
 * \return 0 on success, a negative error code otherwise
 */
int V4L2Device::setFrameStartEnabled(bool enable)
{
	if (frameStartEnabled_ == enable)
		return 0;

	struct v4l2_event_subscription event{};
	event.type = V4L2_EVENT_FRAME_SYNC;

	unsigned long request = enable ? VIDIOC_SUBSCRIBE_EVENT
			      : VIDIOC_UNSUBSCRIBE_EVENT;
	int ret = ioctl(request, &event);
	if (enable && ret)
		return ret;

	fdEventNotifier_->setEnabled(enable);
	frameStartEnabled_ = enable;

	return ret;
}

/**
 * \var V4L2Device::frameStart
 * \brief A Signal emitted when capture of a frame has started
 */

/**
 * \brief Perform an IOCTL system call on the device node
 * \param[in] request The IOCTL request code
 * \param[in] argp A pointer to the IOCTL argument
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::ioctl(unsigned long request, void *argp)
{
	/*
	 * Printing out an error message is usually better performed
	 * in the caller, which can provide more context.
	 */
	if (::ioctl(fd_.get(), request, argp) < 0)
		return -errno;

	return 0;
}

/**
 * \fn V4L2Device::deviceNode()
 * \brief Retrieve the device node path
 * \return The device node path
 */

/**
 * \fn V4L2Device::fd()
 * \brief Retrieve the V4L2 device file descriptor
 * \return The V4L2 device file descriptor, -1 if the device node is not open
 */

/**
 * \brief Retrieve the libcamera control type associated with the V4L2 control
 * \param[in] ctrlType The V4L2 control type
 * \return The ControlType associated to \a ctrlType
 */
ControlType V4L2Device::v4l2CtrlType(uint32_t ctrlType)
{
	switch (ctrlType) {
	case V4L2_CTRL_TYPE_U8:
		return ControlTypeByte;

	case V4L2_CTRL_TYPE_BOOLEAN:
		return ControlTypeBool;

	case V4L2_CTRL_TYPE_INTEGER:
		return ControlTypeInteger32;

	case V4L2_CTRL_TYPE_INTEGER64:
		return ControlTypeInteger64;

	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_BUTTON:
	case V4L2_CTRL_TYPE_BITMASK:
	case V4L2_CTRL_TYPE_INTEGER_MENU:
		/*
		 * More precise types may be needed, for now use a 32-bit
		 * integer type.
		 */
		return ControlTypeInteger32;

	default:
		return ControlTypeNone;
	}
}

/**
 * \brief Create a ControlId for a V4L2 control
 * \param[in] ctrl The v4l2_query_ext_ctrl that represents a V4L2 control
 * \return A ControlId associated to \a ctrl
 */
std::unique_ptr<ControlId> V4L2Device::v4l2ControlId(const v4l2_query_ext_ctrl &ctrl)
{
	const size_t len = strnlen(ctrl.name, sizeof(ctrl.name));
	const std::string name(static_cast<const char *>(ctrl.name), len);
	const ControlType type = v4l2CtrlType(ctrl.type);

	return std::make_unique<ControlId>(ctrl.id, name, type);
}

/**
 * \brief Create a ControlInfo for a V4L2 control
 * \param[in] ctrl The v4l2_query_ext_ctrl that represents a V4L2 control
 * \return A ControlInfo that represents \a ctrl
 */
std::optional<ControlInfo> V4L2Device::v4l2ControlInfo(const v4l2_query_ext_ctrl &ctrl)
{
	switch (ctrl.type) {
	case V4L2_CTRL_TYPE_U8:
		return ControlInfo(static_cast<uint8_t>(ctrl.minimum),
				   static_cast<uint8_t>(ctrl.maximum),
				   static_cast<uint8_t>(ctrl.default_value));

	case V4L2_CTRL_TYPE_BOOLEAN:
		return ControlInfo(static_cast<bool>(ctrl.minimum),
				   static_cast<bool>(ctrl.maximum),
				   static_cast<bool>(ctrl.default_value));

	case V4L2_CTRL_TYPE_INTEGER64:
		return ControlInfo(static_cast<int64_t>(ctrl.minimum),
				   static_cast<int64_t>(ctrl.maximum),
				   static_cast<int64_t>(ctrl.default_value));

	case V4L2_CTRL_TYPE_INTEGER_MENU:
	case V4L2_CTRL_TYPE_MENU:
		return v4l2MenuControlInfo(ctrl);

	default:
		return ControlInfo(static_cast<int32_t>(ctrl.minimum),
				   static_cast<int32_t>(ctrl.maximum),
				   static_cast<int32_t>(ctrl.default_value));
	}
}

/**
 * \brief Create ControlInfo for a V4L2 menu control
 * \param[in] ctrl The v4l2_query_ext_ctrl that represents a V4L2 menu control
 *
 * The created ControlInfo contains indices acquired by VIDIOC_QUERYMENU.
 *
 * \return A ControlInfo that represents \a ctrl
 */
std::optional<ControlInfo> V4L2Device::v4l2MenuControlInfo(const struct v4l2_query_ext_ctrl &ctrl)
{
	std::vector<ControlValue> indices;
	struct v4l2_querymenu menu = {};
	menu.id = ctrl.id;

	if (ctrl.minimum < 0)
		return std::nullopt;

	for (int32_t index = ctrl.minimum; index <= ctrl.maximum; ++index) {
		menu.index = index;
		if (ioctl(VIDIOC_QUERYMENU, &menu) != 0)
			continue;

		indices.push_back(index);
	}

	/*
	 * Some faulty UVC devices are known to return an empty menu control.
	 * Controls without a menu option can not be set, or read, so they are
	 * not exposed.
	 */
	if (indices.size() == 0)
		return std::nullopt;

	return ControlInfo(indices,
			   ControlValue(static_cast<int32_t>(ctrl.default_value)));
}

/*
 * \brief List and store information about all controls supported by the
 * V4L2 device
 */
void V4L2Device::listControls()
{
	ControlInfoMap::Map ctrls;
	struct v4l2_query_ext_ctrl ctrl = {};

	while (1) {
		ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL |
			   V4L2_CTRL_FLAG_NEXT_COMPOUND;
		if (ioctl(VIDIOC_QUERY_EXT_CTRL, &ctrl))
			break;

		if (ctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS ||
		    ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			continue;

		switch (ctrl.type) {
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_BUTTON:
		case V4L2_CTRL_TYPE_INTEGER64:
		case V4L2_CTRL_TYPE_BITMASK:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
		case V4L2_CTRL_TYPE_U8:
			break;
		/* \todo Support other control types. */
		default:
			LOG(V4L2, Debug)
				<< "Control " << utils::hex(ctrl.id)
				<< " has unsupported type " << ctrl.type;
			continue;
		}

		LOG(V4L2, Debug) << "Control: " << ctrl.name
				 << " (" << utils::hex(ctrl.id) << ")";

		controlIds_.emplace_back(v4l2ControlId(ctrl));
		controlIdMap_[ctrl.id] = controlIds_.back().get();
		controlInfo_.emplace(ctrl.id, ctrl);

		std::optional<ControlInfo> info = v4l2ControlInfo(ctrl);

		if (!info) {
			LOG(V4L2, Error)
				<< "Control " << ctrl.name
				<< " cannot be registered";

			continue;
		}

		ctrls.emplace(controlIds_.back().get(), *info);
	}

	controls_ = ControlInfoMap(std::move(ctrls), controlIdMap_);
}

/**
* \brief Update the information for all device controls
 *
 * The V4L2Device class caches information about all controls supported by the
 * device and exposes it through the controls() and controlInfo() functions.
 * Control information may change at runtime, for instance when formats on a
 * subdev are modified. When this occurs, this function can be used to refresh
 * control information. The information is refreshed in-place, all pointers to
 * v4l2_query_ext_ctrl instances previously returned by controlInfo() and
 * iterators to the ControlInfoMap returned by controls() remain valid.
 *
 * Note that control information isn't refreshed automatically is it may be an
 * expensive operation. The V4L2Device users are responsible for calling this
 * function when required, based on their usage pattern of the class.
 */
void V4L2Device::updateControlInfo()
{
	for (auto &[controlId, info] : controls_) {
		unsigned int id = controlId->id();

		/*
		 * Assume controlInfo_ has a corresponding entry, as it has been
		 * generated by listControls().
		 */
		struct v4l2_query_ext_ctrl &ctrl = controlInfo_[id];

		if (ioctl(VIDIOC_QUERY_EXT_CTRL, &ctrl)) {
			LOG(V4L2, Debug)
				<< "Could not refresh control "
				<< utils::hex(id);
			continue;
		}

		info = *v4l2ControlInfo(ctrl);
	}
}

/*
 * \brief Update the value of the first \a count V4L2 controls in \a ctrls using
 * values in \a v4l2Ctrls
 * \param[inout] ctrls List of V4L2 controls to update
 * \param[in] v4l2Ctrls List of V4L2 extended controls as returned by the driver
 */
void V4L2Device::updateControls(ControlList *ctrls,
				Span<const v4l2_ext_control> v4l2Ctrls)
{
	for (const v4l2_ext_control &v4l2Ctrl : v4l2Ctrls) {
		const unsigned int id = v4l2Ctrl.id;

		ControlValue value = ctrls->get(id);
		if (value.isArray()) {
			/*
			 * No action required, the VIDIOC_[GS]_EXT_CTRLS ioctl
			 * accessed the ControlValue storage directly for array
			 * controls.
			 */
			continue;
		}

		const auto iter = controls_.find(id);
		ASSERT(iter != controls_.end());

		switch (iter->first->type()) {
		case ControlTypeInteger64:
			value.set<int64_t>(v4l2Ctrl.value64);
			break;

		default:
			/*
			 * Note: this catches the ControlTypeInteger32 case.
			 *
			 * \todo To be changed when support for string controls
			 * will be added.
			 */
			value.set<int32_t>(v4l2Ctrl.value);
			break;
		}

		ctrls->set(id, value);
	}
}

/**
 * \brief Slot to handle V4L2 events from the V4L2 device
 *
 * When this slot is called, a V4L2 event is available to be dequeued from the
 * device.
 */
void V4L2Device::eventAvailable()
{
	struct v4l2_event event{};
	int ret = ioctl(VIDIOC_DQEVENT, &event);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to dequeue event, disabling event notifier";
		fdEventNotifier_->setEnabled(false);
		return;
	}

	if (event.type != V4L2_EVENT_FRAME_SYNC) {
		LOG(V4L2, Error)
			<< "Spurious event (" << event.type
			<< "), disabling event notifier";
		fdEventNotifier_->setEnabled(false);
		return;
	}

	frameStart.emit(event.u.frame_sync.frame_sequence);
}

static const std::map<uint32_t, ColorSpace> v4l2ToColorSpace = {
	{ V4L2_COLORSPACE_RAW, ColorSpace::Raw },
	{ V4L2_COLORSPACE_SRGB, {
		ColorSpace::Primaries::Rec709,
		ColorSpace::TransferFunction::Srgb,
		ColorSpace::YcbcrEncoding::Rec601,
		ColorSpace::Range::Limited } },
	{ V4L2_COLORSPACE_JPEG, ColorSpace::Sycc },
	{ V4L2_COLORSPACE_SMPTE170M, ColorSpace::Smpte170m },
	{ V4L2_COLORSPACE_REC709, ColorSpace::Rec709 },
	{ V4L2_COLORSPACE_BT2020, ColorSpace::Rec2020 },
};

static const std::map<uint32_t, ColorSpace::TransferFunction> v4l2ToTransferFunction = {
	{ V4L2_XFER_FUNC_NONE, ColorSpace::TransferFunction::Linear },
	{ V4L2_XFER_FUNC_SRGB, ColorSpace::TransferFunction::Srgb },
	{ V4L2_XFER_FUNC_709, ColorSpace::TransferFunction::Rec709 },
};

static const std::map<uint32_t, ColorSpace::YcbcrEncoding> v4l2ToYcbcrEncoding = {
	{ V4L2_YCBCR_ENC_601, ColorSpace::YcbcrEncoding::Rec601 },
	{ V4L2_YCBCR_ENC_709, ColorSpace::YcbcrEncoding::Rec709 },
	{ V4L2_YCBCR_ENC_BT2020, ColorSpace::YcbcrEncoding::Rec2020 },
};

static const std::map<uint32_t, ColorSpace::Range> v4l2ToRange = {
	{ V4L2_QUANTIZATION_FULL_RANGE, ColorSpace::Range::Full },
	{ V4L2_QUANTIZATION_LIM_RANGE, ColorSpace::Range::Limited },
};

static const std::vector<std::pair<ColorSpace, v4l2_colorspace>> colorSpaceToV4l2 = {
	{ ColorSpace::Raw, V4L2_COLORSPACE_RAW },
	{ ColorSpace::Sycc, V4L2_COLORSPACE_JPEG },
	{ ColorSpace::Smpte170m, V4L2_COLORSPACE_SMPTE170M },
	{ ColorSpace::Rec709, V4L2_COLORSPACE_REC709 },
	{ ColorSpace::Rec2020, V4L2_COLORSPACE_BT2020 },
};

static const std::map<ColorSpace::Primaries, v4l2_colorspace> primariesToV4l2 = {
	{ ColorSpace::Primaries::Raw, V4L2_COLORSPACE_RAW },
	{ ColorSpace::Primaries::Smpte170m, V4L2_COLORSPACE_SMPTE170M },
	{ ColorSpace::Primaries::Rec709, V4L2_COLORSPACE_REC709 },
	{ ColorSpace::Primaries::Rec2020, V4L2_COLORSPACE_BT2020 },
};

static const std::map<ColorSpace::TransferFunction, v4l2_xfer_func> transferFunctionToV4l2 = {
	{ ColorSpace::TransferFunction::Linear, V4L2_XFER_FUNC_NONE },
	{ ColorSpace::TransferFunction::Srgb, V4L2_XFER_FUNC_SRGB },
	{ ColorSpace::TransferFunction::Rec709, V4L2_XFER_FUNC_709 },
};

static const std::map<ColorSpace::YcbcrEncoding, v4l2_ycbcr_encoding> ycbcrEncodingToV4l2 = {
	/* V4L2 has no "none" encoding. */
	{ ColorSpace::YcbcrEncoding::None, V4L2_YCBCR_ENC_DEFAULT },
	{ ColorSpace::YcbcrEncoding::Rec601, V4L2_YCBCR_ENC_601 },
	{ ColorSpace::YcbcrEncoding::Rec709, V4L2_YCBCR_ENC_709 },
	{ ColorSpace::YcbcrEncoding::Rec2020, V4L2_YCBCR_ENC_BT2020 },
};

static const std::map<ColorSpace::Range, v4l2_quantization> rangeToV4l2 = {
	{ ColorSpace::Range::Full, V4L2_QUANTIZATION_FULL_RANGE },
	{ ColorSpace::Range::Limited, V4L2_QUANTIZATION_LIM_RANGE },
};

/**
 * \brief Convert the color space fields in a V4L2 format to a ColorSpace
 * \param[in] v4l2Format A V4L2 format containing color space information
 * \param[in] colourEncoding Type of colour encoding
 *
 * The colorspace, ycbcr_enc, xfer_func and quantization fields within a
 * V4L2 format structure are converted to a corresponding ColorSpace.
 *
 * If any V4L2 fields are not recognised then we return an "unset"
 * color space.
 *
 * \return The ColorSpace corresponding to the input V4L2 format
 * \retval std::nullopt One or more V4L2 color space fields were not recognised
 */
template<typename T>
std::optional<ColorSpace> V4L2Device::toColorSpace(const T &v4l2Format,
						   PixelFormatInfo::ColourEncoding colourEncoding)
{
	auto itColor = v4l2ToColorSpace.find(v4l2Format.colorspace);
	if (itColor == v4l2ToColorSpace.end())
		return std::nullopt;

	/* This sets all the color space fields to the correct "default" values. */
	ColorSpace colorSpace = itColor->second;

	if (v4l2Format.xfer_func != V4L2_XFER_FUNC_DEFAULT) {
		auto itTransfer = v4l2ToTransferFunction.find(v4l2Format.xfer_func);
		if (itTransfer == v4l2ToTransferFunction.end())
			return std::nullopt;

		colorSpace.transferFunction = itTransfer->second;
	}

	if (v4l2Format.ycbcr_enc != V4L2_YCBCR_ENC_DEFAULT) {
		auto itYcbcrEncoding = v4l2ToYcbcrEncoding.find(v4l2Format.ycbcr_enc);
		if (itYcbcrEncoding == v4l2ToYcbcrEncoding.end())
			return std::nullopt;

		colorSpace.ycbcrEncoding = itYcbcrEncoding->second;

		/*
		 * V4L2 has no "none" encoding, override the value returned by
		 * the kernel for non-YUV formats as YCbCr encoding isn't
		 * applicable in that case.
		 */
		if (colourEncoding != PixelFormatInfo::ColourEncodingYUV)
			colorSpace.ycbcrEncoding = ColorSpace::YcbcrEncoding::None;
	}

	if (v4l2Format.quantization != V4L2_QUANTIZATION_DEFAULT) {
		auto itRange = v4l2ToRange.find(v4l2Format.quantization);
		if (itRange == v4l2ToRange.end())
			return std::nullopt;

		colorSpace.range = itRange->second;

		/*
		 * "Limited" quantization range is only meant for YUV formats.
		 * Override the range to "Full" for all other formats.
		 */
		if (colourEncoding != PixelFormatInfo::ColourEncodingYUV)
			colorSpace.range = ColorSpace::Range::Full;
	}

	return colorSpace;
}

template std::optional<ColorSpace> V4L2Device::toColorSpace(const struct v4l2_pix_format &,
							    PixelFormatInfo::ColourEncoding);
template std::optional<ColorSpace> V4L2Device::toColorSpace(const struct v4l2_pix_format_mplane &,
							    PixelFormatInfo::ColourEncoding);
template std::optional<ColorSpace> V4L2Device::toColorSpace(const struct v4l2_mbus_framefmt &,
							    PixelFormatInfo::ColourEncoding);

/**
 * \brief Fill in the color space fields of a V4L2 format from a ColorSpace
 * \param[in] colorSpace The ColorSpace to be converted
 * \param[out] v4l2Format A V4L2 format containing color space information
 *
 * The colorspace, ycbcr_enc, xfer_func and quantization fields within a
 * V4L2 format structure are filled in from a corresponding ColorSpace.
 *
 * An error is returned if any of the V4L2 fields do not support the
 * value given in the ColorSpace. Such fields are set to the V4L2
 * "default" values, but all other fields are still filled in where
 * possible.
 *
 * If the color space is completely unset, "default" V4L2 values are used
 * everywhere, so a driver would then choose its preferred color space.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL The ColorSpace does not have a representation using V4L2 enums
 */
template<typename T>
int V4L2Device::fromColorSpace(const std::optional<ColorSpace> &colorSpace, T &v4l2Format)
{
	v4l2Format.colorspace = V4L2_COLORSPACE_DEFAULT;
	v4l2Format.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	v4l2Format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	v4l2Format.quantization = V4L2_QUANTIZATION_DEFAULT;

	if (!colorSpace)
		return 0;

	auto itColor = std::find_if(colorSpaceToV4l2.begin(), colorSpaceToV4l2.end(),
				    [&colorSpace](const auto &item) {
					    return colorSpace == item.first;
				    });
	if (itColor != colorSpaceToV4l2.end()) {
		v4l2Format.colorspace = itColor->second;
		/* Leaving all the other fields as "default" should be fine. */
		return 0;
	}

	/*
	 * If the colorSpace doesn't precisely match a standard color space,
	 * then we must choose a V4L2 colorspace with matching primaries.
	 */
	int ret = 0;

	auto itPrimaries = primariesToV4l2.find(colorSpace->primaries);
	if (itPrimaries != primariesToV4l2.end()) {
		v4l2Format.colorspace = itPrimaries->second;
	} else {
		libcamera::LOG(V4L2, Warning)
			<< "Unrecognised primaries in "
			<< ColorSpace::toString(colorSpace);
		ret = -EINVAL;
	}

	auto itTransfer = transferFunctionToV4l2.find(colorSpace->transferFunction);
	if (itTransfer != transferFunctionToV4l2.end()) {
		v4l2Format.xfer_func = itTransfer->second;
	} else {
		libcamera::LOG(V4L2, Warning)
			<< "Unrecognised transfer function in "
			<< ColorSpace::toString(colorSpace);
		ret = -EINVAL;
	}

	auto itYcbcrEncoding = ycbcrEncodingToV4l2.find(colorSpace->ycbcrEncoding);
	if (itYcbcrEncoding != ycbcrEncodingToV4l2.end()) {
		v4l2Format.ycbcr_enc = itYcbcrEncoding->second;
	} else {
		libcamera::LOG(V4L2, Warning)
			<< "Unrecognised YCbCr encoding in "
			<< ColorSpace::toString(colorSpace);
		ret = -EINVAL;
	}

	auto itRange = rangeToV4l2.find(colorSpace->range);
	if (itRange != rangeToV4l2.end()) {
		v4l2Format.quantization = itRange->second;
	} else {
		libcamera::LOG(V4L2, Warning)
			<< "Unrecognised quantization in "
			<< ColorSpace::toString(colorSpace);
		ret = -EINVAL;
	}

	return ret;
}

template int V4L2Device::fromColorSpace(const std::optional<ColorSpace> &, struct v4l2_pix_format &);
template int V4L2Device::fromColorSpace(const std::optional<ColorSpace> &, struct v4l2_pix_format_mplane &);
template int V4L2Device::fromColorSpace(const std::optional<ColorSpace> &, struct v4l2_mbus_framefmt &);

} /* namespace libcamera */
