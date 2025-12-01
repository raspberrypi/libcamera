
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025 Raspberry Pi Ltd
 *
 * media_device.cpp - PiSP media device helper
 */

#include "media_device.hpp"

#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <sys/ioctl.h>
#include <vector>

namespace fs = std::filesystem;
using namespace libpisp::helpers;

struct DeviceNode
{
	std::string name;
	std::string dev_node;
};

struct MediaDevMap
{
	MediaDevMap(const std::string &_media_node, const std::vector<DeviceNode> &&_device_nodes)
		: media_node(_media_node), device_nodes(_device_nodes)
	{
	}
	std::string media_node;
	std::vector<DeviceNode> device_nodes;
};

class MediaEnumerator
{
public:
	using MediaDevList = std::vector<MediaDevMap>;

	MediaEnumerator(MediaEnumerator &other) = delete;
	void operator=(const MediaEnumerator &other) = delete;

	static const MediaEnumerator *Get()
	{
		static std::unique_ptr<MediaEnumerator> mdev(new MediaEnumerator);
		return mdev.get();
	}

	const MediaDevList &MediaDeviceList() const
	{
		return device_list_;
	}

private:
	MediaEnumerator()
	{
		// Sysfs path for media devices - correct for Raspberry Pi, but not sure if this is globally consistent.
		const fs::path media_sysfs("/sys/bus/media/devices");

		if (!fs::exists(media_sysfs))
			return;

		for (auto const &media_device : fs::directory_iterator { media_sysfs })
		{
			if (!fs::is_symlink(media_device))
				continue;

			// Find a pisp_be device link,
			const std::string link(fs::read_symlink(media_device).string());
			if (link.find("pisp_be") == std::string::npos)
				continue;

			// Find the /dev/mediaX device node.
			const std::string::size_type pos = link.find_last_of("/");
			const std::string media_node("/dev/" + link.substr(pos + 1));

			DeviceFd fd(media_node.c_str(), O_RDWR | O_CLOEXEC);
			if (!fd.Valid())
				continue;

			struct media_v2_topology topology;
			std::memset(&topology, 0, sizeof(topology));

			// First ioctl call to get the number of interfaces.
			int ret = ioctl(fd.Get(), MEDIA_IOC_G_TOPOLOGY, &topology);
			if (ret < 0 || !topology.num_interfaces)
				continue;

			auto interfaces = std::make_unique<struct media_v2_interface[]>(topology.num_interfaces);
			topology.ptr_interfaces = reinterpret_cast<uintptr_t>(interfaces.get());

			// Second ioctl call with allocated space to populate the interface array.
			ret = ioctl(fd.Get(), MEDIA_IOC_G_TOPOLOGY, &topology);
			if (ret < 0)
				continue;

			std::vector<DeviceNode> device_node;
			for (unsigned int i = 0; i < topology.num_interfaces; i++)
			{
				const media_v2_interface &intf = interfaces[i];

				if (intf.intf_type != MEDIA_INTF_T_V4L_VIDEO)
					break;

				// Find the char dev with the major:minor advertised by the interface.
				const fs::path char_dev("/sys/dev/char/" + std::to_string(intf.devnode.major) + ":" +
										std::to_string(intf.devnode.minor));
				if (!fs::is_symlink(char_dev))
					break;

				const std::string char_dev_link = fs::read_symlink(char_dev).string();
				const std::string::size_type npos = char_dev_link.find_last_of("/");
				const std::string dev_node("/dev/" + char_dev_link.substr(npos + 1));

				// Finally get the /dev/videoX node for the interface.
				std::ifstream dev_name_file(char_dev / "name");
				if (!dev_name_file.is_open())
					break;

				std::string dev_name;
				std::getline(dev_name_file, dev_name);
				device_node.push_back({ dev_name, dev_node });
			}

			// If we have some device nodes enumerated, add them to our map.
			if (!device_node.empty())
				device_list_.emplace_back(media_node, std::move(device_node));
		}
	}

	MediaDevList device_list_;
};

MediaDevice::MediaDevice()
	: media_enumerator_(MediaEnumerator::Get())
{
}

MediaDevice::~MediaDevice()
{
	for (auto it = lock_map_.begin(); it != lock_map_.end();)
		it = unlock(it->first);
}

std::string MediaDevice::Acquire(const std::string &device)
{
	for (const auto &m : media_enumerator_->MediaDeviceList())
	{
		if (!device.empty() && m.media_node != device)
			continue;

		// Check if this process has the lock.
		auto it = lock_map_.find(m.media_node.c_str());
		if (it != lock_map_.end())
			continue;

		// Check if another process has the lock.
		DeviceFd fd(m.media_node.c_str(), O_RDWR | O_CLOEXEC);
		if (!fd.Valid())
			continue;

		if (lockf(fd.Get(), F_TLOCK, 0))
			continue;

		lock_map_.emplace(std::piecewise_construct,
						  std::forward_as_tuple(m.media_node), std::forward_as_tuple(std::move(fd)));

		return m.media_node;
	}

	return {};
}

void MediaDevice::Release(const std::string &device)
{
	unlock(device);
}

V4l2DevMap MediaDevice::OpenV4l2Nodes(const std::string &device) const
{
	V4l2DevMap dev_map;

	for (const auto &m : media_enumerator_->MediaDeviceList())
	{
		if (m.media_node != device)
			continue;

		for (auto const &n : m.device_nodes)
		{
			V4l2Device dev(n.dev_node);
			if (!dev.Valid())
				return {};

			dev_map.emplace(std::piecewise_construct, std::forward_as_tuple(n.name),
							std::forward_as_tuple(std::move(dev)));
		}

		return dev_map;
	}

	return {};
}

void MediaDevice::CloseV4l2Nodes(V4l2DevMap &device_map)
{
	for (auto &d : device_map)
		d.second.Close();
}

std::string MediaDevice::List() const
{
	std::stringstream ss;

	for (const auto &it : media_enumerator_->MediaDeviceList())
	{
		ss << std::endl << it.media_node << std::endl;
		for (const auto &it1 : it.device_nodes)
			ss << "    " << it1.dev_node << " " << it1.name << std::endl;
	}

	return ss.str();
}

media_device_info MediaDevice::DeviceInfo(const std::string &device) const
{
	media_device_info info;

	DeviceFd fd(device.c_str(), O_RDONLY | O_CLOEXEC);
	if (!fd.Valid())
		return {};

	int ret = ioctl(fd.Get(), MEDIA_IOC_DEVICE_INFO, &info);
	if (ret)
		return {};

	return info;
}

std::map<std::string, DeviceFd>::iterator MediaDevice::unlock(const std::string &device)
{
	auto it = lock_map_.find(device);
	if (it == lock_map_.end())
		return lock_map_.end();

	std::ignore = lockf(it->second.Get(), F_ULOCK, 0);
	return lock_map_.erase(it);
}
