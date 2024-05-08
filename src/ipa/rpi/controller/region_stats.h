/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * Raspberry Pi region based statistics container
 */
#pragma once

#include <array>
#include <stdint.h>
#include <vector>

#include <libcamera/geometry.h>

namespace RPiController {

template<typename T>
class RegionStats
{
public:
	struct Region {
		T val;
		uint32_t counted;
		uint32_t uncounted;
	};

	RegionStats()
		: size_({}), numFloating_(0), default_({})
	{
	}

	void init(const libcamera::Size &size, unsigned int numFloating = 0)
	{
		size_ = size;
		numFloating_ = numFloating;
		regions_.clear();
		regions_.resize(size_.width * size_.height + numFloating_);
	}

	void init(unsigned int num)
	{
		size_ = libcamera::Size(num, 1);
		numFloating_ = 0;
		regions_.clear();
		regions_.resize(num);
	}

	unsigned int numRegions() const
	{
		return size_.width * size_.height;
	}

	unsigned int numFloatingRegions() const
	{
		return numFloating_;
	}

	libcamera::Size size() const
	{
		return size_;
	}

	void set(unsigned int index, const Region &region)
	{
		if (index >= numRegions())
			return;
		set_(index, region);
	}

	void set(const libcamera::Point &pos, const Region &region)
	{
		set(pos.y * size_.width + pos.x, region);
	}

	void setFloating(unsigned int index, const Region &region)
	{
		if (index >= numFloatingRegions())
			return;
		set(numRegions() + index, region);
	}

	const Region &get(unsigned int index) const
	{
		if (index >= numRegions())
			return default_;
		return get_(index);
	}

	const Region &get(const libcamera::Point &pos) const
	{
		return get(pos.y * size_.width + pos.x);
	}

	const Region &getFloating(unsigned int index) const
	{
		if (index >= numFloatingRegions())
			return default_;
		return get_(numRegions() + index);
	}

	typename std::vector<Region>::iterator begin() { return regions_.begin(); }
	typename std::vector<Region>::iterator end() { return regions_.end(); }
	typename std::vector<Region>::const_iterator begin() const { return regions_.begin(); }
	typename std::vector<Region>::const_iterator end() const { return regions_.end(); }

private:
	void set_(unsigned int index, const Region &region)
	{
		regions_[index] = region;
	}

	const Region &get_(unsigned int index) const
	{
		return regions_[index];
	}

	libcamera::Size size_;
	unsigned int numFloating_;
	std::vector<Region> regions_;
	Region default_;
};

} /* namespace RPiController */
