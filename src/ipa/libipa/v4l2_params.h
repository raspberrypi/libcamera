/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * V4L2 ISP Parameters
 */

#pragma once

#include <map>
#include <stdint.h>
#include <string.h>

#include <linux/media/v4l2-isp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

namespace libcamera {

namespace ipa {

LOG_DECLARE_CATEGORY(V4L2Params)

template<typename T>
class V4L2ParamsBlock
{
public:
	V4L2ParamsBlock(const Span<uint8_t> data)
		: data_(data)
	{
	}

	virtual ~V4L2ParamsBlock() {}

	virtual void setEnabled(bool enabled)
	{
		struct v4l2_isp_params_block_header *header =
			reinterpret_cast<struct v4l2_isp_params_block_header *>(data_.data());

		header->flags &= ~(V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
				   V4L2_ISP_PARAMS_FL_BLOCK_DISABLE);
		header->flags |= enabled ? V4L2_ISP_PARAMS_FL_BLOCK_ENABLE
					 : V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
	}

	virtual const T *operator->() const
	{
		return reinterpret_cast<const T *>(data_.data());
	}

	virtual T *operator->()
	{
		return reinterpret_cast<T *>(data_.data());
	}

	virtual const T &operator*() const
	{
		return *reinterpret_cast<const T *>(data_.data());
	}

	virtual T &operator*()
	{
		return *reinterpret_cast<T *>(data_.data());
	}

protected:
	Span<uint8_t> data_;
};

class V4L2ParamsBase
{
protected:
	V4L2ParamsBase(Span<uint8_t> data, unsigned int version);

public:
	size_t bytesused() const { return used_; }

protected:
	Span<uint8_t> block(uint16_t type, unsigned int blockType,
			    size_t blockSize);

	Span<uint8_t> data_;
	size_t used_;

	std::map<uint16_t, Span<uint8_t>> blocks_;
};

template<typename Traits>
class V4L2Params : public V4L2ParamsBase
{
public:
	static_assert(std::is_same_v<std::underlying_type_t<typename Traits::id_type>, uint16_t>);

	V4L2Params(Span<uint8_t> data, unsigned int version)
		: V4L2ParamsBase(data, version)
	{
	}

	template<typename Traits::id_type Id>
	auto block()
	{
		using Details = typename Traits::template id_to_details<Id>;

		using Type = typename Details::type;
		constexpr auto kernelId = Details::blockType;

		auto data = V4L2ParamsBase::block(utils::to_underlying(Id),
						  kernelId, sizeof(Type));
		return V4L2ParamsBlock<Type>(data);
	}
};

} /* namespace ipa */

} /* namespace libcamera */
