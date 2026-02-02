/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * V4L2 Parameters
 */

#pragma once

#include <map>
#include <stdint.h>
#include <string.h>

#include <linux/media/v4l2-isp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

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

template<typename Traits>
class V4L2Params
{
public:
	V4L2Params(Span<uint8_t> data, unsigned int version)
		: data_(data)
	{
		struct v4l2_isp_params_buffer *params =
			reinterpret_cast<struct v4l2_isp_params_buffer *>(data_.data());
		params->data_size = 0;
		params->version = version;

		used_ = offsetof(struct v4l2_isp_params_buffer, data);
	}

	size_t bytesused() const { return used_; }

	template<typename Traits::id_type Id>
	auto block()
	{
		using Details = typename Traits::template id_to_details<Id>;

		using Type = typename Details::type;
		constexpr auto kernelId = Details::blockType;

		auto data = block(Id, kernelId, sizeof(Type));
		return V4L2ParamsBlock<Type>(data);
	}

protected:
	Span<uint8_t> block(typename Traits::id_type type,
			    unsigned int blockType, size_t blockSize)
	{
		/*
		 * Look up the block in the cache first. If an algorithm
		 * requests the same block type twice, it should get the same
		 * block.
		 */
		auto cacheIt = blocks_.find(type);
		if (cacheIt != blocks_.end())
			return cacheIt->second;

		/*
		 * Make sure we don't run out of space. Assert as otherwise
		 * we get a segfault as soon as someone tries to access the
		 * empty Span<> returned from here.
		 */
		if (blockSize > data_.size() - used_) {
			LOG(Fatal)
				<< "Parameters buffer out of space; potential version mismatch between driver and libcamera";
			return {};
		}

		/* Allocate a new block, clear its memory, and initialize its header. */
		Span<uint8_t> block = data_.subspan(used_, blockSize);
		memset(block.data(), 0, block.size());

		struct v4l2_isp_params_block_header *header =
			reinterpret_cast<struct v4l2_isp_params_block_header *>(block.data());
		header->type = blockType;
		header->size = block.size();

		used_ += block.size();

		reinterpret_cast<struct v4l2_isp_params_buffer *>
			(data_.data())->data_size += block.size();

		/* Update the cache. */
		blocks_[type] = block;

		return block;
	}

	Span<uint8_t> data_;
	size_t used_;

	std::map<typename Traits::id_type, Span<uint8_t>> blocks_;
};

} /* namespace ipa */

} /* namespace libcamera */
