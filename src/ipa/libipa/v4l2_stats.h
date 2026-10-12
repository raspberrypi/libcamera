/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * V4L2 ISP Statistics
 */

#pragma once

#include <map>
#include <span>
#include <stdint.h>

#include <linux/media/v4l2-isp.h>

namespace libcamera {

namespace ipa {

class V4L2StatsBase
{
public:
	V4L2StatsBase(std::span<uint8_t> data, unsigned int version);

	std::span<const uint8_t> block(unsigned int blockType, size_t blockSize) const;
	constexpr explicit operator bool()
	{
		return valid_;
	}

private:
	std::map<uint16_t, std::span<const uint8_t>> cache_;
	std::span<uint8_t> data_;
	bool valid_;
};

template<typename Traits>
class V4L2Stats : public V4L2StatsBase
{
public:
	static_assert(std::is_same_v<std::underlying_type_t<typename Traits::id_type>, uint16_t>);

	V4L2Stats(std::span<uint8_t> data, unsigned int version)
		: V4L2StatsBase(data, version)
	{
	}

	template<typename Traits::id_type Id>
	const typename Traits::template id_to_details<Id>::type *
	block() const
	{
		using Details = typename Traits::template id_to_details<Id>;

		using Type = typename Details::type;
		constexpr auto kernelId = Details::blockType;

		auto data = V4L2StatsBase::block(kernelId, sizeof(Type));

		return data.size() > 0 ?
		       reinterpret_cast<const Type *>(data.data()) : nullptr;
	}
};

} /* namespace ipa */

} /* namespace libcamera */
