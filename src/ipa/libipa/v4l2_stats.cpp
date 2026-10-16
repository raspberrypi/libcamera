/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * V4L2 ISP Statistics
 */

#include "v4l2_stats.h"

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(V4L2Stats)

/**
 * \file v4l2_stats.cpp
 * \brief Helper class to handle an ISP statistics buffer compatible with
 * the generic V4L2 ISP format
 *
 * The Linux kernel defines a generic buffer format for ISP statistics.
 * The format describes a serialisation method that allows userspace to
 * access statistics data from a binary buffer.
 *
 * The V4L2Stats class implements support for the V4L2 ISP statistics buffer
 * format and allows users to retrieve an ISP statistics block.
 *
 * IPA implementations using these helpers should define an enumeration of ISP
 * blocks supported by the IPA module and use a set of common abstractions to
 * help their derived implementation of V4L2Stats translate the enumerated ISP
 * block identifiers to the actual type of the statistics data as defined by
 * the kernel interface.
 */

/**
 * \class V4L2StatsBase
 * \brief Base class for V4L2Stats
 *
 * The V4L2StatsBase is an integral part of V4L2Stats. It serves as a
 * container for all code that does not depend on the V4L2Stats template
 * arguments, to avoid duplicate copies of inline code.
 */

/**
 * \brief Construct an instance of V4L2StatsBase
 * \param[in] data Reference to the v4l2-buffer memory mapped area
 * \param[in] version The ISP parameters version the implementation supports
 *
 * Parse the statistics buffer and construct a cache that maps a block type to
 * the memory location of a statistics block in the buffer.
 *
 * After construction users of this class shall check the validity of the
 * constructed instance using operator bool().
 */
V4L2StatsBase::V4L2StatsBase(std::span<uint8_t> data, unsigned int version)
	: data_(data), valid_(false)
{
	const struct v4l2_isp_buffer *stats =
		reinterpret_cast<const struct v4l2_isp_buffer *>(data_.data());

	if (data_.size() - sizeof(*stats) < stats->data_size) {
		LOG(V4L2Stats, Error)
			<< "Stats buffer size mismatch: " << stats->data_size;
		return;
	}

	if (version != stats->version) {
		LOG(V4L2Stats, Error)
			<< "Unsupported v4l2-isp version: " << stats->version;
		return;
	}

	/* Construct the cache for easier lookup. */
	size_t left = stats->data_size;
	const __u8 *d = stats->data;

	while (left > 0) {
		const struct v4l2_isp_block_header *header =
			reinterpret_cast<const struct v4l2_isp_block_header *>(d);

		if (left < sizeof(*header) || header->size < sizeof(*header)) {
			LOG(V4L2Stats, Error)
				<< "Block type " << header->type
				<< " size is not valid";
			return;
		}

		if (left < header->size) {
			LOG(V4L2Stats, Error)
				<< "Not enough space for block type " << header->type;
			return;
		}

		auto [it, inserted] = cache_.try_emplace(header->type, d, header->size);
		if (!inserted) {
			LOG(V4L2Stats, Error)
				<< "Duplicated block type " << header->type;
			return;
		}

		d += header->size;
		left -= header->size;
	}

	valid_ = true;
}

/**
 * \brief Retrieve an ISP statistics block and return a reference to it
 * \param[in] blockType The kernel-defined ISP block identifier, used to
 * identify the block header
 * \param[in] blockSize The ISP statistics block size, for validation
 *
 * Retrieve a span to the statistics block memory location by accessing the
 * cache built at class construction time.
 *
 * \return The memory location of the ISP statistics block, or an empty span
 * if \a blockType is not supported
 */
std::span<const uint8_t> V4L2StatsBase::block(unsigned int blockType, size_t blockSize) const
{
	const auto it = cache_.find(blockType);
	if (it == cache_.end()) {
		LOG(V4L2Stats, Error) << "Unsupported stats block type: "
				      << blockType;
		return {};
	}

	const struct v4l2_isp_block_header *header =
		reinterpret_cast<const struct v4l2_isp_block_header *>(it->second.data());
	if (header->size != blockSize) {
		LOG(V4L2Stats, Error)
			<< "Block type " << blockType
			<< " size mistmatch: expected "
			<< blockSize << " got:"
			<< header->size;
		return {};
	}

	return it->second;
}

/**
 * \fn V4L2StatsBase::operator bool()
 * \brief Retrieve if a V4L2StatsBase has been successfully constructed
 * \return True if the instance has been constructed successfully, false
 * otherwise
 */

/**
 * \class V4L2Stats
 * \brief Helper class that represents an ISP statistics buffer
 *
 * This class represents an ISP statistics buffer. It is constructed with a
 * reference to the memory mapped buffer that has been dequeued from the ISP
 * driver.
 *
 * This class is templated with the type of the enumeration of ISP blocks that
 * each IPA module is expected to support. IPA modules are expected to derive
 * this class by providing a 'stats_traits' type that helps the class associate
 * a block type with the actual memory area that represents the ISP statistics
 * block.
 *
 * \code{.cpp}
 *
 * // Define the supported ISP statistics blocks
 * enum class myISPStats {
 *	Agc,
 *	Awb,
 *	...
 * };
 *
 * // Maps the C++ enum type to the kernel enum type and concrete parameter type
 * template<myISPStats B>
 * struct block_type {
 * };
 *
 * template<>
 * struct block_type<myISPStats::Agc> {
 *	using type = struct my_isp_kernel_stats_type_agc;
 *	static constexpr kernel_enum_type blockType = MY_ISP_STATS_TYPE_AGC;
 * };
 *
 * template<>
 * struct block_type<myISPStats::Awb> {
 *	using type = struct my_isp_kernel_stats_type_awb;
 *	static constexpr kernel_enum_type blockType = MY_ISP_STATS_TYPE_AWB;
 * };
 *
 * // Convenience type to associate a block id to the 'block_type' overload
 * struct stats_traits {
 * 	using id_type = myISPStats;
 * 	template<id_type Id> using id_to_details = block_type<Id>;
 * };
 *
 * ...
 *
 * // Derive the V4L2Stats class by providing stats_traits
 * class MyISPStats : public V4L2Stats<stats_traits>
 * {
 * public:
 * 	MyISPStats::MyISPStats(std::span<uint8_t> data)
 * 		: V4L2Stats(data, V4L2_ISP_VERSION_V1)
 * 	{
 * 	}
 * };
 *
 * \endcode
 *
 * Users of this class can then easily access an ISP statistics block using the
 * block() function.
 *
 * \code{.cpp}
 *
 * MyISPStats stats(data);
 *
 * auto awb = stats.block<myISPStats::AWB>();
 * auto mean_r = awb->mean_r;
 * auto mean_g = awb->mean_g;
 * auto mean_b = awb->mean_b;
 *
 * \endcode
 */

/**
 * \fn V4L2Stats::V4L2Stats()
 * \brief Construct an instance of V4L2Stats
 * \param[in] data Reference to the v4l2-buffer memory mapped area
 * \param[in] version The expected V4L2 ISP serialization format version
 */

/**
 * \fn V4L2Stats::block() const
 * \brief Retrieve a pointer to an ISP statistics block
 * \return A pointer to the ISP statistics block or nullptr if the block is
 * not present in the statistics buffer
 */

} /* namespace ipa */

} /* namespace libcamera */
