/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * V4L2 Parameters
 */

#include "v4l2_params.h"

namespace libcamera {

namespace ipa {

/**
 * \file v4l2_params.cpp
 * \brief Helper class to populate an ISP configuration buffer compatible with
 * the generic V4L2 ISP format
 *
 * The Linux kernel defines a generic buffer format for configuring ISP devices.
 * The format describes a serialisation method for ISP parameters that allows
 * userspace to populate a buffer of configuration data by appending blocks to
 * the buffer with common headers but device-specific contents one after the
 * other.
 *
 * The V4L2Params class implements support the V4L2 ISP parameters buffer format
 * and allows users to populate the ISP configuration blocks, represented as
 * V4L2ParamBlock class instances.
 *
 * IPA implementations using this helpers should define an enumeration of ISP
 * blocks supported by the IPA module and use a set of common abstraction to
 * help their derived implementation of V4L2Params translate the enumerated ISP
 * block identifier to the actual type of the configuration data as defined by
 * the kernel interface.
 *
 * As an example of this see the RkISP1 and Mali-C55 implementations.
 */

/**
 * \class V4L2ParamsBlock
 * \brief Helper class that represents an ISP configuration block
 *
 * Each ISP function is associated with a set of configuration parameters
 * defined by the kernel interface.
 *
 * This class represents an ISP block configuration entry. It is constructed
 * with a reference to the memory area where the block configuration will be
 * stored in the parameters buffer. The template parameter represents
 * the underlying kernel-defined ISP block configuration type and allows its
 * user to easily cast it to said type to populate and read the configuration
 * parameters.
 *
 * \sa V4L2Params::block()
 */

/**
 * \fn V4L2ParamsBlock::V4L2ParamsBlock()
 * \brief Construct a V4L2ParamsBlock with memory represented by \a data
 * \param[in] data The memory area where the ISP block is located
 */

/**
 * \fn V4L2ParamsBlock::setEnabled()
 * \brief Enable/disable an ISP configuration block
 * \param[in] enabled The enable flag
 */

/**
 * \fn V4L2ParamsBlock::operator->()
 * \brief Access the ISP configuration block casting it to the kernel-defined
 * ISP configuration type
 *
 * The V4L2ParamsBlock is templated with the kernel defined ISP configuration
 * block type. This function allows users to easily cast a V4L2ParamsBlock to
 * the underlying kernel-defined type in order to easily populate or read
 * the ISP configuration data.
 *
 * \code{.cpp}
 *
 * // The kernel header defines the ISP configuration types, in example
 * // struct my_isp_awb_config_data {
 * //		u16 gain_ch00;
 * //		u16 gain_ch01;
 * //		u16 gain_ch10;
 * //		u16 gain_ch11;
 * //  }
 *
 * template<> V4L2ParamsBlock<struct my_isp_awb_config_data> awbBlock = ...
 *
 * awbBlock->gain_ch00 = ...;
 * awbBlock->gain_ch01 = ...;
 * awbBlock->gain_ch10 = ...;
 * awbBlock->gain_ch11 = ...;
 *
 * \endcode
 *
 * Users of this class shall not create a V4L2ParamsBlock manually but should
 * use V4L2Params::block().
 */

/**
 * \fn V4L2ParamsBlock::operator->() const
 * \copydoc V4L2ParamsBlock::operator->()
 */

/**
 * \fn V4L2ParamsBlock::operator*() const
 * \copydoc V4L2ParamsBlock::operator->()
 */

/**
 * \fn V4L2ParamsBlock::operator*()
 * \copydoc V4L2ParamsBlock::operator->()
 */

/**
 * \var V4L2ParamsBlock::data_
 * \brief Memory area reserved for the ISP configuration block
 */

/**
 * \class V4L2Params
 * \brief Helper class that represent an ISP configuration buffer
 *
 * This class represents an ISP configuration buffer. It is constructed
 * with a reference to the memory mapped buffer that will be queued to the ISP
 * driver.
 *
 * This class is templated with the type of the enumeration of ISP blocks that
 * each IPA module is expected to support. IPA modules are expected to derive
 * this class by providing a 'param_traits' type that helps the class associate
 * a block type with the actual memory area that represents the ISP
 * configuration block.
 *
 * \code{.cpp}
 *
 * // Define the supported ISP blocks
 * enum class myISPBlocks {
 *	Agc,
 *	Awb,
 *	...
 * };
 *
 * // Maps the C++ enum type to the kernel enum type and concrete parameter type
 * template<myISPBlocks B>
 * struct block_type {
 * };
 *
 * template<>
 * struct block_type<myISPBlock::Agc> {
 *	using type = struct my_isp_kernel_config_type_agc;
 *	static constexpr kernel_enum_type blockType = MY_ISP_TYPE_AGC;
 * };
 *
 * template<>
 * struct block_type<myISPBlock::Awb> {
 *	using type = struct my_isp_kernel_config_type_awb;
 *	static constexpr kernel_enum_type blockType = MY_ISP_TYPE_AWB;
 * };
 *
 *
 * // Convenience type to associate a block id to the 'block_type' overload
 * struct params_traits {
 * 	using id_type = myISPBlocks;
 * 	template<id_type Id> using id_to_details = block_type<Id>;
 * };
 *
 * ...
 *
 * // Derive the V4L2Params class by providing params_traits
 * class MyISPParams : public V4L2Params<params_traits>
 * {
 * public:
 * 	MyISPParams::MyISPParams(Span<uint8_t> data)
 * 		: V4L2Params(data, kVersion)
 * 	{
 * 	}
 * };
 *
 * \endcode
 *
 * Users of this class can then easily access an ISP configuration block as a
 * V4L2ParamsBlock instance.
 *
 * \code{.cpp}
 *
 * MyISPParams params(data);
 *
 * auto awb = params.block<myISPBlocks::AWB>();
 * awb->gain00 = ...;
 * awb->gain01 = ...;
 * awb->gain10 = ...;
 * awb->gain11 = ...;
 * \endcode
 */

/**
 * \fn V4L2Params::V4L2Params()
 * \brief Construct an instance of V4L2Params
 * \param[in] data Reference to the v4l2-buffer memory mapped area
 * \param[in] version The ISP parameters version the implementation supports
 */

/**
 * \fn V4L2Params::bytesused()
 * \brief Retrieve the used size of the parameters buffer (in bytes)
 *
 * The parameters buffer size is mostly used to populate the v4l2_buffer
 * bytesused field before queueing the buffer to the ISP.
 *
 * \return The number of bytes occupied by the ISP configuration parameters
 */

/**
 * \fn V4L2Params::block()
 * \brief Retrieve the location of an ISP configuration block a return it
 * \return A V4L2ParamsBlock instance that points to the ISP configuration block
 */

/**
 * \fn V4L2Params::block(typename Traits::id_type type, unsigned int blockType, size_t blockSize)
 * \brief Populate an ISP configuration block a returns a reference to its
 * memory
 * \param[in] type The ISP block identifier enumerated by the IPA module
 * \param[in] blockType The kernel-defined ISP block identifier, used to
 * populate the block header
 * \param[in] blockSize The ISP block size, used to populate the block header
 *
 * Initialize the block header with \a blockType and \a blockSize and
 * returns a reference to the memory used to store an ISP configuration block.
 *
 * IPA modules that derive the V4L2Params class shall use this function to
 * retrieve the memory area that will be used to construct a V4L2ParamsBlock<T>
 * before returning it to the caller.
 */

/**
 * \var V4L2Params::data_
 * \brief The ISP parameters buffer memory
 */

/**
 * \var V4L2Params::used_
 * \brief The number of bytes used in the parameters buffer
 */

/**
 * \var V4L2Params::blocks_
 * \brief Cache of ISP configuration blocks
 */

} /* namespace ipa */

} /* namespace libcamera */
