/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 ISP Parameters
 */

#pragma once

#include <map>
#include <stdint.h>

#include <linux/rkisp1-config.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

namespace libcamera {

namespace ipa::rkisp1 {

enum class BlockType {
	Bls,
	Dpcc,
	Sdg,
	AwbGain,
	Flt,
	Bdm,
	Ctk,
	Goc,
	Dpf,
	DpfStrength,
	Cproc,
	Ie,
	Lsc,
	Awb,
	Hst,
	Aec,
	Afc,
	CompandBls,
	CompandExpand,
	CompandCompress,
};

namespace details {

template<BlockType B>
struct block_type {
};

#define RKISP1_DEFINE_BLOCK_TYPE(blockType, blockStruct)		\
template<>								\
struct block_type<BlockType::blockType> {				\
	using type = struct rkisp1_cif_isp_##blockStruct##_config;	\
};

RKISP1_DEFINE_BLOCK_TYPE(Bls, bls)
RKISP1_DEFINE_BLOCK_TYPE(Dpcc, dpcc)
RKISP1_DEFINE_BLOCK_TYPE(Sdg, sdg)
RKISP1_DEFINE_BLOCK_TYPE(AwbGain, awb_gain)
RKISP1_DEFINE_BLOCK_TYPE(Flt, flt)
RKISP1_DEFINE_BLOCK_TYPE(Bdm, bdm)
RKISP1_DEFINE_BLOCK_TYPE(Ctk, ctk)
RKISP1_DEFINE_BLOCK_TYPE(Goc, goc)
RKISP1_DEFINE_BLOCK_TYPE(Dpf, dpf)
RKISP1_DEFINE_BLOCK_TYPE(DpfStrength, dpf_strength)
RKISP1_DEFINE_BLOCK_TYPE(Cproc, cproc)
RKISP1_DEFINE_BLOCK_TYPE(Ie, ie)
RKISP1_DEFINE_BLOCK_TYPE(Lsc, lsc)
RKISP1_DEFINE_BLOCK_TYPE(Awb, awb_meas)
RKISP1_DEFINE_BLOCK_TYPE(Hst, hst)
RKISP1_DEFINE_BLOCK_TYPE(Aec, aec)
RKISP1_DEFINE_BLOCK_TYPE(Afc, afc)
RKISP1_DEFINE_BLOCK_TYPE(CompandBls, compand_bls)
RKISP1_DEFINE_BLOCK_TYPE(CompandExpand, compand_curve)
RKISP1_DEFINE_BLOCK_TYPE(CompandCompress, compand_curve)

} /* namespace details */

class RkISP1Params;

class RkISP1ParamsBlockBase
{
public:
	RkISP1ParamsBlockBase(RkISP1Params *params, BlockType type,
			      const Span<uint8_t> &data);

	Span<uint8_t> data() const { return data_; }

	void setEnabled(bool enabled);

private:
	LIBCAMERA_DISABLE_COPY(RkISP1ParamsBlockBase)

	RkISP1Params *params_;
	BlockType type_;
	Span<uint8_t> header_;
	Span<uint8_t> data_;
};

template<BlockType B>
class RkISP1ParamsBlock : public RkISP1ParamsBlockBase
{
public:
	using Type = typename details::block_type<B>::type;

	RkISP1ParamsBlock(RkISP1Params *params, const Span<uint8_t> &data)
		: RkISP1ParamsBlockBase(params, B, data)
	{
	}

	const Type *operator->() const
	{
		return reinterpret_cast<const Type *>(data().data());
	}

	Type *operator->()
	{
		return reinterpret_cast<Type *>(data().data());
	}

	const Type &operator*() const &
	{
		return *reinterpret_cast<const Type *>(data().data());
	}

	Type &operator*() &
	{
		return *reinterpret_cast<Type *>(data().data());
	}
};

class RkISP1Params
{
public:
	RkISP1Params(uint32_t format, Span<uint8_t> data);

	template<BlockType B>
	RkISP1ParamsBlock<B> block()
	{
		return RkISP1ParamsBlock<B>(this, block(B));
	}

	uint32_t format() const { return format_; }
	size_t size() const { return used_; }

private:
	friend class RkISP1ParamsBlockBase;

	Span<uint8_t> block(BlockType type);
	void setBlockEnabled(BlockType type, bool enabled);

	uint32_t format_;

	Span<uint8_t> data_;
	size_t used_;

	std::map<BlockType, Span<uint8_t>> blocks_;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
