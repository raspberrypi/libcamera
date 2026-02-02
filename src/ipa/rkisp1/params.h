/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 ISP Parameters
 */

#pragma once

#include <linux/rkisp1-config.h>
#include <linux/videodev2.h>

#include <libipa/v4l2_params.h>

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
	Wdr,
};

namespace details {

template<BlockType B>
struct block_type {
};

#define RKISP1_DEFINE_BLOCK_TYPE(blockType, blockStruct, id)		\
template<>								\
struct block_type<BlockType::blockType> {				\
	using type = struct rkisp1_cif_isp_##blockStruct##_config;	\
	static constexpr rkisp1_ext_params_block_type blockType =	\
		RKISP1_EXT_PARAMS_BLOCK_TYPE_##id;			\
};

RKISP1_DEFINE_BLOCK_TYPE(Bls, bls, BLS)
RKISP1_DEFINE_BLOCK_TYPE(Dpcc, dpcc, DPCC)
RKISP1_DEFINE_BLOCK_TYPE(Sdg, sdg, SDG)
RKISP1_DEFINE_BLOCK_TYPE(AwbGain, awb_gain, AWB_GAIN)
RKISP1_DEFINE_BLOCK_TYPE(Flt, flt, FLT)
RKISP1_DEFINE_BLOCK_TYPE(Bdm, bdm, BDM)
RKISP1_DEFINE_BLOCK_TYPE(Ctk, ctk, CTK)
RKISP1_DEFINE_BLOCK_TYPE(Goc, goc, GOC)
RKISP1_DEFINE_BLOCK_TYPE(Dpf, dpf, DPF)
RKISP1_DEFINE_BLOCK_TYPE(DpfStrength, dpf_strength, DPF_STRENGTH)
RKISP1_DEFINE_BLOCK_TYPE(Cproc, cproc, CPROC)
RKISP1_DEFINE_BLOCK_TYPE(Ie, ie, IE)
RKISP1_DEFINE_BLOCK_TYPE(Lsc, lsc, LSC)
RKISP1_DEFINE_BLOCK_TYPE(Awb, awb_meas, AWB_MEAS)
RKISP1_DEFINE_BLOCK_TYPE(Hst, hst, HST_MEAS)
RKISP1_DEFINE_BLOCK_TYPE(Aec, aec, AEC_MEAS)
RKISP1_DEFINE_BLOCK_TYPE(Afc, afc, AFC_MEAS)
RKISP1_DEFINE_BLOCK_TYPE(CompandBls, compand_bls, COMPAND_BLS)
RKISP1_DEFINE_BLOCK_TYPE(CompandExpand, compand_curve, COMPAND_EXPAND)
RKISP1_DEFINE_BLOCK_TYPE(CompandCompress, compand_curve, COMPAND_COMPRESS)
RKISP1_DEFINE_BLOCK_TYPE(Wdr, wdr, WDR)

struct params_traits {
	using id_type = BlockType;

	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace details */

template<typename T>
class RkISP1ParamsBlock;

class RkISP1Params : public V4L2Params<details::params_traits>
{
public:
	static constexpr unsigned int kVersion = RKISP1_EXT_PARAM_BUFFER_V1;

	RkISP1Params(uint32_t format, Span<uint8_t> data)
		: V4L2Params(data, kVersion), format_(format)
	{
		if (format_ == V4L2_META_FMT_RK_ISP1_PARAMS) {
			memset(data.data(), 0, data.size());
			used_ = sizeof(struct rkisp1_params_cfg);
		}
	}

	template<details::params_traits::id_type id>
	auto block()
	{
		using Type = typename details::block_type<id>::type;

		return RkISP1ParamsBlock<Type>(this, id, block(id));
	}

	uint32_t format() const { return format_; }
	void setBlockEnabled(BlockType type, bool enabled);

private:
	Span<uint8_t> block(BlockType type);

	uint32_t format_;
};

template<typename T>
class RkISP1ParamsBlock final : public V4L2ParamsBlock<T>
{
public:
	RkISP1ParamsBlock(RkISP1Params *params, BlockType type,
			  const Span<uint8_t> data)
		: V4L2ParamsBlock<T>(data)
	{
		params_ = params;
		type_ = type;

		/*
		 * cifData_ points to the actual configuration data
		 * (struct rkisp1_cif_isp_*) which is not prefixed by any header,
		 * for the legacy fixed format.
		 */
		if (params_->format() == V4L2_META_FMT_RK_ISP1_PARAMS)
			cifData_ = data;
		else
			cifData_ = data.subspan(sizeof(v4l2_isp_params_block_header));
	}

	void setEnabled(bool enabled) override
	{
		/*
		 * For the legacy fixed format, blocks are enabled in the
		 * top-level header. Delegate to the RkISP1Params class.
		 */
		if (params_->format() == V4L2_META_FMT_RK_ISP1_PARAMS)
			return params_->setBlockEnabled(type_, enabled);

		return V4L2ParamsBlock<T>::setEnabled(enabled);
	}

	/*
	 * Override the dereference operators to return a reference to the
	 * actual configuration data (struct rkisp1_cif_isp_*) skipping the
	 * 'v4l2_isp_params_block_header' header.
	 */

	virtual const T *operator->() const override
	{
		return reinterpret_cast<const T *>(cifData_.data());
	}

	virtual T *operator->() override
	{
		return reinterpret_cast<T *>(cifData_.data());
	}

	virtual const T &operator*() const override
	{
		return *reinterpret_cast<const T *>(cifData_.data());
	}

	virtual T &operator*() override
	{
		return *reinterpret_cast<T *>(cifData_.data());
	}

private:
	RkISP1Params *params_;
	BlockType type_;
	Span<uint8_t> cifData_;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
