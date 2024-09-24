/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 ISP Parameters
 */

#include "params.h"

#include <map>
#include <stddef.h>
#include <string.h>

#include <linux/rkisp1-config.h>
#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(RkISP1Params)

namespace ipa::rkisp1 {

namespace {

struct BlockTypeInfo {
	enum rkisp1_ext_params_block_type type;
	size_t size;
	size_t offset;
	uint32_t enableBit;
};

#define RKISP1_BLOCK_TYPE_ENTRY(block, id, type, category, bit)			\
	{ BlockType::block, {							\
		RKISP1_EXT_PARAMS_BLOCK_TYPE_##id,				\
		sizeof(struct rkisp1_cif_isp_##type##_config),			\
		offsetof(struct rkisp1_params_cfg, category.type##_config),	\
		RKISP1_CIF_ISP_MODULE_##bit,					\
	} }

#define RKISP1_BLOCK_TYPE_ENTRY_MEAS(block, id, type)				\
	RKISP1_BLOCK_TYPE_ENTRY(block, id##_MEAS, type, meas, id)

#define RKISP1_BLOCK_TYPE_ENTRY_OTHERS(block, id, type)				\
	RKISP1_BLOCK_TYPE_ENTRY(block, id, type, others, id)

#define RKISP1_BLOCK_TYPE_ENTRY_EXT(block, id, type)				\
	{ BlockType::block, {							\
		RKISP1_EXT_PARAMS_BLOCK_TYPE_##id,				\
		sizeof(struct rkisp1_cif_isp_##type##_config),			\
		0, 0,								\
	} }

const std::map<BlockType, BlockTypeInfo> kBlockTypeInfo = {
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Bls, BLS, bls),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Dpcc, DPCC, dpcc),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Sdg, SDG, sdg),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(AwbGain, AWB_GAIN, awb_gain),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Flt, FLT, flt),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Bdm, BDM, bdm),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Ctk, CTK, ctk),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Goc, GOC, goc),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Dpf, DPF, dpf),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(DpfStrength, DPF_STRENGTH, dpf_strength),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Cproc, CPROC, cproc),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Ie, IE, ie),
	RKISP1_BLOCK_TYPE_ENTRY_OTHERS(Lsc, LSC, lsc),
	RKISP1_BLOCK_TYPE_ENTRY_MEAS(Awb, AWB, awb_meas),
	RKISP1_BLOCK_TYPE_ENTRY_MEAS(Hst, HST, hst),
	RKISP1_BLOCK_TYPE_ENTRY_MEAS(Aec, AEC, aec),
	RKISP1_BLOCK_TYPE_ENTRY_MEAS(Afc, AFC, afc),
	RKISP1_BLOCK_TYPE_ENTRY_EXT(CompandBls, COMPAND_BLS, compand_bls),
	RKISP1_BLOCK_TYPE_ENTRY_EXT(CompandExpand, COMPAND_EXPAND, compand_curve),
	RKISP1_BLOCK_TYPE_ENTRY_EXT(CompandCompress, COMPAND_COMPRESS, compand_curve),
};

} /* namespace */

RkISP1ParamsBlockBase::RkISP1ParamsBlockBase(RkISP1Params *params, BlockType type,
					     const Span<uint8_t> &data)
	: params_(params), type_(type)
{
	if (params_->format() == V4L2_META_FMT_RK_ISP1_EXT_PARAMS) {
		header_ = data.subspan(0, sizeof(rkisp1_ext_params_block_header));
		data_ = data.subspan(sizeof(rkisp1_ext_params_block_header));
	} else {
		data_ = data;
	}
}

void RkISP1ParamsBlockBase::setEnabled(bool enabled)
{
	/*
	 * For the legacy fixed format, blocks are enabled in the top-level
	 * header. Delegate to the RkISP1Params class.
	 */
	if (params_->format() == V4L2_META_FMT_RK_ISP1_PARAMS)
		return params_->setBlockEnabled(type_, enabled);

	/*
	 * For the extensible format, set the enable and disable flags in the
	 * block header directly.
	 */
	struct rkisp1_ext_params_block_header *header =
		reinterpret_cast<struct rkisp1_ext_params_block_header *>(header_.data());
	header->flags &= ~(RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE |
			   RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE);
	header->flags |= enabled ? RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE
				 : RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE;
}

RkISP1Params::RkISP1Params(uint32_t format, Span<uint8_t> data)
	: format_(format), data_(data), used_(0)
{
	if (format_ == V4L2_META_FMT_RK_ISP1_EXT_PARAMS) {
		struct rkisp1_ext_params_cfg *cfg =
			reinterpret_cast<struct rkisp1_ext_params_cfg *>(data.data());

		cfg->version = RKISP1_EXT_PARAM_BUFFER_V1;
		cfg->data_size = 0;

		used_ += offsetof(struct rkisp1_ext_params_cfg, data);
	} else {
		memset(data.data(), 0, data.size());
		used_ = sizeof(struct rkisp1_params_cfg);
	}
}

void RkISP1Params::setBlockEnabled(BlockType type, bool enabled)
{
	const BlockTypeInfo &info = kBlockTypeInfo.at(type);

	struct rkisp1_params_cfg *cfg =
		reinterpret_cast<struct rkisp1_params_cfg *>(data_.data());
	if (enabled)
		cfg->module_ens |= info.enableBit;
	else
		cfg->module_ens &= ~info.enableBit;
}

Span<uint8_t> RkISP1Params::block(BlockType type)
{
	auto infoIt = kBlockTypeInfo.find(type);
	if (infoIt == kBlockTypeInfo.end()) {
		LOG(RkISP1Params, Error)
			<< "Invalid parameters block type "
			<< utils::to_underlying(type);
		return {};
	}

	const BlockTypeInfo &info = infoIt->second;

	/*
	 * For the legacy format, return a block referencing the fixed location
	 * of the data.
	 */
	if (format_ == V4L2_META_FMT_RK_ISP1_PARAMS) {
		/*
		 * Blocks available only in extended parameters have an offset
		 * of 0. Return nullptr in that case.
		 */
		if (info.offset == 0) {
			LOG(RkISP1Params, Error)
				<< "Block type " << utils::to_underlying(type)
				<< " unavailable in fixed parameters format";
			return {};
		}

		struct rkisp1_params_cfg *cfg =
			reinterpret_cast<struct rkisp1_params_cfg *>(data_.data());

		cfg->module_cfg_update |= info.enableBit;
		cfg->module_en_update |= info.enableBit;

		return data_.subspan(info.offset, info.size);
	}

	/*
	 * For the extensible format, allocate memory for the block, including
	 * the header. Look up the block in the cache first. If an algorithm
	 * requests the same block type twice, it should get the same block.
	 */
	auto cacheIt = blocks_.find(type);
	if (cacheIt != blocks_.end())
		return cacheIt->second;

	/* Make sure we don't run out of space. */
	size_t size = sizeof(struct rkisp1_ext_params_block_header)
		    + ((info.size + 7) & ~7);
	if (size > data_.size() - used_) {
		LOG(RkISP1Params, Error)
			<< "Out of memory to allocate block type "
			<< utils::to_underlying(type);
		return {};
	}

	/* Allocate a new block, clear its memory, and initialize its header. */
	Span<uint8_t> block = data_.subspan(used_, size);
	used_ += size;

	struct rkisp1_ext_params_cfg *cfg =
		reinterpret_cast<struct rkisp1_ext_params_cfg *>(data_.data());
	cfg->data_size += size;

	memset(block.data(), 0, block.size());

	struct rkisp1_ext_params_block_header *header =
		reinterpret_cast<struct rkisp1_ext_params_block_header *>(block.data());
	header->type = info.type;
	header->size = block.size();

	/* Update the cache. */
	blocks_[type] = block;

	return block;
}

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */
