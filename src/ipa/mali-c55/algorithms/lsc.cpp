/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * Mali-C55 Lens shading correction algorithm
 */

#include "lsc.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

namespace libcamera {

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Lsc)

/* Gain values in [1, 5] range */
static constexpr unsigned int kMeshScale = 6;

/* Mali-C55 hw supports configurable mesh sizes; we fix it to 32. */
static constexpr unsigned int kMeshSize = 32;
static constexpr unsigned int kGridSize = kMeshSize * kMeshSize;

/* Per-colour component page offsets in the mesh table. */
static constexpr unsigned int kRedOffset = 0;
static constexpr unsigned int kGreenOffset = 1024;
static constexpr unsigned int kBlueOffset = 2048;

/*
 * \todo Clarify if Mali-C55 can support up to 4 colour temperatures.
 *
 * The uAPI only expose MALI_C55_NUM_MESH_SHADING_ELEMENTS (3072) gain elements,
 * which correspond to three pages of 1024 (32x32) entries.
 */
static constexpr unsigned int kMaxColourTemperatures = 3;

/*
 * The LSC algorithm implementation only supports 32x32 grids. Create a list of
 * positions from the grid size.
 */
std::vector<double> Lsc::segmentsToPosition() const
{
	std::vector<double> positions(kMeshSize);
	for (double i = 0.0; i < kMeshSize; ++i)
		positions[i] = i / (kMeshSize - 1);

	return positions;
}

int Lsc::init(IPAContext &context, const ValueNode &tuningData)
{
	gridPos_ = segmentsToPosition();

	return lscAlgo_.init(tuningData, context.ctrlMap, {
				.keys = { "r", "g", "b" },
				.numHSamples = kMeshSize,
				.numVSamples = kMeshSize,
				.sensorSize = context.sensorInfo.activeAreaSize
			     });
}

int Lsc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	int ret = lscAlgo_.configure(context.activeState.lsc, configInfo.analogCrop,
				     gridPos_, gridPos_);
	if (ret)
		return ret;

	/* Re-initialize the mesh tables and reserve space for enough entries. */
	mesh_ = std::vector<uint32_t>(kGridSize * kMaxColourTemperatures);
	colourTemperatures_.clear();

	/*
	 * Get the lsc tables per colour components and populate mesh_ with
	 * their content.
	 */
	auto &components = lscAlgo_.getComponents();
	for (auto const &[ct, component] : components) {
		if (std::count(colourTemperatures_.begin(),
			       colourTemperatures_.end(), ct)) {
			LOG(MaliC55Lsc, Error)
				<< "Multiple sets found for colour temperature";
			return -EINVAL;
		}

		const std::vector<uint8_t> &rTable = component.at("r");
		const std::vector<uint8_t> &gTable = component.at("g");
		const std::vector<uint8_t> &bTable = component.at("b");

		/* Only 32x32 tables of coefficients are accepted. */
		ASSERT(rTable.size() == kGridSize &&
		       gTable.size() == kGridSize &&
		       bTable.size() != kGridSize);

		if (colourTemperatures_.size() >= kMaxColourTemperatures) {
			LOG(MaliC55Lsc, Error)
				<< "A maximum of 3 colour temperatures are supported";
			return -EINVAL;
		}

		/*
		 * Create the mesh table entries by assembling up to 3 gains per
		 * colour temperature in a u32 word.
		 */
		for (unsigned int i = 0; i < kGridSize; i++) {
			mesh_[kRedOffset + i] |=
				(rTable[i] << (colourTemperatures_.size() * 8));
			mesh_[kGreenOffset + i] |=
				(gTable[i] << (colourTemperatures_.size() * 8));
			mesh_[kBlueOffset + i] |=
				(bTable[i] << (colourTemperatures_.size() * 8));
		}

		colourTemperatures_.push_back(ct);
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Lsc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext, const ControlList &controls)
{
	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc,
			      controls);
}

void Lsc::fillConfigParamsBlock(MaliC55Params *params) const
{
	auto block = params->block<MaliC55Blocks::MeshShadingConfig>();

	block->mesh_show = false;
	block->mesh_scale = kMeshScale;
	block->mesh_page_r = 0;
	block->mesh_page_g = 1;
	block->mesh_page_b = 2;
	block->mesh_width = kMeshSize - 1;
	block->mesh_height = kMeshSize - 1;

	std::copy(mesh_.begin(), mesh_.end(), block->mesh);
}

void Lsc::fillSelectionParamsBlock(MaliC55Params *params, uint8_t bank,
				   uint8_t alpha) const
{
	auto block = params->block<MaliC55Blocks::MeshShadingSel>();

	block->mesh_alpha_bank_r = bank;
	block->mesh_alpha_bank_g = bank;
	block->mesh_alpha_bank_b = bank;
	block->mesh_alpha_r = alpha;
	block->mesh_alpha_g = alpha;
	block->mesh_alpha_b = alpha;
	block->mesh_strength = 0x1000; /* Otherwise known as 1.0 */
}

std::tuple<uint8_t, uint8_t> Lsc::findBankAndAlpha(uint32_t ct) const
{
	unsigned int i;

	ct = std::clamp<uint32_t>(ct, colourTemperatures_.front(),
				  colourTemperatures_.back());

	for (i = 0; i < colourTemperatures_.size() - 1; i++) {
		if (ct >= colourTemperatures_[i] &&
		    ct <= colourTemperatures_[i + 1])
			break;
	}

	/*
	 * With the clamping, we're guaranteed an index into colourTemperatures_
	 * that's <= colourTemperatures_.size() - 1.
	 */
	uint8_t alpha = (255 * (ct - colourTemperatures_[i])) /
			(colourTemperatures_[i + 1] - colourTemperatures_[i]);

	return { i, alpha };
}

void Lsc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  MaliC55Params *params)
{
	/*
	 * For each frame we assess the colour temperature of the **last** frame
	 * and then select an appropriately blended table of coefficients based
	 * on that ct. As a bit of a shortcut, if we've only a single table the
	 * handling is somewhat simpler; if it's the first frame we just select
	 * that table and if we're past the first frame then we can just do
	 * nothing - the config will never change.
	 */
	uint32_t temperatureK = context.activeState.agc.temperatureK;
	uint8_t bank, alpha;

	if (colourTemperatures_.size() == 1) {
		if (frame > 0)
			return;

		bank = 0;
		alpha = 0;
	} else {
		std::tie(bank, alpha) = findBankAndAlpha(temperatureK);
	}

	fillSelectionParamsBlock(params, bank, alpha);

	if (frame > 0)
		return;

	/*
	 * If this is the first frame, we need to load the parsed coefficient
	 * tables from tuning data to the ISP.
	 */
	fillConfigParamsBlock(params);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Lsc::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const mali_c55_stats_buffer *stats,
		  ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
