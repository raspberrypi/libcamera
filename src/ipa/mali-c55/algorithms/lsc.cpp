/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * Mali-C55 Lens shading correction algorithm
 */

#include "lsc.h"

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Lsc)

int Lsc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	if (!tuningData.contains("meshScale")) {
		LOG(MaliC55Lsc, Error) << "meshScale missing from tuningData";
		return -EINVAL;
	}

	meshScale_ = tuningData["meshScale"].get<uint32_t>(0);

	const YamlObject &yamlSets = tuningData["sets"];
	if (!yamlSets.isList()) {
		LOG(MaliC55Lsc, Error) << "LSC tables missing or invalid";
		return -EINVAL;
	}

	size_t tableSize = 0;
	const auto &sets = yamlSets.asList();
	for (const auto &yamlSet : sets) {
		uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

		if (!ct) {
			LOG(MaliC55Lsc, Error) << "Invalid colour temperature";
			return -EINVAL;
		}

		if (std::count(colourTemperatures_.begin(),
			       colourTemperatures_.end(), ct)) {
			LOG(MaliC55Lsc, Error)
				<< "Multiple sets found for colour temperature";
			return -EINVAL;
		}

		std::vector<uint8_t> rTable =
			yamlSet["r"].getList<uint8_t>().value_or(std::vector<uint8_t>{});
		std::vector<uint8_t> gTable =
			yamlSet["g"].getList<uint8_t>().value_or(std::vector<uint8_t>{});
		std::vector<uint8_t> bTable =
			yamlSet["b"].getList<uint8_t>().value_or(std::vector<uint8_t>{});

		/*
		 * Some validation to do; only 16x16 and 32x32 tables of
		 * coefficients are acceptable, and all tables across all of the
		 * sets must be the same size. The first time we encounter a
		 * table we check that it is an acceptable size and if so make
		 * sure all other tables are of equal size.
		 */
		if (!tableSize) {
			if (rTable.size() != 256 && rTable.size() != 1024) {
				LOG(MaliC55Lsc, Error)
					<< "Invalid table size for colour temperature " << ct;
				return -EINVAL;
			}
			tableSize = rTable.size();
		}

		if (rTable.size() != tableSize ||
		    gTable.size() != tableSize ||
		    bTable.size() != tableSize) {
			LOG(MaliC55Lsc, Error)
				<< "Invalid or mismatched table size for colour temperature " << ct;
			return -EINVAL;
		}

		if (colourTemperatures_.size() >= 3) {
			LOG(MaliC55Lsc, Error)
				<< "A maximum of 3 colour temperatures are supported";
			return -EINVAL;
		}

		for (unsigned int i = 0; i < tableSize; i++) {
			mesh_[kRedOffset + i] |=
				(rTable[i] << (colourTemperatures_.size() * 8));
			mesh_[kGreenOffset + i] |=
				(gTable[i] << (colourTemperatures_.size() * 8));
			mesh_[kBlueOffset + i] |=
				(bTable[i] << (colourTemperatures_.size() * 8));
		}

		colourTemperatures_.push_back(ct);
	}

	/*
	 * The mesh has either 16x16 or 32x32 nodes, we tell the driver which it
	 * is based on the number of values in the tuning data's table.
	 */
	if (tableSize == 256)
		meshSize_ = 15;
	else
		meshSize_ = 31;

	return 0;
}

void Lsc::fillConfigParamsBlock(MaliC55Params *params) const
{
	auto block = params->block<MaliC55Blocks::MeshShadingConfig>();

	block->mesh_show = false;
	block->mesh_scale = meshScale_;
	block->mesh_page_r = 0;
	block->mesh_page_g = 1;
	block->mesh_page_b = 2;
	block->mesh_width = meshSize_;
	block->mesh_height = meshSize_;

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

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
