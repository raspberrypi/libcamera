/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * Table-based LSC implementation
 */

#include "lsc_table.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(LscTable)

namespace ipa {

/**
 * \class LscTable
 * \brief Table based LSC algorithm implementation
 *
 * Table based LSC algorithm implementation. The LSCTable class implements LSC
 * support using tabular LSC data.
 *
 * \sa LscImplementation
 */

/**
 * \brief Parse tabular LSC data
 * \param[in] sets The tuning file content
 * \param[in] descriptor The LSC engine descriptor
 *
 * Parse the LSC data in tabular form from the \a sets tuning data.
 *
 * \todo Currently the gain values parsed from tuning file are expressed in the
 * platform fixed-point register format. Use gains in floating point
 * representation for tabular LSC data in order to facilitate re-use of LSC
 * tuning data across different platforms.
 *
 * \return 0 on success or a negative error number otherwise
 */
int LscTable::parseLscData(const ValueNode &sets,
			   const LscDescriptor &descriptor)
{
	for (const auto &set : sets.asList()) {
		uint32_t ct = set["ct"].get<uint32_t>(0);

		int ret = parseLscComponent(set, ct, descriptor);
		if (ret)
			return ret;
	}

	if (lscData_.empty()) {
		LOG(LscTable, Error) << "Failed to load any sets";
		return -EINVAL;
	}

	return 0;
}

int LscTable::parseLscComponent(const ValueNode &yamlSet,
				unsigned int ct, const LscDescriptor &descriptor)
{
	LscImplementation::Components component;
	for (auto &k : descriptor.keys) {
		auto [it, inserted] =
			component.try_emplace(k, parseTable(yamlSet,
							    k.c_str(),
							    descriptor.numHSamples,
							    descriptor.numVSamples));
		ASSERT(inserted);

		if (it->second.empty()) {
			LOG(LscTable, Error)
				<< "Set " << k << " for color temperature "
				<< ct << " is missing";
			return -EINVAL;
		}
	}

	auto [it, inserted] = lscData_.try_emplace(ct, std::move(component));
	if (!inserted) {
		LOG(LscTable, Error)
			<< "Multiple sets found for color temperature "
			<< ct;
		return -EINVAL;
	}

	return 0;
}

std::vector<float> LscTable::parseTable(const ValueNode &tuningData,
					const char *prop,
					unsigned int numHSamples,
					unsigned int numVSamples)
{
	unsigned int lscNumSamples = numHSamples * numVSamples;

	/*
	 * Cast to float even if gains are expressed as fixed-point
	 * representations. This prepares to express gains in floating point
	 * formats in tuning files.
	 */
	std::vector<float> table =
		tuningData[prop].get<std::vector<float>>().value_or(utils::defopt);
	if (table.size() != lscNumSamples) {
		LOG(LscTable, Error)
			<< "Invalid '" << prop << "' values: expected "
			<< lscNumSamples
			<< " elements, got " << table.size();
		return {};
	}

	return table;
}

} /* namespace ipa */

} /* namespace libcamera */
