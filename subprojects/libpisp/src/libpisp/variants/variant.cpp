/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * pisp_variantPiSP variant configuration definitions
 */

#include "variant.hpp"

#include <algorithm>

namespace libpisp
{

namespace {

const PiSPVariant variant_error {};

} // namespace

const PiSPVariant BCM2712_C0 {
	"BCM2712_C0",			/* Name */
	0x00114666,				/* FrontEnd version */
	0x02252700,				/* BackEnd version */
	2,						/* Number of FrontEnds */
	1,						/* Number of BackEnds */
	{ 2, 2 },				/* Number of branches per FrontEnd */
	{ 6144, 6144 }, 		/* Maximum statistics width per FrontEnd branch */
	{{ { true, true }, { true, true } }}, /* Availability of downscalers per FrontEnd branch */
	{{ { 6144, 4096 }, { 6144, 4096 } }}, /* Maximum width of downscalers per FrontEnd branch */
	640,					/* Maximum tile size of the BackEnd */
	{ 2 },					/* Number of branches per BackEnd */
	{{ { false, false } }}, /* Availability of integral image output per BackEnd branch */
	{{ { false, true } }},	/* Availability of downscalers per BackEnd branch */
	false,					/* BackEnd RGB32 output format support */
};

const PiSPVariant BCM2712_D0 {
	"BCM2712_D0",			/* Name */
	0x00114666,				/* FrontEnd version */
	0x02252701,				/* BackEnd version */
	2,						/* Number of FrontEnds */
	1,						/* Number of BackEnds */
	{ 2, 2 },				/* Number of branches per FrontEnd */
	{ 6144, 6144 }, 		/* Maximum statistics width per FrontEnd branch */
	{{ { true, true }, { true, true } }}, /* Availability of downscalers per FrontEnd branch */
	{{ { 6144, 4096 }, { 6144, 4096 } }}, /* Maximum width of downscalers per FrontEnd branch */
	640,					/* Maximum tile size of the BackEnd */
	{ 2 },					/* Number of branches per BackEnd */
	{{ { false, false } }}, /* Availability of integral image output per BackEnd branch */
	{{ { false, true } }},	/* Availability of downscalers per BackEnd branch */
	true,					/* BackEnd RGB32 output format support */
};

const std::vector<PiSPVariant> &get_variants()
{
	static const std::vector<PiSPVariant> variants {
		BCM2712_C0,
		BCM2712_D0
	};

	return variants;
}

const PiSPVariant &get_variant(unsigned int fe_version, unsigned int be_version)
{
	const std::vector<PiSPVariant> &variants = get_variants();

	auto it = std::find_if(variants.begin(), variants.end(),
				   [fe_version, be_version](const auto &hw) { return hw.FrontEndVersion() == fe_version &&
																	 hw.BackEndVersion() == be_version; });

	if (it == variants.end())
		return variant_error;

	return *it;
}

} // namespace libpisp
