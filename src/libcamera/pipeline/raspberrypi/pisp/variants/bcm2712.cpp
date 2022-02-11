#include "pisp_variant.h"

namespace PiSP {

const PiSPVariant BCM2712_HW (
	2,					/* numFrontEnds */
	1,					/* numBackEnds */
	{ 2, 2 }, 				/* numFrontEndBranches */
	{ 6144, 6144 },				/* frontEndMaxWidth */
	{{ { true, true }, { true, true } }},	/* frontEndDownscaler */
	{{ { 6144, 4096 }, { 6144, 4096 } }},	/* frontEndDownscalerMaxWidth */
	640,					/* backEndMaxTileWidth */
	{ 2 },					/* numBackEndBranches */
	{ { false, false } },			/* backEndIntegralImage */
	{ { false, true } }			/* backEndDownscalers */
);

} // namespace PiSP
