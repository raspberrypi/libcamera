#include "pisp_variant.h"

namespace PiSP {

namespace BCM2712 {

class HW final : public PiSPVariant
{
public:
	HW() : PiSPVariant(
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
		)
	{
	}
};

BCM2712::HW BCM2712Variant;

} // namespace BCM2712

} // namespace PiSP
