#include <libcamera/ipa/ipa_module_info.h>

namespace libcamera {

extern "C" {
const struct libcamera::IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	9001,
	"bleep",
	"It's over nine thousand!",
};
};

}; /* namespace libcamera */
