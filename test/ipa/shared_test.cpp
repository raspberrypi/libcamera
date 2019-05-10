#include <libcamera/ipa/ipa_module_info.h>

namespace libcamera {

extern "C" {
const struct libcamera::IPAModuleInfo ipaModuleInfo = {
	"It's over nine thousand!",
	9001,
};
};

}; /* namespace libcamera */
