#pragma once

#include <cstddef>

#include "pisp_common.h"

namespace PiSP {

extern void compute_stride(pisp_image_format_config &config);
extern void compute_stride_align(pisp_image_format_config &config, int align);
extern void compute_addr_offset(const pisp_image_format_config &config, int x, int y,
				uint32_t *addr_offset, uint32_t *addr_offset2);
extern int num_planes(pisp_image_format format);
extern std::size_t get_plane_size(const pisp_image_format_config &config, int plane);

} // namespace PiSP