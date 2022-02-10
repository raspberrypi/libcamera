#pragma once

// Some data structures and an API function to make it easier to tile up and image for a PiSP-like pipeline.

#include <iostream>

#include "types.hpp"

namespace PiSP {

const int NUM_OUTPUT_BRANCHES = 2;

struct Tile {
	tiling::Region input;
	tiling::Region decompress;
	tiling::Region context;
	tiling::Region crop;
	tiling::Region downscale[NUM_OUTPUT_BRANCHES];
	tiling::Region resample[NUM_OUTPUT_BRANCHES];
	tiling::Region hog;
	tiling::Region output[NUM_OUTPUT_BRANCHES];
};

struct TilingConfig {
	tiling::Length2 input_image_size;
	tiling::Interval2 crop;
	tiling::Length2 downscale_image_size[NUM_OUTPUT_BRANCHES];
	tiling::Length2 output_image_size[NUM_OUTPUT_BRANCHES];
	tiling::Length2 max_tile_size;
	tiling::Length2 min_tile_size;
	tiling::Length2 downscale_factor[NUM_OUTPUT_BRANCHES];
	tiling::Length2 resample_factor[NUM_OUTPUT_BRANCHES];
	bool output_h_mirror[NUM_OUTPUT_BRANCHES];
	int resample_enables;
	int downscale_enables;
	bool compressed_input;
	tiling::Length2 input_alignment;
	tiling::Length2 output_max_alignment[NUM_OUTPUT_BRANCHES]; // "preferred" alignment
	tiling::Length2 output_min_alignment[NUM_OUTPUT_BRANCHES]; // "required" minimum alignment
};
inline std::ostream &operator<<(std::ostream &os, TilingConfig const &tc)
{
	os << "TilingConfig:" << std::endl;
	os << "\tinput_image_size " << tc.input_image_size << " align " << tc.input_alignment << std::endl;
	os << "\tcrop " << tc.crop << std::endl;
	for (int i = 0; i < NUM_OUTPUT_BRANCHES; i++) {
		os << "\toutput_image_size[" << i << "] " << tc.output_image_size[i] << " align max " << tc.output_max_alignment[i] << " min " << tc.output_min_alignment[i] << std::endl;
		os << "\tdownscale_image_size " << tc.downscale_image_size[i] << " downscale_factor " << tc.downscale_factor[i] << " resample_factor " << tc.resample_factor[i] << std::endl;
	}
	os << "\tenables resample " << tc.resample_enables << " downscale " << tc.downscale_enables << std::endl;
	return os << "\tmax_tile_size " << tc.max_tile_size;
}

void tile_pipeline(TilingConfig const &config, Tile *tile, int num_tile, tiling::Length2 *grid);

} // namespace PiSP