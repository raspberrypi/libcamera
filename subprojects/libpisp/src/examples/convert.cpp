
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025 Raspberry Pi Ltd
 *
 * convert.cpp - libpisp simple image converter example
 */

#include <algorithm>
#include <array>
#include <assert.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <stdint.h>
#include <string>

#include <linux/media.h>

#include <cxxopts.hpp>

#include "helpers/backend_device.hpp"
#include "helpers/media_device.hpp"

#include "libpisp/backend/backend.hpp"
#include "libpisp/common/logging.hpp"
#include "libpisp/common/utils.hpp"
#include "libpisp/variants/variant.hpp"

void read_plane(uint8_t *mem, std::ifstream &in, unsigned int width, unsigned int height, unsigned int file_stride,
				unsigned int buffer_stride)
{
	width = std::min(width, file_stride);

	for (unsigned int y = 0; y < height; y++)
	{
		in.read((char *)mem + y * buffer_stride, width);
		in.seekg(file_stride - width, std::ios_base::cur);
	}
}

void write_plane(std::ofstream &out, uint8_t *mem, unsigned int width, unsigned int height, unsigned int file_stride,
				 unsigned int buffer_stride)
{
	width = std::min(width, file_stride);

	for (unsigned int y = 0; y < height; y++)
	{
		out.write((char *)mem + y * buffer_stride, width);
		for (unsigned int i = 0; i < file_stride - width; i++)
			out.put(0);
	}
}

void read_rgb888(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
				 unsigned int file_stride, unsigned int buffer_stride)
{
	read_plane((uint8_t *)mem[0], in, width * 3, height, file_stride, buffer_stride);
}

void write_rgb888(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
				  unsigned int file_stride, unsigned int buffer_stride)
{
	write_plane(out, (uint8_t *)mem[0], width * 3, height, file_stride, buffer_stride);
}

void read_32(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
			 unsigned int file_stride, unsigned int buffer_stride)
{
	read_plane((uint8_t *)mem[0], in, width * 4, height, file_stride, buffer_stride);
}

void write_32(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
			  unsigned int file_stride, unsigned int buffer_stride)
{
	write_plane(out, (uint8_t *)mem[0], width * 4, height, file_stride, buffer_stride);
}

void read_yuv(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
			  unsigned int file_stride, unsigned int buffer_stride, unsigned int ss_x, unsigned int ss_y)
{
	uint8_t *dst = mem[0];
	// Y
	read_plane(dst, in, width, height, file_stride, buffer_stride);
	// U
	dst = mem[1] ? mem[1] : dst + buffer_stride * height;
	read_plane(dst, in, width / ss_x, height / ss_y, file_stride / ss_x, buffer_stride / ss_x);
	// V
	dst = mem[2] ? mem[2] : dst + buffer_stride / ss_x * height / ss_y;
	read_plane(dst, in, width / ss_x, height / ss_y, file_stride / ss_x, buffer_stride / ss_x);
}

void write_yuv(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
			   unsigned int file_stride, unsigned int buffer_stride, unsigned int ss_x, unsigned int ss_y)
{
	uint8_t *src = mem[0];
	// Y
	write_plane(out, src, width, height, file_stride, buffer_stride);
	// U
	src = mem[1] ? mem[1] : src + buffer_stride * height;
	write_plane(out, src, width / ss_x, height / ss_y, file_stride / ss_x, buffer_stride / ss_x);
	// V
	src = mem[2] ? mem[2] : src + buffer_stride / ss_x * height / ss_y;
	write_plane(out, src, width / ss_x, height / ss_y, file_stride / ss_x, buffer_stride / ss_x);
}

void read_yuv420(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
				 unsigned int file_stride, unsigned int buffer_stride)
{
	read_yuv(mem, in, width, height, file_stride, buffer_stride, 2, 2);
}

void read_yuv422p(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
				  unsigned int file_stride, unsigned int buffer_stride)
{
	read_yuv(mem, in, width, height, file_stride, buffer_stride, 2, 1);
}

void read_yuv444p(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
				  unsigned int file_stride, unsigned int buffer_stride)
{
	read_yuv(mem, in, width, height, file_stride, buffer_stride, 1, 1);
}

void read_yuv422i(std::array<uint8_t *, 3> &mem, std::ifstream &in, unsigned int width, unsigned int height,
				  unsigned int file_stride, unsigned int buffer_stride)
{
	read_plane(mem[0], in, width * 2, height, file_stride, buffer_stride);
}

void write_yuv420(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
				  unsigned int file_stride, unsigned int buffer_stride)
{
	write_yuv(out, mem, width, height, file_stride, buffer_stride, 2, 2);
}

void write_yuv422p(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
				   unsigned int file_stride, unsigned int buffer_stride)
{
	write_yuv(out, mem, width, height, file_stride, buffer_stride, 2, 1);
}

void write_yuv444p(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
				   unsigned int file_stride, unsigned int buffer_stride)
{
	write_yuv(out, mem, width, height, file_stride, buffer_stride, 1, 1);
}

void write_yuv422i(std::ofstream &out, std::array<uint8_t *, 3> &mem, unsigned int width, unsigned int height,
				   unsigned int file_stride, unsigned int buffer_stride)
{
	write_plane(out, mem[0], width * 2, height, file_stride, buffer_stride);
}

struct FormatFuncs
{
	std::function<void(std::array<uint8_t *, 3> &, std::ifstream &, unsigned int, unsigned int, unsigned int,
					   unsigned int)> read_file;
	std::function<void(std::ofstream &, std::array<uint8_t *, 3> &, unsigned int, unsigned int, unsigned int,
					   unsigned int)> write_file;
};

const std::map<std::string, FormatFuncs> Formats =
{
	{ "RGB888", { read_rgb888, write_rgb888 } },
	{ "RGBX8888", { read_32, write_32 } },
	{ "YUV420P", { read_yuv420, write_yuv420 } },
	{ "YUV422P", { read_yuv422p, write_yuv422p } },
	{ "YUV444P", { read_yuv444p, write_yuv444p } },
	{ "YUYV", { read_yuv422i, write_yuv422i } },
	{ "UYVY", { read_yuv422i, write_yuv422i } },
};

struct Format
{
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	std::string format;
};

Format parse_format(const std::string &fmt)
{
	Format format;
	size_t pos = 0, start = 0;

	pos = fmt.find(':', start);
	if (pos == std::string::npos)
		return {};
	format.width = std::stoi(fmt.substr(start, pos - start));
	start = pos + 1;

	pos = fmt.find(':', start);
	if (pos == std::string::npos)
		return {};
	format.height = std::stoi(fmt.substr(start, pos - start));
	start = pos + 1;

	pos = fmt.find(':', start);
	if (pos == std::string::npos)
		return {};
	format.stride = std::stoi(fmt.substr(start, pos - start));
	start = pos + 1;

	format.format = fmt.substr(start);

	return format;
}

int main(int argc, char *argv[])
{
	libpisp::helpers::MediaDevice devices;

	libpisp::logging_init();

	cxxopts::Options options(argv[0], "PiSP Image Converter");

	options.add_options()
		("input", "Input file", cxxopts::value<std::string>())
		("output", "Output file", cxxopts::value<std::string>())
		("input-format", "Input format in the form width:height:stride:format\n"
						 "Bit-depth is assumed to be 8-bit.",cxxopts::value<std::string>()->default_value(""))
		("output-format", "Output format in the form width:height:stride:format\n"
						  "Bit-depth is assumed to be 8-bit.", cxxopts::value<std::string>()->default_value(""))
		("f,formats", "List available format strings that can be used")
		("l,list", "Enumerate the media device nodes")
		("h,help", "Print usage")
	;

	options.parse_positional({ "input", "output" });
	options.positional_help("<input file> <output file>");
	options.set_width(120);

	auto args = options.parse(argc, argv);

	if (args.count("help"))
	{
		std::cerr << options.help() << std::endl;
		exit(0);
	}
	else if (args.count("list"))
	{
		std::cerr << devices.List() << std::endl;
		exit(0);
	}
	else if (args.count("formats"))
	{
		for (const auto &f : Formats)
			std::cerr << f.first << " ";
		std::cerr << std::endl;
		exit(0);
	}

	std::string media_dev = devices.Acquire();
	if (media_dev.empty())
	{
		std::cerr << "Unable to acquire any pisp_be device!" << std::endl;
		exit(-1);
	}

	libpisp::helpers::BackendDevice backend_device { media_dev };
	std::cerr << "Acquired device " << media_dev << std::endl;

	auto in_file = parse_format(args["input-format"].as<std::string>());
	if (!Formats.count(in_file.format))
	{
		std::cerr << "Invalid input-format specified" << std::endl;
		exit(-1);
	}

	auto out_file = parse_format(args["output-format"].as<std::string>());
	if (!Formats.count(out_file.format))
	{
		std::cerr << "Invalid output-format specified" << std::endl;
		exit(-1);
	}

	const std::vector<libpisp::PiSPVariant> &variants = libpisp::get_variants();
	const media_device_info info = devices.DeviceInfo(media_dev);
	auto variant = std::find_if(variants.begin(), variants.end(),
								[&info](const auto &v) { return v.BackEndVersion() == info.hw_revision; });
	if (variant == variants.end())
	{
		std::cerr << "Backend hardware cound not be identified: " << info.hw_revision << std::endl;
		exit(-1);
	}

	libpisp::BackEnd be(libpisp::BackEnd::Config({}), *variant);

	pisp_be_global_config global;
	be.GetGlobal(global);
	global.bayer_enables = 0;
	global.rgb_enables = PISP_BE_RGB_ENABLE_INPUT + PISP_BE_RGB_ENABLE_OUTPUT0;

	if (in_file.format == "RGBX8888" && !variant->BackendRGB32Supported(0))
	{
		std::cerr << "Backend hardware does not support RGBX input" << std::endl;
		exit(-1);
	}
	pisp_image_format_config i = {};
	i.width = in_file.width;
	i.height = in_file.height;
	i.format = libpisp::get_pisp_image_format(in_file.format);
	assert(i.format);
	libpisp::compute_optimal_stride(i);
	be.SetInputFormat(i);

	pisp_be_output_format_config o = {};
	if (out_file.format == "RGBX8888" && !variant->BackendRGB32Supported(0))
	{
		// Hack to generate RGBX even when BE_MINOR_VERSION < 1 using Resample
		if (out_file.width < i.width)
			std::cerr << "Backend hardware has limited RGBX support; resize artifacts may be present" << std::endl;

		o.image.width = out_file.width * 2 - 1;
		o.image.height = out_file.height;
		o.image.format = libpisp::get_pisp_image_format("UYVY");

		pisp_be_ccm_config csc = {}; // Define a matrix to swap components [0] and [1]
		csc.coeffs[1] = 1024;
		csc.coeffs[3] = 1024;
		csc.coeffs[8] = 1024;
		csc.offsets[0] = 131072; // round to nearest after Resample, for 8-bit output
		csc.offsets[1] = 131072;
		csc.offsets[2] = 131072;
		be.SetCsc(0, csc);
		global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC0;
	}
	else
	{
		o.image.width = out_file.width;
		o.image.height = out_file.height;
		o.image.format = libpisp::get_pisp_image_format(out_file.format);
	}
	assert(o.image.format);
	libpisp::compute_optimal_stride(o.image, true);
	be.SetOutputFormat(0, o);

	if (!out_file.stride)
		out_file.stride = o.image.stride;

	if (in_file.format >= "U")
	{
		pisp_be_ccm_config csc;
		be.InitialiseYcbcrInverse(csc, "jpeg");
		be.SetCcm(csc);
		global.rgb_enables |= PISP_BE_RGB_ENABLE_CCM;
	}

	if (out_file.format >= "U")
	{
		pisp_be_ccm_config csc;
		be.InitialiseYcbcr(csc, "jpeg");
		be.SetCsc(0, csc);
		global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC0;
	}

	be.SetGlobal(global);
	be.SetCrop(0, { 0, 0, i.width, i.height });
	be.SetSmartResize(0, { o.image.width, o.image.height });

	pisp_be_tiles_config config = {};
	be.Prepare(&config);

	backend_device.Setup(config);
	auto buffers = backend_device.AcquireBuffers();

	std::string input_filename = args["input"].as<std::string>();
	std::ifstream in(input_filename, std::ios::binary);
	if (!in.is_open())
	{
		std::cerr << "Unable to open " << input_filename << std::endl;
		exit(-1);
	}

	std::cerr << "Reading " << input_filename << " "
			  << in_file.width << ":" << in_file.height << ":" << in_file.stride << ":" << in_file.format << std::endl;

	Formats.at(in_file.format)
		.read_file(buffers["pispbe-input"].mem, in, in_file.width, in_file.height, in_file.stride,
				   i.stride);
	in.close();

	int ret = backend_device.Run(buffers);
	if (ret)
	{
		std::cerr << "Job run error!" << std::endl;
		exit(-1);
	}

	std::string output_file = args["output"].as<std::string>();
	std::ofstream out(output_file, std::ios::binary);
	if (!out.is_open())
	{
		std::cerr << "Unable to open " << output_file << std::endl;
		exit(-1);
	}

	Formats.at(out_file.format)
		.write_file(out, buffers["pispbe-output0"].mem, out_file.width, out_file.height, out_file.stride,
					o.image.stride);
	out.close();

	backend_device.ReleaseBuffer(buffers);

	std::cerr << "Writing " << output_file << " "
			  << out_file.width << ":" << out_file.height << ":" << out_file.stride << ":" << out_file.format << std::endl;

	return 0;
}
