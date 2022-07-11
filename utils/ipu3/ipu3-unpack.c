/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ipu3-unpack - Unpack IPU3 raw Bayer format to 16-bit Bayer
 *
 * Copyright 2018 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static void usage(const char *argv0)
{
	printf("Usage: %s input-file output-file\n", basename(argv0));
	printf("Unpack the IPU3 raw Bayer format to 16-bit Bayer\n");
}

int main(int argc, char *argv[])
{
	int in_fd;
	int out_fd;
	int ret;

	if (argc != 3) {
		usage(argv[0]);
		return 1;
	}

	in_fd = open(argv[1], O_RDONLY);
	if (in_fd == -1) {
		fprintf(stderr, "Failed to open input file '%s': %s\n",
			argv[1], strerror(errno));
		return 1;
	}

	out_fd = open(argv[2], O_WRONLY | O_TRUNC | O_CREAT, 0644);
	if (out_fd == -1) {
		fprintf(stderr, "Failed to open output file '%s': %s\n",
			argv[2], strerror(errno));
		return 1;
	}

	while (1) {
		uint8_t in_data[32];
		uint8_t out_data[50];
		unsigned int i;

		ret = read(in_fd, in_data, 32);
		if (ret == -1) {
			fprintf(stderr, "Failed to read input data: %s\n",
				strerror(errno));
			goto done;
		}

		if (ret < 32) {
			if (ret != 0)
				fprintf(stderr, "%u bytes of stray data at end of input\n",
					ret);
			break;
		}

		for (i = 0; i < 25; ++i) {
			unsigned int index = (i * 10) / 8;
			unsigned int lsb_shift = (i * 10) % 8;
			unsigned int msb_shift = 8 - lsb_shift;
			uint16_t pixel;

			pixel = ((in_data[index+1] << msb_shift) & 0x3ff)
			      | ((in_data[index+0] >> lsb_shift) & 0x3ff);
			out_data[i*2+0] = (pixel >> 0) & 0xff;
			out_data[i*2+1] = (pixel >> 8) & 0xff;
		}

		ret = write(out_fd, out_data, 50);
		if (ret < -1) {
			fprintf(stderr, "Failed to write output data: %s\n",
				strerror(errno));
			goto done;
		}
	}

done:
	close(in_fd);
	close(out_fd);

	return ret ? 1 : 0;
}
