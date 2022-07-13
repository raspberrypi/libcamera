/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ipu3-pack - Convert unpacked RAW10 Bayer data to the IPU3 packed Bayer formats
 *
 * Copyright 2022 Umang Jain <umang.jain@ideasonboard.com>
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
	printf("Convert unpacked RAW10 Bayer data to the IPU3 packed Bayer formats\n");
	printf("If the output-file '-', output data will be written to standard output\n");
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

	if (strcmp(argv[2], "-") == 0) {
		out_fd = STDOUT_FILENO;
	} else {
		out_fd = open(argv[2], O_WRONLY | O_TRUNC | O_CREAT, 0644);
		if (out_fd == -1) {
			fprintf(stderr, "Failed to open output file '%s': %s\n",
				argv[2], strerror(errno));
			close(in_fd);
			return 1;
		}
	}

	while (1) {
		uint16_t in_data[25];
		uint8_t out_data[32];
		unsigned int i;

		ret = read(in_fd, in_data, sizeof(in_data));
		if (ret < 0) {
			fprintf(stderr, "Failed to read input data: %s\n",
				strerror(errno));
			goto done;
		}

		if ((unsigned)ret < sizeof(in_data)) {
			if (ret != 0)
				fprintf(stderr, "%u bytes of stray data at end of input\n",
					ret);
			goto done;
		}

		for (i = 0; i < 30; ++i) {
			unsigned int index = (i * 8) / 10;
			unsigned int msb_shift = (i * 8) % 10;
			unsigned int lsb_shift = 10 - msb_shift;

			out_data[i] = ((in_data[index] >> msb_shift) & 0xff)
				    | ((in_data[index+1] << lsb_shift) & 0xff);
		}

		out_data[30] = (in_data[24] >> 0) & 0xff;
		out_data[31] = (in_data[24] >> 8) & 0x03;

		ret = write(out_fd, out_data, sizeof(out_data));
		if (ret < 0) {
			fprintf(stderr, "Failed to write output data: %s\n",
				strerror(errno));
			goto done;
		}
	}

done:
	close(in_fd);
	if (out_fd != STDOUT_FILENO)
		close(out_fd);

	return ret ? 1 : 0;
}
