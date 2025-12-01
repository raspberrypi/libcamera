#!/usr/bin/python3

# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2025 Raspberry Pi Ltd
#
# test_convert.py - Test script for libpisp convert utility
#

import argparse
import os
import subprocess
import sys
import tempfile
import shutil
from pathlib import Path
import hashlib


class ConvertTester:
    def __init__(self, convert_binary, output_dir=None, input_dir=None, reference_dir=None):
        """Initialize the tester with the path to the convert binary."""
        self.convert_binary = convert_binary
        self.output_dir = output_dir
        self.input_dir = input_dir
        self.reference_dir = reference_dir
        if not os.path.exists(convert_binary):
            raise FileNotFoundError(f"Convert binary not found: {convert_binary}")

        # Test cases: (input_file, output_file, input_format, output_format, reference_file)
        self.test_cases = [
            {
                "input_file": "conv_yuv420_4056x3040_4056s.yuv",
                "output_file": "out_4056x3050_12168s_rgb888.rgb",
                "input_format": "4056:3040:4056:YUV420P",
                "output_format": "4056:3040:12168:RGB888",
                "reference_file": "ref_4056x3050_12168s_rgb888.rgb"
            },
            {
                "input_file": "conv_800x600_1200s_422_yuyv.yuv",
                "output_file": "out_1600x1200_422p.yuv",
                "input_format": "800:600:1600:YUYV",
                "output_format": "1600:1200:1600:YUV422P",
                "reference_file": "ref_1600x1200_422p.yuv"
            },
            {
                "input_file": "conv_rgb888_800x600_2432s.rgb",
                "output_file": "out_4000x3000_4032s.yuv",
                "input_format": "800:600:2432:RGB888",
                "output_format": "4000:3000:0:YUV444P",
                "reference_file": "ref_4000x3000_4032s.yuv"
            },
            # Add more test cases here as needed
        ]

    def run_convert(self, input_file, output_file, input_format, output_format):
        """Run the convert utility with the specified parameters."""
        # Use input directory if specified
        if self.input_dir:
            input_file = os.path.join(self.input_dir, input_file)

        # Use output directory if specified
        if self.output_dir:
            output_file = os.path.join(self.output_dir, output_file)

        cmd = [
            self.convert_binary,
            input_file,
            output_file,
            "--input-format", input_format,
            "--output-format", output_format
        ]

        print(f"Running: {' '.join(cmd)}")

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            print("Convert completed successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Convert failed with exit code {e.returncode}")
            print(f"stdout: {e.stdout}")
            print(f"stderr: {e.stderr}")
            return False

    def compare_files(self, file1, file2):
        """Compare two files and return True if they are identical."""
        if not os.path.exists(file1):
            print(f"Error: File {file1} does not exist")
            return False
        if not os.path.exists(file2):
            print(f"Error: File {file2} does not exist")
            return False

        # Compare file sizes first
        size1 = os.path.getsize(file1)
        size2 = os.path.getsize(file2)

        if size1 != size2:
            print(f"Files have different sizes: {size1} vs {size2}")
            return False

        # Compare file contents using hash
        hash1 = self._file_hash(file1)
        hash2 = self._file_hash(file2)

        if hash1 == hash2:
            print("Files are identical")
            return True
        else:
            print("Files are different")
            return False

    def _file_hash(self, filepath):
        """Calculate SHA256 hash of a file."""
        hash_sha256 = hashlib.sha256()
        with open(filepath, "rb") as f:
            for chunk in iter(lambda: f.read(4096), b""):
                hash_sha256.update(chunk)
        return hash_sha256.hexdigest()

    def run_test_case(self, test_case):
        """Run a single test case."""
        print(f"\n=== Running test case ===")
        print(f"Input file: {test_case['input_file']}")
        print(f"Output file: {test_case['output_file']}")
        print(f"Input format: {test_case['input_format']}")
        print(f"Output format: {test_case['output_format']}")
        print(f"Reference file: {test_case['reference_file']}")

        # Check if input file exists
        input_file = test_case['input_file']
        if self.input_dir:
            input_file = os.path.join(self.input_dir, test_case['input_file'])

        if not os.path.exists(input_file):
            print(f"Error: Input file {input_file} does not exist")
            return False

        # Run the convert utility
        success = self.run_convert(
            test_case['input_file'],
            test_case['output_file'],
            test_case['input_format'],
            test_case['output_format']
        )

        if not success:
            return False

        # Compare with reference file if it exists
        reference_file = test_case['reference_file']
        if self.reference_dir:
            reference_file = os.path.join(self.reference_dir, test_case['reference_file'])

        if os.path.exists(reference_file):
            print(f"Comparing output with reference file...")
            # Use output directory for the generated output file
            output_file = test_case['output_file']
            if self.output_dir:
                output_file = os.path.join(self.output_dir, test_case['output_file'])
            return self.compare_files(output_file, reference_file)
        else:
            print(f"Reference file {reference_file} not found, skipping comparison")
            return True

    def run_all_tests(self):
        """Run all test cases."""
        print(f"Testing convert utility: {self.convert_binary}")
        if self.input_dir:
            print(f"Input directory: {self.input_dir}")
        if self.output_dir:
            print(f"Output directory: {self.output_dir}")
        if self.reference_dir:
            print(f"Reference directory: {self.reference_dir}")
        print(f"Number of test cases: {len(self.test_cases)}")

        passed = 0
        failed = 0

        for i, test_case in enumerate(self.test_cases, 1):
            print(f"\n--- Test case {i}/{len(self.test_cases)} ---")

            if self.run_test_case(test_case):
                passed += 1
                print("✓ Test PASSED")
            else:
                failed += 1
                print("✗ Test FAILED")

        print(f"\n=== Test Summary ===")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Total: {len(self.test_cases)}")

        return failed == 0


def main():
    parser = argparse.ArgumentParser(description="Test script for libpisp convert utility")
    parser.add_argument("convert_binary", help="Path to the convert binary")
    parser.add_argument("--test-dir", help="Directory containing test files")
    parser.add_argument("--in", dest="input_dir", help="Directory containing input files")
    parser.add_argument("--out", help="Directory where output files will be written")
    parser.add_argument("--ref", help="Directory containing reference files")

    args = parser.parse_args()

    try:
        tester = ConvertTester(args.convert_binary, args.out, args.input_dir, args.ref)

        # Change to test directory if specified
        if args.test_dir:
            if not os.path.exists(args.test_dir):
                print(f"Error: Test directory {args.test_dir} does not exist")
                return 1
            os.chdir(args.test_dir)
            print(f"Changed to test directory: {args.test_dir}")

        # Create output directory if specified and it doesn't exist
        if args.out:
            if not os.path.exists(args.out):
                os.makedirs(args.out)
                print(f"Created output directory: {args.out}")

        # Run all tests
        success = tester.run_all_tests()
        return 0 if success else 1

    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
