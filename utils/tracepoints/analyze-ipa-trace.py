#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# Example of how to extract information from libcamera lttng traces

import argparse
import bt2
import statistics as stats
import sys

# pipeline -> {function -> stack(timestamps)}
timestamps = {}

# pipeline:function -> samples[]
samples = {}

def main(argv):
    parser = argparse.ArgumentParser(
            description='A simple analysis script to get statistics on time taken for IPA calls')
    parser.add_argument('-p', '--pipeline', type=str,
                        help='Name of pipeline to filter for')
    parser.add_argument('trace_path', type=str,
                        help='Path to lttng trace (eg. ~/lttng-traces/demo-20201029-184003)')
    args = parser.parse_args(argv[1:])

    traces = bt2.TraceCollectionMessageIterator(args.trace_path)
    for msg in traces:
        if type(msg) is not bt2._EventMessageConst or \
           'pipeline_name' not in msg.event.payload_field or \
           (args.pipeline is not None and \
            msg.event.payload_field['pipeline_name'] != args.pipeline):
            continue

        pipeline = msg.event.payload_field['pipeline_name']
        event = msg.event.name
        func = msg.event.payload_field['function_name']
        timestamp_ns = msg.default_clock_snapshot.ns_from_origin

        if event == 'libcamera:ipa_call_begin':
            if pipeline not in timestamps:
                timestamps[pipeline] = {}
            if func not in timestamps[pipeline]:
                timestamps[pipeline][func] = []
            timestamps[pipeline][func].append(timestamp_ns)

        if event == 'libcamera:ipa_call_end':
            ts = timestamps[pipeline][func].pop()
            key = f'{pipeline}:{func}'
            if key not in samples:
                samples[key] = []
            samples[key].append(timestamp_ns - ts)

    # Compute stats
    rows = []
    rows.append(['pipeline:function', 'min', 'max', 'mean', 'stddev'])
    for k, v in samples.items():
        mean = int(stats.mean(v))
        stddev = int(stats.stdev(v))
        minv = min(v)
        maxv = max(v)
        rows.append([k, str(minv), str(maxv), str(mean), str(stddev)])

    # Get maximum string width for every column
    widths = []
    for i in range(len(rows[0])):
        widths.append(max([len(row[i]) for row in rows]))

    # Print stats table
    for row in rows:
        fmt = [row[i].rjust(widths[i]) for i in range(1, 5)]
        print('{} {} {} {} {}'.format(row[0].ljust(widths[0]), *fmt))

if __name__ == '__main__':
    sys.exit(main(sys.argv))
