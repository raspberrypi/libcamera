# SPDX-License-Identifier: BSD-2-Clause

import re
import sys
import os

if len(sys.argv) != 2:
    print("Usage: {} <infile>".format(sys.argv[0]))
    sys.exit()

infile = sys.argv[1]
insplit = os.path.splitext(infile)
outfile = insplit[0] + '_parsed' + insplit[1]

frame_re = re.compile(r'frame (\d+) started')

delays = {
    'Analogue Gain': 1,
    'Exposure': 2,
    'Vertical Blanking': 2
}

ctrl_action = {
    'Write': {},
    'Get': {},
    'Queue': {},
    'No-op': {}
}

ctrl_re = {
    'Write': re.compile(r'Setting (.*?) to (\d+) at index (\d+)'),
    'No-op': re.compile(r'Queue is empty, (.*?) (.*?) (.*?)'),
    'Get': re.compile(r'Reading (.*?) to (\d+) at index (\d+)'),
    'Queue': re.compile(r'Queuing (.*?) to (\d+) at index (\d+)')
}

frame_num = -1

max_delay = 0
for k, d in delays.items():
    if max_delay < d:
        max_delay = d

with open(infile) as f:
    lines = f.readlines()

for line in lines:
    r = frame_re.search(line)
    if r:
        frame_num = int(r.group(1))

    for (key, re) in ctrl_re.items():
        r = re.search(line)
        if r:
            ctrl_action[key][(frame_num, r.group(1))] = (r.group(2), r.group(3))

with open(outfile, 'wt') as f:
    queueIndex = 1
    f.write('{:<10}{:<15}{:<12}{:<18}{}\n'.format('Frame', 'Action', 'Gain', 'Exposure', 'Vblank'))
    for frame in range(0, frame_num + 1):
        for (k, a) in ctrl_action.items():
            str = '{:<10}{:<10}'.format(frame, k)

            for c in delays.keys():
                # Tabulate all results
                str += '{:>5} {:<10}'.format(a[(frame, c)][0] if (frame, c) in a.keys() else '---',
                                             '[' + (a[(frame, c)][1] if (frame, c) in a.keys() else '-') + ']')

            f.write(str.strip() + '\n')

# Test the write -> get matches the set delay.
for (frame, c) in ctrl_action['Write'].keys():
    set_value = ctrl_action['Write'][(frame, c)][0]
    delay_frame = frame + delays[c]
    if (delay_frame <= frame_num):
        if (delay_frame, c) in ctrl_action['Get']:
            get_value = ctrl_action['Get'][(delay_frame, c)][0]
            if get_value != set_value:
                print('Error: {} written at frame {} to value {} != {} at frame {}'
                      .format(c, frame, set_value, get_value, delay_frame))
        else:
            print('Warning: {} written at frame {} to value {} did not get logged on frame {} - dropped frame?'
                  .format(c, frame, set_value, delay_frame))

# Test the queue -> write matches the set delay.
for (frame, c) in ctrl_action['Queue'].keys():
    set_value = ctrl_action['Queue'][(frame, c)][0]
    delay_frame = frame + max_delay - delays[c] + 1
    if (delay_frame <= frame_num):
        if (delay_frame, c) in ctrl_action['Write']:
            write_value = ctrl_action['Write'][(delay_frame, c)][0]
            if write_value != set_value:
                print('Info: {} queued at frame {} to value {} != {} written at frame {}'
                      ' - lagging behind or double queue on a single frame!'
                      .format(c, frame, set_value, write_value, delay_frame))
        else:
            print('Warning: {} queued at frame {} to value {} did not get logged on frame {} - dropped frame?'
                  .format(c, frame, set_value, delay_frame))

# Test the get -> write matches the set delay going backwards.
for (frame, c) in ctrl_action['Get'].keys():
    get_value = ctrl_action['Get'][(frame, c)][0]
    delay_frame = frame - delays[c]
    if (delay_frame >= 6):
        if (delay_frame, c) in ctrl_action['Write']:
            write_value = ctrl_action['Write'][(delay_frame, c)][0]
            if get_value != write_value:
                print('Info: {} got at frame {} to value {} != {} written at frame {}'
                      ' - lagging behind or double queue on a single frame!'
                      .format(c, frame, get_value, write_value, delay_frame))
        else:
            print('Warning: {} got at frame {} to value {} did not get written on frame {}'
                  .format(c, frame, get_value, delay_frame))
