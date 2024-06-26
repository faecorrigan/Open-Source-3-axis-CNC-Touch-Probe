#!/usr/bin/env python

import sys
import subprocess
from datetime import datetime
from functools import reduce

t0 = None
data = []
tags = {}

# Header notes, for example packet 6188 6122 204c 104c 1001 800e e3c1
##############
# 6188 --> 0x8861 header flags. I have also seen variant 0x8961. The difference is 89 sets the bit for sequence number suppression. Note: maybe it was actually a bit flip; it doesn't seem normal
# next byte on the wire (61) is the sequence number. Does not seem to function as a sequence number fwiw
# next pair of bytes 2220 -> 0x2022 "Destination PAN"
# next pair of bytes 4c10 -> 0x104c "Destination"
# next pair of bytes 4c10 -> 0x104c "Source"
# remaining bytes: data

# ACK example:  0200 00ec c6
#############
# 0200 -> 0x0002 header flags = ACK
# 00 -> sequence number
# ec c6 -> ??? Matched the two trailing bytes on the previous message

sample_ok = "0x0000:  6188 0122 204c 104c 1001 3b11 efec"
ack_ok = "0x0000:  0200 1deb ec"
found_unexpected_preamble = False

for fname in sys.argv[1:]:
    print(fname)
    x = subprocess.run(['tcpdump', '-xx', '-n', '-r', fname], capture_output=True)

    for line in x.stdout.decode('utf8').split('\n'):
        line = line.strip()
        if 'IEEE' in line:
            tstr = line.split()[0]
            t = datetime.strptime(tstr, "%H:%M:%S.%f")
            if t0 is None:
                t0 = t
            dt = (t - t0).total_seconds()
            continue
        extra = ""
        if line.startswith(sample_ok[:8]) and len(line) == len(sample_ok):
            notes = []
            line = line[8:].replace(" ", "")
            flags = line[0:4]
            if not flags in ["6188"]: #, "6189"]:
                notes.append("Flags {}".format(flags))
            seq = line[4:6]
            pan = line[6:10]
            if pan != "2220":
                notes.append("PAN {}".format(pan))
            dest = line[10:14]
            if dest != "4c10":
                notes.append("DST {}".format(dest))
            src = line[14:18]
            if src != "4c10":
                notes.append("SRC {}".format(src))
            line = line[18:]

            cmd = line[:2]
            cmd_data = line[2:6]
            crc_metadata = line[6:]

            extra = ""
            if cmd == "03":
                notes.append("ADR {}".format(cmd_data))
                notes.append("SEQ {}".format(seq))
            else:
                extra = "Bat {} Seq {}".format(cmd_data, seq)

            line = cmd + " " + cmd_data
            line = " ".join([line] + notes)
        elif line.startswith(ack_ok[:13]) and len(line) == len(ack_ok):
            line = line[13:].replace(" ", "")
            seq = line[0:2]
            tail1 = line[2:4]
            tail2 = line[4:]
            line = "ACK  {} {}".format(tail1, tail2)
            extra = "Seq {}".format(seq)
        else:
            if not found_unexpected_preamble:
                print("Lines with unexpected preamble:")
                found_unexpected_preamble = True
            print("\t{}".format(line))
        if not line:
            continue
        data.append((dt, line, extra))

presses = []
press_times = []
tlast = -1000000
new_tags = {}
double_tags = {}
for (t, line, extra) in data:
    if t - tlast > .3:
        # new press
        presses.append([])
        press_times.append(t)
        print()
        print("Press {}:".format(len(presses)))

    tag_name = None
    default_tag_name = "P{}_{}".format(len(presses), len(presses[-1]) + 1)
    # if len(presses[-1]) == 0:
    #     tag_name = "P{}_init1".format(len(presses))
    # elif len(presses[-1]) == 1:
    #     tag_name = "P{}_init2".format(len(presses))
    tag_name = tag_name or default_tag_name
    
    if tag_name is not None:
        if line in tags:
            if tag_name != default_tag_name:
                double_tags[tag_name] = line
        else:
            new_tags[tag_name] = line
            tags[line] = tag_name

    tags[line] = tags.get(line, tag_name or default_tag_name)

    dt_us = "____" if len(presses[-1]) == 0 else (int)((t-tlast)*1e6)
    print("{:.3f}\t+{:04}us\t{:15}\t{}  {}".format(t, dt_us,tags[line], line, extra))

    presses[-1].append((line))
    tlast = t

print()

for i, press in enumerate(presses):
    press_tags = []
    for line in press:
        if not tags[line] in press_tags:
            press_tags.append(tags[line])

    print("Press {} ({:6.03f}): {}".format(i+1, press_times[i] - press_times[0], press_tags))

print()
print("new tags:")
for tag, line in new_tags.items():
    print('\t"{:10}": "{}"'.format(tag, line))
if len(double_tags):
    print()
    print("double tags:")
    for tag, line in double_tags.items():
        print('\t{:10}: {} ({})'.format(tag, line, tags[line]))
