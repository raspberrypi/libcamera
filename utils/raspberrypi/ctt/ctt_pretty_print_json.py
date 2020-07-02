# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi (Trading) Limited
#
# ctt_pretty_print_json.py - camera tuning tool JSON formatter

import sys


class JSONPrettyPrinter(object):
    """
    Take a collapsed JSON file and make it more readable
    """
    def __init__(self, fout):
        self.state = {
            "indent": 0,
            "inarray": [False],
            "arraycount": [],
            "skipnewline": True
        }

        self.fout = fout

    def newline(self):
        self.fout.write('\n')
        self.fout.write(' ' * self.state["indent"] * 4)

    def process_char(self, c):
        if c == '{':
            if not self.state["skipnewline"]:
                self.newline()
            self.fout.write(c)
            self.state["indent"] += 1
            self.newline()
        elif c == '}':
            self.state["indent"] -= 1
            self.newline()
            self.fout.write(c)
        elif c == '[':
            self.newline()
            self.fout.write(c)
            self.state["indent"] += 1
            self.newline()
            self.state["inarray"] = [True] + self.state["inarray"]
            self.state["arraycount"] = [0] + self.state["arraycount"]
        elif c == ']':
            self.state["indent"] -= 1
            self.newline()
            self.state["inarray"].pop(0)
            self.state["arraycount"].pop(0)
            self.fout.write(c)
        elif c == ':':
            self.fout.write(c)
            self.fout.write(' ')
        elif c == ' ':
            pass
        elif c == ',':
            if not self.state["inarray"][0]:
                self.fout.write(c)
                self.fout.write(' ')
                self.newline()
            else:
                self.fout.write(c)
                self.state["arraycount"][0] += 1
                if self.state["arraycount"][0] == 16:
                    self.state["arraycount"][0] = 0
                    self.newline()
                else:
                    self.fout.write(' ')
        else:
            self.fout.write(c)
        self.state["skipnewline"] = (c == '[')

    def print(self, string):
        for c in string:
            self.process_char(c)


def pretty_print_json(str_in, output_filename):
    with open(output_filename, "w") as fout:
        printer = JSONPrettyPrinter(fout)
        printer.print(str_in)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: %s filename" % sys.argv[0])
        sys.exit(1)

    input_filename = sys.argv[1]
    with open(input_filename, "r") as fin:
        printer = JSONPrettyPrinter(sys.stdout)
        printer.print(fin.read())
