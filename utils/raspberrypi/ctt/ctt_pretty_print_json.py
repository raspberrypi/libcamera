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
            "skipnewline": True,
            "need_indent": False,
            "need_space": False,
        }

        self.fout = fout

    def newline(self):
        if not self.state["skipnewline"]:
            self.fout.write('\n')
            self.state["need_indent"] = True
            self.state["need_space"] = False
        self.state["skipnewline"] = True

    def write(self, c):
        if self.state["need_indent"]:
            self.fout.write(' ' * self.state["indent"] * 4)
            self.state["need_indent"] = False
        if self.state["need_space"]:
            self.fout.write(' ')
            self.state["need_space"] = False
        self.fout.write(c)
        self.state["skipnewline"] = False

    def process_char(self, c):
        if c == '{':
            self.newline()
            self.write(c)
            self.state["indent"] += 1
            self.newline()
        elif c == '}':
            self.state["indent"] -= 1
            self.newline()
            self.write(c)
        elif c == '[':
            self.newline()
            self.write(c)
            self.state["indent"] += 1
            self.newline()
            self.state["inarray"] = [True] + self.state["inarray"]
            self.state["arraycount"] = [0] + self.state["arraycount"]
        elif c == ']':
            self.state["indent"] -= 1
            self.newline()
            self.state["inarray"].pop(0)
            self.state["arraycount"].pop(0)
            self.write(c)
        elif c == ':':
            self.write(c)
            self.state["need_space"] = True
        elif c == ',':
            if not self.state["inarray"][0]:
                self.write(c)
                self.newline()
            else:
                self.write(c)
                self.state["arraycount"][0] += 1
                if self.state["arraycount"][0] == 16:
                    self.state["arraycount"][0] = 0
                    self.newline()
                else:
                    self.state["need_space"] = True
        elif c.isspace():
            pass
        else:
            self.write(c)

    def print(self, string):
        for c in string:
            self.process_char(c)
        self.newline()


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
