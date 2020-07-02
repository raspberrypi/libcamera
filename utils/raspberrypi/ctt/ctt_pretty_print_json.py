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
    def __init__(self):
        self.state = {
            "indent": 0,
            "inarray": [False],
            "arraycount": [],
            "skipnewline": True
        }

    def newline(self, fout):
        fout.write('\n')
        fout.write(' ' * self.state["indent"] * 4)

    def process_char(self, c, fout):
        if c == '{':
            if not self.state["skipnewline"]:
                self.newline(fout)
            fout.write(c)
            self.state["indent"] += 1
            self.newline(fout)
        elif c == '}':
            self.state["indent"] -= 1
            self.newline(fout)
            fout.write(c)
        elif c == '[':
            self.newline(fout)
            fout.write(c)
            self.state["indent"] += 1
            self.newline(fout)
            self.state["inarray"] = [True] + self.state["inarray"]
            self.state["arraycount"] = [0] + self.state["arraycount"]
        elif c == ']':
            self.state["indent"] -= 1
            self.newline(fout)
            self.state["inarray"].pop(0)
            self.state["arraycount"].pop(0)
            fout.write(c)
        elif c == ':':
            fout.write(c)
            fout.write(' ')
        elif c == ' ':
            pass
        elif c == ',':
            if not self.state["inarray"][0]:
                fout.write(c)
                fout.write(' ')
                self.newline(fout)
            else:
                fout.write(c)
                self.state["arraycount"][0] += 1
                if self.state["arraycount"][0] == 16:
                    self.state["arraycount"][0] = 0
                    self.newline(fout)
                else:
                    fout.write(' ')
        else:
            fout.write(c)
        self.state["skipnewline"] = (c == '[')

    def print(self, string, fout):
        for c in string:
            self.process_char(c, fout)


def pretty_print_json(str_in, output_filename):
    with open(output_filename, "w") as fout:
        printer = JSONPrettyPrinter()
        printer.print(str_in, fout)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: %s filename" % sys.argv[0])
        sys.exit(1)

    input_filename = sys.argv[1]
    with open(input_filename, "r") as fin:
        pretty_print_json(fin.read(), "pretty.json")
