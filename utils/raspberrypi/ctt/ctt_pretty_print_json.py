# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi (Trading) Limited
#
# ctt_pretty_print_json.py - camera tuning tool JSON formatter

"""
takes a collapsed json file and makes it more readable
"""
def process_file(string, fout, state):
    for c in string:
        process_char(c, fout, state)

def print_newline(fout, state):
    fout.write('\n')
    fout.write(' '*state["indent"]*4)

def process_char(c, fout, state):
    if c == '{':
        if not state["skipnewline"]: print_newline(fout, state)
        fout.write(c)
        state["indent"] += 1
        print_newline(fout, state)
    elif c == '}':
        state["indent"] -= 1
        print_newline(fout, state)
        fout.write(c)
    elif c == '[':
        print_newline(fout, state)
        fout.write(c)
        state["indent"] += 1
        print_newline(fout, state)
        state["inarray"] = [True] + state["inarray"]
        state["arraycount"] = [0] + state["arraycount"]
    elif c == ']':
        state["indent"] -= 1
        print_newline(fout, state)
        state["inarray"].pop(0)
        state["arraycount"].pop(0)
        fout.write(c)
    elif c == ':':
        fout.write(c)
        fout.write(' ')
    elif c == ' ':
        pass
    elif c == ',':
        if not state["inarray"][0]:
            fout.write(c)
            fout.write(' ')
            print_newline(fout, state)
        else:
            fout.write(c)
            state["arraycount"][0] += 1
            if state["arraycount"][0] == 16:
                state["arraycount"][0] = 0
                print_newline(fout, state)
            else:
                fout.write(' ')
    else:
        fout.write(c)
    state["skipnewline"] = (c == '[')

def pretty_print_json(str_in, output_filename):
    state = {"indent": 0, "inarray": [False], "arraycount": [], "skipnewline" : True}
    with open(output_filename, "w") as fout:
        process_file(str_in, fout, state)


if __name__ == '__main__':
    pretty_print_json("../ctt/ref_json/final_imx477.json", "pretty.json")
