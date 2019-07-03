#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2018, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# checkstyle.py - A patch style checker script based on astyle or clang-format
#
# TODO:
#
# - Support other formatting tools and checkers (cppcheck, cpplint, kwstyle, ...)
# - Split large hunks to minimize context noise
# - Improve style issues counting
#

import argparse
import difflib
import fnmatch
import os.path
import re
import shutil
import subprocess
import sys

astyle_options = (
    '-n',
    '--style=linux',
    '--indent=force-tab=8',
    '--attach-namespaces',
    '--attach-extern-c',
    '--pad-oper',
    '--align-pointer=name',
    '--align-reference=name',
    '--keep-one-line-blocks',
    '--max-code-length=120'
)

dependencies = {
    'astyle': False,
    'clang-format': False,
    'git': True,
}

# ------------------------------------------------------------------------------
# Colour terminal handling
#

class Colours:
    Default = 0
    Black = 0
    Red = 31
    Green = 32
    Yellow = 33
    Blue = 34
    Magenta = 35
    Cyan = 36
    LightGrey = 37
    DarkGrey = 90
    LightRed = 91
    LightGreen = 92
    Lightyellow = 93
    LightBlue = 94
    LightMagenta = 95
    LightCyan = 96
    White = 97

    @staticmethod
    def fg(colour):
        if sys.stdout.isatty():
            return '\033[%um' % colour
        else:
            return ''

    @staticmethod
    def bg(colour):
        if sys.stdout.isatty():
            return '\033[%um' % (colour + 10)
        else:
            return ''

    @staticmethod
    def reset():
        if sys.stdout.isatty():
            return '\033[0m'
        else:
            return ''


# ------------------------------------------------------------------------------
# Diff parsing, handling and printing
#

class DiffHunkSide(object):
    """A side of a diff hunk, recording line numbers"""
    def __init__(self, start):
        self.start = start
        self.touched = []
        self.untouched = []

    def __len__(self):
        return len(self.touched) + len(self.untouched)


class DiffHunk(object):
    diff_header_regex = re.compile('@@ -([0-9]+),([0-9]+) \+([0-9]+),([0-9]+) @@')

    def __init__(self, line):
        match = DiffHunk.diff_header_regex.match(line)
        if not match:
            raise RuntimeError("Malformed diff hunk header '%s'" % line)

        self.__from_line = int(match.group(1))
        self.__to_line = int(match.group(3))
        self.__from = DiffHunkSide(self.__from_line)
        self.__to = DiffHunkSide(self.__to_line)

        self.lines = []

    def __repr__(self):
        s = '%s@@ -%u,%u +%u,%u @@\n' % \
                (Colours.fg(Colours.Cyan),
                 self.__from.start, len(self.__from),
                 self.__to.start, len(self.__to))

        for line in self.lines:
            if line[0] == '-':
                s += Colours.fg(Colours.Red)
            elif line[0] == '+':
                s += Colours.fg(Colours.Green)

            if line[0] == '-':
                spaces = 0
                for i in range(len(line)):
                    if line[-i-1].isspace():
                        spaces += 1
                    else:
                        break
                spaces = len(line) - spaces
                line = line[0:spaces] + Colours.bg(Colours.Red) + line[spaces:]

            s += line
            s += Colours.reset()
            s += '\n'

        return s[:-1]

    def append(self, line):
        if line[0] == ' ':
            self.__from.untouched.append(self.__from_line)
            self.__from_line += 1
            self.__to.untouched.append(self.__to_line)
            self.__to_line += 1
        elif line[0] == '-':
            self.__from.touched.append(self.__from_line)
            self.__from_line += 1
        elif line[0] == '+':
            self.__to.touched.append(self.__to_line)
            self.__to_line += 1

        self.lines.append(line.rstrip('\n'))

    def intersects(self, lines):
        for line in lines:
            if line in self.__from.touched:
                return True
        return False

    def side(self, side):
        if side == 'from':
            return self.__from
        else:
            return self.__to


def parse_diff(diff):
    hunks = []
    hunk = None
    for line in diff:
        if line.startswith('@@'):
            if hunk:
                hunks.append(hunk)
            hunk = DiffHunk(line)

        elif hunk is not None:
            hunk.append(line)

    if hunk:
        hunks.append(hunk)

    return hunks


# ------------------------------------------------------------------------------
# Style Checkers
#

_style_checkers = []

class StyleCheckerRegistry(type):
    def __new__(cls, clsname, bases, attrs):
        newclass = super(StyleCheckerRegistry, cls).__new__(cls, clsname, bases, attrs)
        if clsname != 'StyleChecker':
            _style_checkers.append(newclass)
        return newclass


class StyleChecker(metaclass=StyleCheckerRegistry):
    def __init__(self):
        pass

    #
    # Class methods
    #
    @classmethod
    def checkers(cls, filename):
        for checker in _style_checkers:
            if checker.supports(filename):
                yield checker

    @classmethod
    def supports(cls, filename):
        for pattern in cls.patterns:
            if fnmatch.fnmatch(os.path.basename(filename), pattern):
                return True
        return False

    @classmethod
    def all_patterns(cls):
        patterns = set()
        for checker in _style_checkers:
            patterns.update(checker.patterns)

        return patterns


class StyleIssue(object):
    def __init__(self, line_number, line, msg):
        self.line_number = line_number
        self.line = line
        self.msg = msg


class LogCategoryChecker(StyleChecker):
    log_regex = re.compile('\\bLOG\((Debug|Info|Warning|Error|Fatal)\)')
    patterns = ('*.cpp',)

    def __init__(self, content):
        super().__init__()
        self.__content = content

    def check(self, line_numbers):
        issues = []
        for line_number in line_numbers:
            line = self.__content[line_number-1]
            if not LogCategoryChecker.log_regex.search(line):
                continue

            issues.append(StyleIssue(line_number, line, 'LOG() should use categories'))

        return issues


class MesonChecker(StyleChecker):
    patterns = ('meson.build',)

    def __init__(self, content):
        super().__init__()
        self.__content = content

    def check(self, line_numbers):
        issues = []
        for line_number in line_numbers:
            line = self.__content[line_number-1]
            if line.find('\t') != -1:
                issues.append(StyleIssue(line_number, line, 'meson.build should use spaces for indentation'))
        return issues


# ------------------------------------------------------------------------------
# Formatters
#

_formatters = []

class FormatterRegistry(type):
    def __new__(cls, clsname, bases, attrs):
        newclass = super(FormatterRegistry, cls).__new__(cls, clsname, bases, attrs)
        if clsname != 'Formatter':
            _formatters.append(newclass)
        return newclass


class Formatter(metaclass=FormatterRegistry):
    enabled = True

    def __init__(self):
        pass

    #
    # Class methods
    #
    @classmethod
    def formatters(cls, filename):
        for formatter in _formatters:
            if not cls.enabled:
                continue
            if formatter.supports(filename):
                yield formatter

    @classmethod
    def supports(cls, filename):
        if not cls.enabled:
            return False
        for pattern in cls.patterns:
            if fnmatch.fnmatch(os.path.basename(filename), pattern):
                return True
        return False

    @classmethod
    def all_patterns(cls):
        patterns = set()
        for formatter in _formatters:
            if not cls.enabled:
                continue
            patterns.update(formatter.patterns)

        return patterns


class AStyleFormatter(Formatter):
    enabled = False
    patterns = ('*.c', '*.cpp', '*.h')

    @classmethod
    def format(cls, filename, data):
        ret = subprocess.run(['astyle', *astyle_options],
                             input=data.encode('utf-8'), stdout=subprocess.PIPE)
        return ret.stdout.decode('utf-8')


class CLangFormatter(Formatter):
    enabled = False
    patterns = ('*.c', '*.cpp', '*.h')

    @classmethod
    def format(cls, filename, data):
        ret = subprocess.run(['clang-format', '-style=file',
                              '-assume-filename=' + filename],
                             input=data.encode('utf-8'), stdout=subprocess.PIPE)
        return ret.stdout.decode('utf-8')


class DoxygenFormatter(Formatter):
    patterns = ('*.c', '*.cpp')

    return_regex = re.compile(' +\\* +\\\\return +[a-z]')

    @classmethod
    def format(cls, filename, data):
        lines = []
        in_doxygen = False

        for line in data.split('\n'):
            if line.find('/**') != -1:
                in_doxygen = True

            if not in_doxygen:
                lines.append(line)
                continue

            line = cls.return_regex.sub(lambda m: m.group(0)[:-1] + m.group(0)[-1].upper(), line)

            if line.find('*/') != -1:
                in_doxygen = False

            lines.append(line)

        return '\n'.join(lines)


class StripTrailingSpaceFormatter(Formatter):
    patterns = ('*.c', '*.cpp', '*.h', '*.py', 'meson.build')

    @classmethod
    def format(cls, filename, data):
        lines = data.split('\n')
        for i in range(len(lines)):
            lines[i] = lines[i].rstrip() + '\n'
        return ''.join(lines)


# ------------------------------------------------------------------------------
# Style checking
#

def check_file(top_level, commit, filename):
    # Extract the line numbers touched by the commit.
    diff = subprocess.run(['git', 'diff', '%s~..%s' % (commit, commit), '--',
                           '%s/%s' % (top_level, filename)],
                          stdout=subprocess.PIPE).stdout
    diff = diff.decode('utf-8').splitlines(True)
    commit_diff = parse_diff(diff)

    lines = []
    for hunk in commit_diff:
        lines.extend(hunk.side('to').touched)

    # Skip commits that don't add any line.
    if len(lines) == 0:
        return 0

    # Format the file after the commit with all formatters and compute the diff
    # between the unformatted and formatted contents.
    after = subprocess.run(['git', 'show', '%s:%s' % (commit, filename)],
                           stdout=subprocess.PIPE).stdout
    after = after.decode('utf-8')

    formatted = after
    for formatter in Formatter.formatters(filename):
        formatted = formatter.format(filename, formatted)

    after = after.splitlines(True)
    formatted = formatted.splitlines(True)
    diff = difflib.unified_diff(after, formatted)

    # Split the diff in hunks, recording line number ranges for each hunk, and
    # filter out hunks that are not touched by the commit.
    formatted_diff = parse_diff(diff)
    formatted_diff = [hunk for hunk in formatted_diff if hunk.intersects(lines)]

    # Check for code issues not related to formatting.
    issues = []
    for checker in StyleChecker.checkers(filename):
        checker = checker(after)
        for hunk in commit_diff:
            issues += checker.check(hunk.side('to').touched)

    # Print the detected issues.
    if len(issues) == 0 and len(formatted_diff) == 0:
        return 0

    print('%s---' % Colours.fg(Colours.Red), filename)
    print('%s+++' % Colours.fg(Colours.Green), filename)

    if len(formatted_diff):
        for hunk in formatted_diff:
            print(hunk)

    if len(issues):
        issues = sorted(issues, key=lambda i: i.line_number)
        for issue in issues:
            print('%s#%u: %s' % (Colours.fg(Colours.Yellow), issue.line_number, issue.msg))
            print('+%s%s' % (issue.line.rstrip(), Colours.reset()))

    return len(formatted_diff) + len(issues)


def check_style(top_level, commit):
    # Get the commit title and list of files.
    ret = subprocess.run(['git', 'show', '--pretty=oneline','--name-only', commit],
                         stdout=subprocess.PIPE)
    files = ret.stdout.decode('utf-8').splitlines()
    title = files[0]
    files = files[1:]

    separator = '-' * len(title)
    print(separator)
    print(title)
    print(separator)

    # Filter out files we have no checker for.
    patterns = set()
    patterns.update(StyleChecker.all_patterns())
    patterns.update(Formatter.all_patterns())
    files = [f for f in files if len([p for p in patterns if fnmatch.fnmatch(os.path.basename(f), p)])]
    if len(files) == 0:
        print("Commit doesn't touch source files, skipping")
        return

    issues = 0
    for f in files:
        issues += check_file(top_level, commit, f)

    if issues == 0:
        print("No style issue detected")
    else:
        print('---')
        print("%u potential style %s detected, please review" % \
                (issues, 'issue' if issues == 1 else 'issues'))


def extract_revlist(revs):
    """Extract a list of commits on which to operate from a revision or revision
    range.
    """
    ret = subprocess.run(['git', 'rev-parse', revs], stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    if ret.returncode != 0:
        print(ret.stderr.decode('utf-8').splitlines()[0])
        return []

    revlist = ret.stdout.decode('utf-8').splitlines()

    # If the revlist contains more than one item, pass it to git rev-list to list
    # each commit individually.
    if len(revlist) > 1:
        ret = subprocess.run(['git', 'rev-list', *revlist], stdout=subprocess.PIPE)
        revlist = ret.stdout.decode('utf-8').splitlines()
        revlist.reverse()

    return revlist


def git_top_level():
    """Get the absolute path of the git top-level directory."""
    ret = subprocess.run(['git', 'rev-parse', '--show-toplevel'],
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    if ret.returncode != 0:
        print(ret.stderr.decode('utf-8').splitlines()[0])
        return None

    return ret.stdout.decode('utf-8').strip()


def main(argv):

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--formatter', '-f', type=str, choices=['astyle', 'clang-format'],
                        help='Code formatter. Default to clang-format if not specified.')
    parser.add_argument('revision_range', type=str, default='HEAD', nargs='?',
                        help='Revision range (as defined by git rev-parse). Defaults to HEAD if not specified.')
    args = parser.parse_args(argv[1:])

    # Check for required dependencies.
    for command, mandatory in dependencies.items():
        found = shutil.which(command)
        if mandatory and not found:
            print("Executable %s not found" % command)
            return 1

        dependencies[command] = found

    if args.formatter:
        if not args.formatter in dependencies or \
           not dependencies[args.formatter]:
            print("Formatter %s not available" % args.formatter)
            return 1
        formatter = args.formatter
    else:
        if dependencies['clang-format']:
            CLangFormatter.enabled = True
        elif dependencies['astyle']:
            AStyleFormatter.enabled = True
        else:
            print("No formatter found, please install clang-format or astyle")
            return 1

    # Get the top level directory to pass absolute file names to git diff
    # commands, in order to support execution from subdirectories of the git
    # tree.
    top_level = git_top_level()
    if top_level is None:
            return 1

    revlist = extract_revlist(args.revision_range)

    for commit in revlist:
        check_style(top_level, commit)
        print('')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
