#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Helper classes to handle source code generation for libcamera controls


class ControlEnum(object):
    def __init__(self, data):
        self.__data = data

    @property
    def description(self):
        """The enum description"""
        return self.__data.get('description')

    @property
    def name(self):
        """The enum name"""
        return self.__data.get('name')

    @property
    def value(self):
        """The enum value"""
        return self.__data.get('value')


class Control(object):
    def __init__(self, name, data, vendor, mode):
        self.__name = name
        self.__data = data
        self.__enum_values = None
        self.__size = None
        self.__vendor = vendor

        enum_values = data.get('enum')
        if enum_values is not None:
            self.__enum_values = [ControlEnum(enum) for enum in enum_values]

        size = self.__data.get('size')
        if size is not None:
            if len(size) == 0:
                raise RuntimeError(f'Control `{self.__name}` size must have at least one dimension')

            # Compute the total number of elements in the array. If any of the
            # array dimension is a string, the array is variable-sized.
            num_elems = 1
            for dim in size:
                if type(dim) is str:
                    num_elems = 0
                    break

                dim = int(dim)
                if dim <= 0:
                    raise RuntimeError(f'Control `{self.__name}` size must have positive values only')

                num_elems *= dim

            self.__size = num_elems

        if mode == 'properties':
            self.__direction = 'out'
        else:
            direction = self.__data.get('direction')
            if direction is None:
                raise RuntimeError(f'Control `{self.__name}` missing required field `direction`')
            if direction not in ['in', 'out', 'inout']:
                raise RuntimeError(f'Control `{self.__name}` direction `{direction}` is invalid; must be one of `in`, `out`, or `inout`')
            self.__direction = direction

    @property
    def description(self):
        """The control description"""
        return self.__data.get('description')

    @property
    def enum_values(self):
        """The enum values, if the control is an enumeration"""
        if self.__enum_values is None:
            return
        for enum in self.__enum_values:
            yield enum

    @property
    def enum_values_count(self):
        """The number of enum values, if the control is an enumeration"""
        if self.__enum_values is None:
            return 0
        return len(self.__enum_values)

    @property
    def is_enum(self):
        """Is the control an enumeration"""
        return self.__enum_values is not None

    @property
    def vendor(self):
        """The vendor string, or None"""
        return self.__vendor

    @property
    def name(self):
        """The control name (CamelCase)"""
        return self.__name

    @property
    def type(self):
        typ = self.__data.get('type')
        size = self.__data.get('size')

        if typ == 'string':
            return 'std::string'

        if self.__size is None:
            return typ

        if self.__size:
            return f"Span<const {typ}, {self.__size}>"
        else:
            return f"Span<const {typ}>"

    @property
    def direction(self):
        in_flag = 'ControlId::Direction::In'
        out_flag = 'ControlId::Direction::Out'

        if self.__direction == 'inout':
            return f'{in_flag} | {out_flag}'
        if self.__direction == 'in':
            return in_flag
        if self.__direction == 'out':
            return out_flag

    @property
    def element_type(self):
        return self.__data.get('type')

    @property
    def size(self):
        return self.__size
