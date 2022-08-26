#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# mojom_libcamera_generator.py - Generates libcamera files from a mojom.Module.

import argparse
import datetime
import os
import re

import mojom.fileutil as fileutil
import mojom.generate.generator as generator
import mojom.generate.module as mojom
from mojom.generate.template_expander import UseJinja


GENERATOR_PREFIX = 'libcamera'

_kind_to_cpp_type = {
    mojom.BOOL:   'bool',
    mojom.INT8:   'int8_t',
    mojom.UINT8:  'uint8_t',
    mojom.INT16:  'int16_t',
    mojom.UINT16: 'uint16_t',
    mojom.INT32:  'int32_t',
    mojom.UINT32: 'uint32_t',
    mojom.FLOAT:  'float',
    mojom.INT64:  'int64_t',
    mojom.UINT64: 'uint64_t',
    mojom.DOUBLE: 'double',
}

_bit_widths = {
    mojom.BOOL:   '8',
    mojom.INT8:   '8',
    mojom.UINT8:  '8',
    mojom.INT16:  '16',
    mojom.UINT16: '16',
    mojom.INT32:  '32',
    mojom.UINT32: '32',
    mojom.FLOAT:  '32',
    mojom.INT64:  '64',
    mojom.UINT64: '64',
    mojom.DOUBLE: '64',
}

def ModuleName(path):
    return path.split('/')[-1].split('.')[0]

def ModuleClassName(module):
    return re.sub(r'^IPA(.*)Interface$', lambda match: match.group(1),
                  module.interfaces[0].mojom_name)

def Capitalize(name):
    return name[0].upper() + name[1:]

def ConstantStyle(name):
    return generator.ToUpperSnakeCase(name)

def Choose(cond, t, f):
    return t if cond else f

def CommaSep(l):
    return ', '.join([m for m in l])

def ParamsCommaSep(l):
    return ', '.join([m.mojom_name for m in l])

def GetDefaultValue(element):
    if element.default is not None:
        return element.default
    if type(element.kind) == mojom.Kind:
        return '0'
    if IsFlags(element):
        return ''
    if mojom.IsEnumKind(element.kind):
        return f'static_cast<{element.kind.mojom_name}>(0)'
    if isinstance(element.kind, mojom.Struct) and \
       element.kind.mojom_name == 'SharedFD':
        return '-1'
    return ''

def HasDefaultValue(element):
    return GetDefaultValue(element) != ''

def HasDefaultFields(element):
    return True in [HasDefaultValue(x) for x in element.fields]

def GetAllTypes(element):
    if mojom.IsArrayKind(element):
        return GetAllTypes(element.kind)
    if mojom.IsMapKind(element):
        return GetAllTypes(element.key_kind) + GetAllTypes(element.value_kind)
    if isinstance(element, mojom.Parameter):
        return GetAllTypes(element.kind)
    if mojom.IsEnumKind(element):
        return [element.mojom_name]
    if not mojom.IsStructKind(element):
        return [element.spec]
    if len(element.fields) == 0:
        return [element.mojom_name]
    ret = [GetAllTypes(x.kind) for x in element.fields]
    ret = [x for sublist in ret for x in sublist]
    return list(set(ret))

def GetAllAttrs(element):
    if mojom.IsArrayKind(element):
        return GetAllAttrs(element.kind)
    if mojom.IsMapKind(element):
        return {**GetAllAttrs(element.key_kind), **GetAllAttrs(element.value_kind)}
    if isinstance(element, mojom.Parameter):
        return GetAllAttrs(element.kind)
    if mojom.IsEnumKind(element):
        return element.attributes if element.attributes is not None else {}
    if mojom.IsStructKind(element) and len(element.fields) == 0:
        return element.attributes if element.attributes is not None else {}
    if not mojom.IsStructKind(element):
        if hasattr(element, 'attributes'):
            return element.attributes or {}
        return {}
    attrs = [(x.attributes) for x in element.fields]
    ret = {}
    for d in attrs:
        ret.update(d or {})
    if hasattr(element, 'attributes'):
        ret.update(element.attributes or {})
    return ret

def NeedsControlSerializer(element):
    types = GetAllTypes(element)
    for type in ['ControlList', 'ControlInfoMap']:
        if f'x:{type}' in types:
            raise Exception(f'Unknown type "{type}" in {element.mojom_name}, did you mean "libcamera.{type}"?')
    return "ControlList" in types or "ControlInfoMap" in types

def HasFd(element):
    attrs = GetAllAttrs(element)
    if isinstance(element, mojom.Kind):
        types = GetAllTypes(element)
    else:
        types = GetAllTypes(element.kind)
    return "SharedFD" in types or (attrs is not None and "hasFd" in attrs)

def WithDefaultValues(element):
    return [x for x in element if HasDefaultValue(x)]

def WithFds(element):
    return [x for x in element if HasFd(x)]

def MethodParamInputs(method):
    return method.parameters

def MethodParamOutputs(method):
    if method.response_parameters is None:
        return []

    if MethodReturnValue(method) == 'void':
        return method.response_parameters

    if len(method.response_parameters) <= 1:
        return []

    return method.response_parameters[1:]

def MethodParamsHaveFd(parameters):
    return len([x for x in parameters if HasFd(x)]) > 0

def MethodInputHasFd(method):
    return MethodParamsHaveFd(method.parameters)

def MethodOutputHasFd(method):
    return MethodParamsHaveFd(MethodParamOutputs(method))

def MethodParamNames(method):
    params = []
    for param in method.parameters:
        params.append(param.mojom_name)
    for param in MethodParamOutputs(method):
        params.append(param.mojom_name)
    return params

def MethodParameters(method):
    params = []
    for param in method.parameters:
        params.append('const %s %s%s' % (GetNameForElement(param),
                                         '' if IsPod(param) or IsEnum(param) else '&',
                                         param.mojom_name))
    for param in MethodParamOutputs(method):
        params.append(f'{GetNameForElement(param)} *{param.mojom_name}')
    return params

def MethodReturnValue(method):
    if method.response_parameters is None or len(method.response_parameters) == 0:
        return 'void'
    first_output = method.response_parameters[0]
    if ((len(method.response_parameters) == 1 and IsPod(first_output)) or
        first_output.kind == mojom.INT32):
        return GetNameForElement(first_output)
    return 'void'

def IsAsync(method):
    # Events are always async
    if re.match("^IPA.*EventInterface$", method.interface.mojom_name):
        return True
    elif re.match("^IPA.*Interface$", method.interface.mojom_name):
        if method.attributes is None:
            return False
        elif 'async' in method.attributes and method.attributes['async']:
            return True
    return False

def IsArray(element):
    return mojom.IsArrayKind(element.kind)

def IsControls(element):
    return mojom.IsStructKind(element.kind) and (element.kind.mojom_name == "ControlList" or
                                                 element.kind.mojom_name == "ControlInfoMap")

def IsEnum(element):
    return mojom.IsEnumKind(element.kind)


# Only works the enum definition, not types
def IsScoped(element):
    attributes = getattr(element, 'attributes', None)
    if not attributes:
        return False
    return 'scopedEnum' in attributes


def IsEnumScoped(element):
    if not IsEnum(element):
        return False
    return IsScoped(element.kind)

def IsFd(element):
    return mojom.IsStructKind(element.kind) and element.kind.mojom_name == "SharedFD"


def IsFlags(element):
    attributes = getattr(element, 'attributes', None)
    if not attributes:
        return False
    return 'flags' in attributes

def IsMap(element):
    return mojom.IsMapKind(element.kind)

def IsPlainStruct(element):
    return mojom.IsStructKind(element.kind) and not IsControls(element) and not IsFd(element)

def IsPod(element):
    return element.kind in _kind_to_cpp_type

def IsStr(element):
    return element.kind.spec == 's'

def BitWidth(element):
    if element.kind in _bit_widths:
        return _bit_widths[element.kind]
    if mojom.IsEnumKind(element.kind):
        return '32'
    return ''

def ByteWidthFromCppType(t):
    key = None
    for mojo_type, cpp_type in _kind_to_cpp_type.items():
        if t == cpp_type:
            key = mojo_type
    if key is None:
        raise Exception('invalid type')
    return str(int(_bit_widths[key]) // 8)

# Get the type name for a given element
def GetNameForElement(element):
    # Flags
    if IsFlags(element):
        return f'Flags<{GetFullNameForElement(element.kind)}>'
    # structs
    if (mojom.IsEnumKind(element) or
        mojom.IsInterfaceKind(element) or
        mojom.IsStructKind(element)):
        return element.mojom_name
    # vectors
    if (mojom.IsArrayKind(element)):
        elem_name = GetFullNameForElement(element.kind)
        return f'std::vector<{elem_name}>'
    # maps
    if (mojom.IsMapKind(element)):
        key_name = GetFullNameForElement(element.key_kind)
        value_name = GetFullNameForElement(element.value_kind)
        return f'std::map<{key_name}, {value_name}>'
    # struct fields and function parameters
    if isinstance(element, (mojom.Field, mojom.Method, mojom.Parameter)):
        # maps and vectors
        if (mojom.IsArrayKind(element.kind) or mojom.IsMapKind(element.kind)):
            return GetNameForElement(element.kind)
        # strings
        if (mojom.IsReferenceKind(element.kind) and element.kind.spec == 's'):
            return 'std::string'
        # PODs
        if element.kind in _kind_to_cpp_type:
            return _kind_to_cpp_type[element.kind]
        # structs and enums
        return element.kind.mojom_name
    # PODs that are members of vectors/maps
    if (hasattr(element, '__hash__') and element in _kind_to_cpp_type):
        return _kind_to_cpp_type[element]
    if (hasattr(element, 'spec')):
        # strings that are members of vectors/maps
        if (element.spec == 's'):
            return 'std::string'
        # structs that aren't defined in mojom that are members of vectors/maps
        if (element.spec[0] == 'x'):
            return element.spec.replace('x:', '').replace('.', '::')
    if (mojom.IsInterfaceRequestKind(element) or
        mojom.IsAssociatedKind(element) or
        mojom.IsPendingRemoteKind(element) or
        mojom.IsPendingReceiverKind(element) or
        mojom.IsUnionKind(element)):
        raise Exception('Unsupported element: %s' % element)
    raise Exception('Unexpected element: %s' % element)

def GetFullNameForElement(element):
    name = GetNameForElement(element)
    namespace_str = ''
    if (mojom.IsStructKind(element) or mojom.IsEnumKind(element)):
        namespace_str = element.module.mojom_namespace.replace('.', '::')
    elif (hasattr(element, 'kind') and
          (mojom.IsStructKind(element.kind) or mojom.IsEnumKind(element.kind))):
        namespace_str = element.kind.module.mojom_namespace.replace('.', '::')

    if namespace_str == '':
        return name

    if IsFlags(element):
        return GetNameForElement(element)

    return f'{namespace_str}::{name}'

def ValidateZeroLength(l, s, cap=True):
    if l is None:
        return
    if len(l) > 0:
        raise Exception(f'{s.capitalize() if cap else s} should be empty')

def ValidateSingleLength(l, s, cap=True):
    if len(l) > 1:
        raise Exception(f'Only one {s} allowed')
    if len(l) < 1:
        raise Exception(f'{s.capitalize() if cap else s} is required')

def GetMainInterface(interfaces):
    intf = [x for x in interfaces
            if re.match("^IPA.*Interface", x.mojom_name) and
               not re.match("^IPA.*EventInterface", x.mojom_name)]
    ValidateSingleLength(intf, 'main interface')
    return None if len(intf) == 0 else intf[0]

def GetEventInterface(interfaces):
    event = [x for x in interfaces if re.match("^IPA.*EventInterface", x.mojom_name)]
    ValidateSingleLength(event, 'event interface')
    return None if len(event) == 0 else event[0]

def ValidateNamespace(namespace):
    if namespace == '':
        raise Exception('Must have a namespace')

    if not re.match('^ipa\.[0-9A-Za-z_]+', namespace):
        raise Exception('Namespace must be of the form "ipa.{pipeline_name}"')

def ValidateInterfaces(interfaces):
    # Validate presence of main interface
    intf = GetMainInterface(interfaces)
    if intf is None:
        raise Exception('Must have main IPA interface')

    # Validate presence of event interface
    event = GetEventInterface(interfaces)
    if intf is None:
        raise Exception('Must have event IPA interface')

    # Validate required main interface functions
    f_init  = [x for x in intf.methods if x.mojom_name == 'init']
    f_start = [x for x in intf.methods if x.mojom_name == 'start']
    f_stop  = [x for x in intf.methods if x.mojom_name == 'stop']

    ValidateSingleLength(f_init, 'init()', False)
    ValidateSingleLength(f_start, 'start()', False)
    ValidateSingleLength(f_stop, 'stop()', False)

    f_stop  = f_stop[0]

    # No need to validate init() and start() as they are customizable

    # Validate parameters to stop()
    ValidateZeroLength(f_stop.parameters, 'input parameter to stop()')
    ValidateZeroLength(f_stop.parameters, 'output parameter from stop()')

    # Validate that event interface has at least one event
    if len(event.methods) < 1:
        raise Exception('Event interface must have at least one event')

    # Validate that all async methods don't have return values
    intf_methods_async = [x for x in intf.methods if IsAsync(x)]
    for method in intf_methods_async:
        ValidateZeroLength(method.response_parameters,
                           f'{method.mojom_name} response parameters', False)

    event_methods_async = [x for x in event.methods if IsAsync(x)]
    for method in event_methods_async:
        ValidateZeroLength(method.response_parameters,
                           f'{method.mojom_name} response parameters', False)

class Generator(generator.Generator):
    @staticmethod
    def GetTemplatePrefix():
        return 'libcamera_templates'

    def GetFilters(self):
        libcamera_filters = {
            'all_types': GetAllTypes,
            'bit_width': BitWidth,
            'byte_width' : ByteWidthFromCppType,
            'cap': Capitalize,
            'choose': Choose,
            'comma_sep': CommaSep,
            'default_value': GetDefaultValue,
            'has_default_fields': HasDefaultFields,
            'has_fd': HasFd,
            'is_async': IsAsync,
            'is_array': IsArray,
            'is_controls': IsControls,
            'is_enum': IsEnum,
            'is_enum_scoped': IsEnumScoped,
            'is_fd': IsFd,
            'is_flags': IsFlags,
            'is_map': IsMap,
            'is_plain_struct': IsPlainStruct,
            'is_pod': IsPod,
            'is_scoped': IsScoped,
            'is_str': IsStr,
            'method_input_has_fd': MethodInputHasFd,
            'method_output_has_fd': MethodOutputHasFd,
            'method_param_names': MethodParamNames,
            'method_param_inputs': MethodParamInputs,
            'method_param_outputs': MethodParamOutputs,
            'method_parameters': MethodParameters,
            'method_return_value': MethodReturnValue,
            'name': GetNameForElement,
            'name_full': GetFullNameForElement,
            'needs_control_serializer': NeedsControlSerializer,
            'params_comma_sep': ParamsCommaSep,
            'with_default_values': WithDefaultValues,
            'with_fds': WithFds,
        }
        return libcamera_filters

    def _GetJinjaExports(self):
        return {
            'cmd_enum_name': '_%sCmd' % self.module_name,
            'cmd_event_enum_name': '_%sEventCmd' % self.module_name,
            'consts': self.module.constants,
            'enums': self.module.enums,
            'has_array': len([x for x in self.module.kinds.keys() if x[0] == 'a']) > 0,
            'has_map': len([x for x in self.module.kinds.keys() if x[0] == 'm']) > 0,
            'has_namespace': self.module.mojom_namespace != '',
            'interface_event': GetEventInterface(self.module.interfaces),
            'interface_main': GetMainInterface(self.module.interfaces),
            'interface_name': 'IPA%sInterface' % self.module_name,
            'module_name': ModuleName(self.module.path),
            'namespace': self.module.mojom_namespace.split('.'),
            'namespace_str': self.module.mojom_namespace.replace('.', '::') if
                             self.module.mojom_namespace is not None else '',
            'proxy_name': 'IPAProxy%s' % self.module_name,
            'proxy_worker_name': 'IPAProxy%sWorker' % self.module_name,
            'structs_nonempty': [x for x in self.module.structs if len(x.fields) > 0],
        }

    def _GetJinjaExportsForCore(self):
        return {
            'consts': self.module.constants,
            'enums_gen_header': [x for x in self.module.enums if x.attributes is None or 'skipHeader' not in x.attributes],
            'has_array': len([x for x in self.module.kinds.keys() if x[0] == 'a']) > 0,
            'has_map': len([x for x in self.module.kinds.keys() if x[0] == 'm']) > 0,
            'structs_gen_header': [x for x in self.module.structs if x.attributes is None or 'skipHeader' not in x.attributes],
            'structs_gen_serializer': [x for x in self.module.structs if x.attributes is None or 'skipSerdes' not in x.attributes],
        }

    @UseJinja('core_ipa_interface.h.tmpl')
    def _GenerateCoreHeader(self):
        return self._GetJinjaExportsForCore()

    @UseJinja('core_ipa_serializer.h.tmpl')
    def _GenerateCoreSerializer(self):
        return self._GetJinjaExportsForCore()

    @UseJinja('module_ipa_interface.h.tmpl')
    def _GenerateDataHeader(self):
        return self._GetJinjaExports()

    @UseJinja('module_ipa_serializer.h.tmpl')
    def _GenerateSerializer(self):
        return self._GetJinjaExports()

    @UseJinja('module_ipa_proxy.cpp.tmpl')
    def _GenerateProxyCpp(self):
        return self._GetJinjaExports()

    @UseJinja('module_ipa_proxy.h.tmpl')
    def _GenerateProxyHeader(self):
        return self._GetJinjaExports()

    @UseJinja('module_ipa_proxy_worker.cpp.tmpl')
    def _GenerateProxyWorker(self):
        return self._GetJinjaExports()

    def GenerateFiles(self, unparsed_args):
        parser = argparse.ArgumentParser()
        parser.add_argument('--libcamera_generate_core_header',     action='store_true')
        parser.add_argument('--libcamera_generate_core_serializer', action='store_true')
        parser.add_argument('--libcamera_generate_header',          action='store_true')
        parser.add_argument('--libcamera_generate_serializer',      action='store_true')
        parser.add_argument('--libcamera_generate_proxy_cpp',       action='store_true')
        parser.add_argument('--libcamera_generate_proxy_h',         action='store_true')
        parser.add_argument('--libcamera_generate_proxy_worker',    action='store_true')
        parser.add_argument('--libcamera_output_path')
        args = parser.parse_args(unparsed_args)

        if not args.libcamera_generate_core_header and \
           not args.libcamera_generate_core_serializer:
            ValidateNamespace(self.module.mojom_namespace)
            ValidateInterfaces(self.module.interfaces)
            self.module_name = ModuleClassName(self.module)

        fileutil.EnsureDirectoryExists(os.path.dirname(args.libcamera_output_path))

        gen_funcs = [
                [args.libcamera_generate_core_header,     self._GenerateCoreHeader],
                [args.libcamera_generate_core_serializer, self._GenerateCoreSerializer],
                [args.libcamera_generate_header,          self._GenerateDataHeader],
                [args.libcamera_generate_serializer,      self._GenerateSerializer],
                [args.libcamera_generate_proxy_cpp,       self._GenerateProxyCpp],
                [args.libcamera_generate_proxy_h,         self._GenerateProxyHeader],
                [args.libcamera_generate_proxy_worker,    self._GenerateProxyWorker],
        ]

        for pair in gen_funcs:
            if pair[0]:
                self.Write(pair[1](), args.libcamera_output_path)
