Coding Style Guidelines
=======================

The libcamera project has high standards of stability, efficiency and
reliability. To achieve those, the project goes to great length to produce
code that is as easy to read, understand and maintain as possible.

These coding guidelines are meant to ensure code quality. As a contributor
you are expected to follow them in all code submitted to the project. While
strict compliance is desired, exceptions are tolerated when justified with
good reasons. Please read the whole coding guidelines and use common sense
to decide when departing from them is appropriate.

libcamera is written in C++, a language that has seen many revisions and
offers an extensive set of features that are easy to abuse. These coding
guidelines establish the subset of C++ used by the project.


Coding Style
------------

Even if the programming language in use is different, the project embraces the
`Linux Kernel Coding Style`_ with a few exception and some C++ specificities.

.. _Linux Kernel Coding Style: https://www.kernel.org/doc/html/latest/process/coding-style.html

In particular, from the kernel style document, the following section are adopted:

* 1 "Indentation"
* 2 "Breaking Long Lines" striving to fit code within 80 columns and
  accepting up to 120 columns when necessary
* 3 "Placing Braces and Spaces"
* 3.1 "Spaces"
* 8 "Commenting" with the exception that in-function comments are not
  always un-welcome.

While libcamera uses the kernel coding style for all typographic matters, the
project is a user space library, developed in a different programming language,
and the kernel guidelines fall short for this use case.

For this reason, rules and guidelines from the `Google C++ Style Guide`_ have
been adopted as well as most coding principles specified therein, with a
few exceptions and relaxed limitations on some subjects.

.. _Google C++ Style Guide: https://google.github.io/styleguide/cppguide.html

The following exceptions apply to the naming conventions specified in the
document:

* File names: libcamera uses the .cpp extensions for C++ source files and
  the .h extension for header files
* Variables, function parameters, function names and class members use
  camel case style, with the first letter in lower-case (as in 'camelCase'
  and not 'CamelCase')
* Types (classes, structs, type aliases, and type template parameters) use
  camel case, with the first letter in capital case (as in 'CamelCase' and
  not 'camelCase')
* Enum members use 'CamelCase', while macros are in capital case with
  underscores in between
* All formatting rules specified in the selected sections of the Linux kernel
  Code Style for indentation, braces, spacing, etc
* Header guards are formatted as '__LIBCAMERA_FILE_NAME_H__'


C++ Specific Rules
------------------

The code shall be implemented in C++03, extended with the following
C++-11-specific features:

* Initializer lists
* Type inference (auto and decltype)
  Type inference shall be used with caution, to avoid drifting towards an
  untyped language.
* Range-based for loop
* Lambda functions
* Explicit overrides and final
* Null pointer constant
* General-purpose smart pointers (std::unique_ptr), deprecating std::auto_ptr.
  Smart pointers, as well as shared pointers and weak pointers, shall not be
  overused.
