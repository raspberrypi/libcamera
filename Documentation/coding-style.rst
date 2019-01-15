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
* Variadic class and function templates


Tools
-----

The 'astyle' code formatting tool can be used to reformat source files with the
libcamera coding style, defined by the following arguments.

::

  --style=linux
  --indent=force-tab=8
  --attach-namespaces
  --attach-extern-c
  --pad-oper
  --align-pointer=name
  --align-reference=name
  --max-code-length=120

As astyle is a code formatter, it operates on full files outputs reformatted
source code. While it can be used to reformat code before sending patches, it
may generate unrelated changes. To avoid this, libcamera provides a
'checkstyle.py' script wrapping astyle to only retain related changes. This
should be used to validate modifications before submitting them for review.

The script operates on one or multiple git commits specified on the command
line. It does not modify the git tree, the index or the working directory and
is thus safe to run at any point.

Commits are specified using the same revision range syntax as 'git log'. The
most usual use cases are to specify a single commit by sha1, branch name or tag
name, or a commit range with the <from>..<to> syntax. When no arguments are
given, the topmost commit of the current branch is selected.

::

	$ ./utils/checkstyle.py cc7d204b2c51
	----------------------------------------------------------------------------------
	cc7d204b2c51853f7d963d144f5944e209e7ea29 libcamera: Use the logger instead of cout
	----------------------------------------------------------------------------------
	No style issue detected

When operating on a range of commits, style checks are performed on each commit
from oldest to newest.

::

	$ ../utils/checkstyle.py 3b56ddaa96fb~3..3b56ddaa96fb
	----------------------------------------------------------------------------------
	b4351e1a6b83a9cfbfc331af3753602a02dbe062 libcamera: log: Fix Doxygen documentation
	----------------------------------------------------------------------------------
	No style issue detected
	
	--------------------------------------------------------------------------------------
	6ab3ff4501fcfa24db40fcccbce35bdded7cd4bc libcamera: log: Document the LogMessage class
	--------------------------------------------------------------------------------------
	No style issue detected
	
	---------------------------------------------------------------------------------
	3b56ddaa96fbccf4eada05d378ddaa1cb6209b57 build: Add 'std=c++11' cpp compiler flag
	---------------------------------------------------------------------------------
	Commit doesn't touch source files, skipping

Commits that do not touch any .c, .cpp or .h files are skipped.

::

	$ ./utils/checkstyle.py edbd2059d8a4
	----------------------------------------------------------------------
	edbd2059d8a4bd759302ada4368fa4055638fd7f libcamera: Add initial logger
	----------------------------------------------------------------------
	--- src/libcamera/include/log.h
	+++ src/libcamera/include/log.h
	@@ -21,11 +21,14 @@
	 {
	 public:
	        LogMessage(const char *fileName, unsigned int line,
	-                 LogSeverity severity);
	-       LogMessage(const LogMessage&) = delete;
	+                  LogSeverity severity);
	+       LogMessage(const LogMessage &) = delete;
	        ~LogMessage();
	 
	-       std::ostream& stream() { return msgStream; }
	+       std::ostream &stream()
	+       {
	+               return msgStream;
	+       }
	 
	 private:
	        std::ostringstream msgStream;
	 
	--- src/libcamera/log.cpp
	+++ src/libcamera/log.cpp
	@@ -42,7 +42,7 @@
	 
	 static const char *log_severity_name(LogSeverity severity)
	 {
	-       static const char * const names[] = {
	+       static const char *const names[] = {
	                "INFO",
	                "WARN",
	                " ERR",
	
	---
	2 potential style issues detected, please review

When potential style issues are detected, they are displayed in the form of a
diff that fixes the issues, on top of the corresponding commit. As the script is
in early development false positive are expected. The flagged issues should be
reviewed, but the diff doesn't need to be applied blindly.

Happy hacking, libcamera awaits your patches!
