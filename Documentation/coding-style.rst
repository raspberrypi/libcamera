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
* rvalue references, move constructor and move assignment

Object Ownership
~~~~~~~~~~~~~~~~

libcamera creates and destroys many objects at runtime, for both objects
internal to the library and objects exposed to the user. To guarantee proper
operation without use after free, double free or memory leaks, knowing who owns
each object at any time is crucial. The project has enacted a set of rules to
make object ownership tracking as explicit and fool-proof as possible.

In the context of this section, the terms object and instance are used
interchangeably and both refer to an instance of a class. The term reference
refers to both C++ references and C++ pointers in their capacity to refer to an
object. Passing a reference means offering a way to a callee to obtain a
reference to an object that the caller has a valid reference to. Borrowing a
reference means using a reference passed by a caller without ownership transfer
based on the assumption that the caller guarantees the validity of the
reference for the duration of the operation that borrows it.

#. Single Owner Objects

   * By default an object has a single owner at any time.
   * Storage of single owner objects varies depending on how the object
     ownership will evolve through the lifetime of the object.

     * Objects whose ownership needs to be transferred shall be stored as
       std::unique_ptr<> as much as possible to emphasize the single ownership.
     * Objects whose owner doesn't change may be embedded in other objects, or
       stored as pointer or references. They may be stored as std::unique_ptr<>
       for automatic deletion if desired.

   * Ownership is transferred by passing the reference as a std::unique_ptr<>
     and using std::move(). After ownership transfer the former owner has no
     valid reference to the object anymore and shall not access it without first
     obtaining a valid reference.
   * Objects may be borrowed by passing an object reference from the owner to
     the borrower, providing that

     * the owner guarantees the validity of the reference for the whole duration
       of the borrowing, and
     * the borrower doesn't access the reference after the end of the borrowing.

     When borrowing from caller to callee for the duration of a function call,
     this implies that the callee shall not keep any stored reference after it
     returns. These rules apply to the callee and all the functions it calls,
     directly or indirectly.

     When the object is stored in a std::unique_ptr<>, borrowing passes a
     reference to the object, not to the std::unique_ptr<>, as

     * a 'const &' when the object doesn't need to be modified and may not be
       null.
     * a pointer when the object may be modified or may be null. Unless
       otherwise specified, pointers passed to functions are considered as
       borrowed references valid for the duration of the function only.

#. Shared Objects

   * Objects that may have multiple owners at a given time are called shared
     objects. They are reference-counted and live as long as any references to
     the object exist.
   * Shared objects are created with std::make_shared<> or
     std::allocate_shared<> and stored in an std::shared_ptr<>.
   * Ownership is shared by creating and passing copies of any valid
     std::shared_ptr<>. Ownership is released by destroying the corresponding
     std::shared_ptr<>.
   * When passed to a function, std::shared_ptr<> are always passed by value,
     never by reference. The caller can decide whether to transfer its ownership
     of the std::shared_ptr<> with std::move() or retain it. The callee shall
     use std::move() if it needs to store the shared pointer.
   * Do not over-use std::move(), as it may prevent copy-elision. In particular
     a function returning a std::shared_ptr<> value shall not use std::move() in
     its return statements, and its callers shall not wrap the function call
     with std::move().
   * Borrowed references to shared objects are passed as references to the
     objects themselves, not to the std::shared_ptr<>, with the same rules as
     for single owner objects.

These rules match the `object ownership rules from the Chromium C++ Style Guide`_.

.. _object ownership rules from the Chromium C++ Style Guide: https://chromium.googlesource.com/chromium/src/+/master/styleguide/c++/c++.md#object-ownership-and-calling-conventions

.. attention:: Long term borrowing of single owner objects is allowed. Example
   use cases are implementation of the singleton pattern (where the singleton
   guarantees the validity of the reference forever), or returning references
   to global objects whose lifetime matches the lifetime of the application. As
   long term borrowing isn't marked through language constructs, it shall be
   documented explicitly in details in the API.


Documentation
-------------

All public and protected classes, structures, enumerations, macros, functions
and variables shall be documented with a Doxygen comment block, using the
Javadoc style with C-style comments. When documenting private member functions
and variables the same Doxygen style shall be used as for public and protected
members.

Documentation relates to header files, but shall be stored in the .cpp source
files in order to group the implementation and documentation. Every documented
header file shall have a \file documentation block in the .cpp source file.

The following comment block shows an example of correct documentation for a
member function of the PipelineHandler class.

::

  /**
   * \fn PipelineHandler::start()
   * \brief Start capturing from a group of streams
   * \param[in] camera The camera to start
   *
   * Start the group of streams that have been configured for capture by
   * \a configureStreams(). The intended caller of this method is the Camera
   * class which will in turn be called from the application to indicate that
   * it has configured the streams and is ready to capture.
   *
   * \return 0 on success or a negative error code otherwise
   */

The comment block shall be placed right before the function it documents. If
the function is defined inline in the class definition in the header file, the
comment block shall be placed alone in the .cpp source file in the same order
as the function definitions in the header file and shall start with an \fn
line. Otherwise no \fn line shall be present.

The \brief directive shall be present. If the function takes parameters, \param
directives shall be present, with the appropriate [in], [out] or [inout]
specifiers. Only when the direction of the parameters isn't known (for instance
when defining a template function with variadic arguments) the direction
specifier shall be omitted. The \return directive shall be present when the
function returns a value, and shall be omitted otherwise.

The long description is optional. When present it shall be surrounded by empty
lines and may span multiple paragraphs. No blank lines shall otherwise be added
between the \fn, \brief, \param and \return directives.


Tools
-----

The 'clang-format' code formatting tool can be used to reformat source files
with the libcamera coding style, defined in the .clang-format file at the root
of the source tree.

Alternatively the 'astyle' tool can also be used, with the following arguments.

::

  --style=linux
  --indent=force-tab=8
  --attach-namespaces
  --attach-extern-c
  --pad-oper
  --align-pointer=name
  --align-reference=name
  --max-code-length=120

Use of astyle is discouraged as clang-format better matches the libcamera coding
style.

As both astyle and clang-format are code formatters, they operate on full files
and output reformatted source code. While they can be used to reformat code
before sending patches, it may generate unrelated changes. To avoid this,
libcamera provides a 'checkstyle.py' script wrapping the formatting tools to
only retain related changes. This should be used to validate modifications
before submitting them for review.

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

The checkstyle.py script uses clang-format by default if found, and otherwise
falls back to astyle. The formatter can be manually selected with the
'--formatter' argument.

Happy hacking, libcamera awaits your patches!
