#!/usr/bin/awk -f

# SPDX-License-Identifier: LGPL-2.1-or-later

# Controls are documented using Doxygen in the main controls.cpp source.
#
# Generate control tables directly from the documentation, creating enumerations
# to support the IDs and static type information regarding each control.

BEGIN {
	id=0
	input=ARGV[1]
	mode=ARGV[2]
	output=ARGV[3]
}

# Detect Doxygen style comment blocks and ignore other lines
/^\/\*\*$/ { in_doxygen=1; first_line=1; next }
// { if (!in_doxygen) next }

# Entry point for the Control Documentation
/ * \\enum ControlId$/ { in_controls=1; first_line=0; next }
// { if (!in_controls) next }

# Extract control information
/ \* \\var/ { names[++id]=$3; first_line=0; next }
/ \* ControlType:/ { types[id] = $3 }

# End of comment blocks
/^ \*\// { in_doxygen=0 }

# Identify the end of controls
/^ \* \\/ { if (first_line) exit }
// { first_line=0 }

################################
# Support output file generation

function basename(file) {
	sub(".*/", "", file)
	return file
}

function Header(file, description) {
	print "/* SPDX-License-Identifier: LGPL-2.1-or-later */" > file
	print "/*" > file
	print " * Copyright (C) 2019, Google Inc." > file
	print " *" > file
	print " * " basename(file) " - " description > file
	print " *" > file
	print " * This file is auto-generated. Do not edit." > file
	print " */" > file
	print "" > file
}

function EnterNameSpace(file) {
	print "namespace libcamera {" > file
	print "" > file
}

function ExitNameSpace(file) {
	print "" > file
	print "} /* namespace libcamera */" > file
}

function GenerateHeader(file) {
	Header(file, "Control ID list")

	print "#ifndef __LIBCAMERA_CONTROL_IDS_H__" > file
	print "#define __LIBCAMERA_CONTROL_IDS_H__" > file
	print "" > file

	EnterNameSpace(file)
	print "enum ControlId {" > file
	for (i=1; i <= id; ++i) {
		printf "\t%s,\n", names[i] > file
	}
	print "};" > file
	ExitNameSpace(file)

	print "" > file
	print "#endif // __LIBCAMERA_CONTROL_IDS_H__" > file
}

function GenerateTable(file) {
	Header(file, "Control types")
	print "#include <libcamera/controls.h>" > file
	print "" > file

	EnterNameSpace(file)

	print "extern const std::unordered_map<ControlId, ControlIdentifier>" > file
	print "controlTypes {" > file
	for (i=1; i <= id; ++i) {
		printf "\t{ %s, { %s, \"%s\", ControlType%s } },\n", names[i], names[i], names[i], types[i] > file
	}
	print "};" > file
	ExitNameSpace(file)
}

END {
	if (mode == "--header")
		GenerateHeader(output)
	else
		GenerateTable(output)
}
