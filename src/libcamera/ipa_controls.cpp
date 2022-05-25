/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_controls.cpp - IPA control handling
 */

#include <libcamera/ipa/ipa_controls.h>

/**
 * \file ipa_controls.h
 * \brief Type definitions for serialized controls
 *
 * This file defines binary formats to store ControlList and ControlInfoMap
 * instances in contiguous, self-contained memory areas called control packets.
 * It describes the layout of the packets through a set of C structures. These
 * formats shall be used when serializing ControlList and ControlInfoMap to
 * transfer them through the IPA C interface and IPA IPC transports.
 *
 * A control packet contains a list of entries, each of them describing a single
 * control info or control value. The packet starts with a fixed-size header
 * described by the ipa_controls_header structure, followed by an array of
 * fixed-size entries. Each entry is associated with data, stored either
 * directly in the entry, or in a data section after the entries array.
 *
 * The following diagram describes the layout of the ControlList packet.
 *
 * ~~~~
 *           +-------------------------+    .                      .
 *  Header / | ipa_controls_header     |    |                      |
 *         | |                         |    |                      |
 *         \ |                         |    |                      |
 *           +-------------------------+    |                      |
 *         / | ipa_control_value_entry |    | hdr.data_offset      |
 *         | | #0                      |    |                      |
 * Control | +-------------------------+    |                      |
 *   value | | ...                     |    |                      |
 * entries | +-------------------------+    |                      |
 *         | | ipa_control_value_entry |    |             hdr.size |
 *         \ | #hdr.entries - 1        |    |                      |
 *           +-------------------------+    |                      |
 *           | empty space (optional)  |    |                      |
 *           +-------------------------+ <--´  .                   |
 *         / | ...                     |       | entry[n].offset   |
 *    Data | | ...                     |       |                   |
 * section | | value data for entry #n | <-----´                   |
 *         \ | ...                     |                           |
 *           +-------------------------+                           |
 *           | empty space (optional)  |                           |
 *           +-------------------------+ <-------------------------´
 * ~~~~
 *
 * The packet header contains the size of the packet, the number of entries, and
 * the offset from the beginning of the packet to the data section. The packet
 * entries array immediately follows the header. The data section starts at the
 * offset ipa_controls_header::data_offset from the beginning of the packet, and
 * shall be aligned to a multiple of 8 bytes.
 *
 * Entries are described by the ipa_control_value_entry structure. They contain
 * the numerical ID of the control, its type, and the number of control values.
 *
 * The control values are stored in the data section in the platform's native
 * format. The ipa_control_value_entry::offset field stores the offset from the
 * beginning of the data section to the values.
 *
 * All control values in the data section shall be stored in the same order as
 * the respective control entries, shall be aligned to a multiple of 8 bytes,
 * and shall be contiguous in memory.
 *
 * Empty spaces may be present between the end of the entries array and the
 * data section, and after the data section. They shall be ignored when parsing
 * the packet.
 *
 * The following diagram describes the layout of the ControlInfoMap packet.
 *
 * ~~~~
 *           +-------------------------+    .                      .
 *  Header / | ipa_controls_header     |    |                      |
 *         | |                         |    |                      |
 *         \ |                         |    |                      |
 *           +-------------------------+    |                      |
 *         / | ipa_control_info_entry  |    | hdr.data_offset      |
 *         | | #0                      |    |                      |
 * Control | +-------------------------+    |                      |
 *    info | | ...                     |    |                      |
 * entries | +-------------------------+    |                      |
 *         | | ipa_control_info_entry  |    |             hdr.size |
 *         \ | #hdr.entries - 1        |    |                      |
 *           +-------------------------+    |                      |
 *           | empty space (optional)  |    |                      |
 *           +-------------------------+ <--´  .                   |
 *         / | ...                     |       | entry[n].offset   |
 *    Data | | ...                     |       |                   |
 * section | | info data for entry #n  | <-----´                   |
 *         \ | ...                     |                           |
 *           +-------------------------+                           |
 *           | empty space (optional)  |                           |
 *           +-------------------------+ <-------------------------´
 * ~~~~
 *
 * The packet header is identical to the ControlList packet header.
 *
 * Entries are described by the ipa_control_info_entry structure. They contain
 * the numerical ID and type of the control. The control info data is stored
 * in the data section as described by the following diagram.
 *
 * ~~~~
 *           +-------------------------+       .
 *         / | ...                     |       | entry[n].offset
 *         | +-------------------------+ <-----´
 *         | | minimum value (#n)      | \
 *    Data | +-------------------------+ |
 * section | | maximum value (#n)      | | Entry #n
 *         | +-------------------------+ |
 *         | | default value (#n)      | /
 *         | +-------------------------+
 *         \ | ...                     |
 *           +-------------------------+
 * ~~~~
 *
 * The minimum, maximum and default values are stored in the platform's native
 * data format. The ipa_control_info_entry::offset field stores the offset from
 * the beginning of the data section to the info data.
 *
 * Info data in the data section shall be stored in the same order as the
 * entries array, shall be aligned to a multiple of 8 bytes, and shall be
 * contiguous in memory.
 *
 * As for the ControlList packet, empty spaces may be present between the end of
 * the entries array and the data section, and after the data section. They
 * shall be ignored when parsing the packet.
 */

namespace libcamera {

/**
 * \def IPA_CONTROLS_FORMAT_VERSION
 * \brief The current control serialization format version
 */

/**
 * \var ipa_controls_id_map_type
 * \brief Enumerates the different control id map types
 *
 * Each ControlInfoMap and ControlList refers to a control id map that
 * associates the ControlId references to a numerical identifier.
 * During the serialization procedure the raw pointers to the ControlId
 * instances cannot be transported on the wire, hence their numerical id is
 * used to identify them in the serialized data buffer. At deserialization time
 * it is required to associate back to the numerical id the ControlId instance
 * it represents. This enumeration describes which ControlIdMap should be
 * used to perform such operation.
 *
 * \var ipa_controls_id_map_type::IPA_CONTROL_ID_MAP_CONTROLS
 * \brief The numerical control identifier are resolved to a ControlId * using
 * the global controls::controls id map
 * \var ipa_controls_id_map_type::IPA_CONTROL_ID_MAP_PROPERTIES
 * \brief The numerical control identifier are resolved to a ControlId * using
 * the global properties::properties id map
 * \var ipa_controls_id_map_type::IPA_CONTROL_ID_MAP_V4L2
 * \brief ControlId for V4L2 defined controls are created by the video device
 * that enumerates them, and are not available across the IPC boundaries. The
 * deserializer shall create new ControlId instances for them as well as store
 * them in a dedicated ControlIdMap. Only lookup by numerical id can be
 * performed on de-serialized ControlInfoMap that represents V4L2 controls.
 */

/**
 * \struct ipa_controls_header
 * \brief Serialized control packet header
 * \var ipa_controls_header::version
 * Control packet format version number (shall be IPA_CONTROLS_FORMAT_VERSION)
 * \var ipa_controls_header::handle
 * For ControlInfoMap packets, this field contains a unique non-zero handle
 * generated when the ControlInfoMap is serialized. For ControlList packets,
 * this field contains the handle of the corresponding ControlInfoMap.
 * \var ipa_controls_header::entries
 * Number of entries in the packet
 * \var ipa_controls_header::size
 * The total packet size in bytes
 * \var ipa_controls_header::data_offset
 * Offset in bytes from the beginning of the packet of the data section start
 * \var ipa_controls_header::id_map_type
 * The id map type as defined by the ipa_controls_id_map_type enumeration
 * \var ipa_controls_header::reserved
 * Reserved for future extensions
 */

static_assert(sizeof(ipa_controls_header) == 32,
	      "Invalid ABI size change for struct ipa_control_header");

/**
 * \struct ipa_control_value_entry
 * \brief Description of a serialized ControlValue entry
 * \var ipa_control_value_entry::id
 * The numerical ID of the control
 * \var ipa_control_value_entry::type
 * The type of the control (defined by enum ControlType)
 * \var ipa_control_value_entry::is_array
 * True if the control value stores an array, false otherwise
 * \var ipa_control_value_entry::count
 * The number of control array entries for array controls (1 otherwise)
 * \var ipa_control_value_entry::offset
 * The offset in bytes from the beginning of the data section to the control
 * value data (shall be a multiple of 8 bytes).
 * \var ipa_control_value_entry::padding
 * Padding bytes (shall be set to 0)
 */

static_assert(sizeof(ipa_control_value_entry) == 16,
	      "Invalid ABI size change for struct ipa_control_value_entry");

/**
 * \struct ipa_control_info_entry
 * \brief Description of a serialized ControlInfo entry
 * \var ipa_control_info_entry::id
 * The numerical ID of the control
 * \var ipa_control_info_entry::type
 * The type of the control (defined by enum ControlType)
 * \var ipa_control_info_entry::offset
 * The offset in bytes from the beginning of the data section to the control
 * info data (shall be a multiple of 8 bytes)
 * \var ipa_control_info_entry::padding
 * Padding bytes (shall be set to 0)
 */

static_assert(sizeof(ipa_control_info_entry) == 16,
	      "Invalid ABI size change for struct ipa_control_info_entry");

} /* namespace libcamera */
