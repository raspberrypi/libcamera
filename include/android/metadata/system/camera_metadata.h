/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SYSTEM_MEDIA_INCLUDE_ANDROID_CAMERA_METADATA_H
#define SYSTEM_MEDIA_INCLUDE_ANDROID_CAMERA_METADATA_H

#include <string.h>
#include <stdint.h>
#include <cutils/compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Tag hierarchy and enum definitions for camera_metadata_entry
 * =============================================================================
 */

/**
 * Main enum definitions are in a separate file to make it easy to
 * maintain
 */
#include "camera_metadata_tags.h"

/**
 * Enum range for each top-level category
 */
ANDROID_API
extern unsigned int camera_metadata_section_bounds[ANDROID_SECTION_COUNT][2];
ANDROID_API
extern const char *camera_metadata_section_names[ANDROID_SECTION_COUNT];

/**
 * Type definitions for camera_metadata_entry
 * =============================================================================
 */
enum {
    // Unsigned 8-bit integer (uint8_t)
    TYPE_BYTE = 0,
    // Signed 32-bit integer (int32_t)
    TYPE_INT32 = 1,
    // 32-bit float (float)
    TYPE_FLOAT = 2,
    // Signed 64-bit integer (int64_t)
    TYPE_INT64 = 3,
    // 64-bit float (double)
    TYPE_DOUBLE = 4,
    // A 64-bit fraction (camera_metadata_rational_t)
    TYPE_RATIONAL = 5,
    // Number of type fields
    NUM_TYPES
};

typedef struct camera_metadata_rational {
    int32_t numerator;
    int32_t denominator;
} camera_metadata_rational_t;

/**
 * A reference to a metadata entry in a buffer.
 *
 * The data union pointers point to the real data in the buffer, and can be
 * modified in-place if the count does not need to change. The count is the
 * number of entries in data of the entry's type, not a count of bytes.
 */
typedef struct camera_metadata_entry {
    size_t   index;
    uint32_t tag;
    uint8_t  type;
    size_t   count;
    union {
        uint8_t *u8;
        int32_t *i32;
        float   *f;
        int64_t *i64;
        double  *d;
        camera_metadata_rational_t *r;
    } data;
} camera_metadata_entry_t;

/**
 * A read-only reference to a metadata entry in a buffer. Identical to
 * camera_metadata_entry in layout
 */
typedef struct camera_metadata_ro_entry {
    size_t   index;
    uint32_t tag;
    uint8_t  type;
    size_t   count;
    union {
        const uint8_t *u8;
        const int32_t *i32;
        const float   *f;
        const int64_t *i64;
        const double  *d;
        const camera_metadata_rational_t *r;
    } data;
} camera_metadata_ro_entry_t;

/**
 * Size in bytes of each entry type
 */
ANDROID_API
extern const size_t camera_metadata_type_size[NUM_TYPES];

/**
 * Human-readable name of each entry type
 */
ANDROID_API
extern const char* camera_metadata_type_names[NUM_TYPES];

/**
 * Main definitions for the metadata entry and array structures
 * =============================================================================
 */

/**
 * A packet of metadata. This is a list of metadata entries, each of which has
 * an integer tag to identify its meaning, 'type' and 'count' field, and the
 * data, which contains a 'count' number of entries of type 'type'. The packet
 * has a fixed capacity for entries and for extra data.  A new entry uses up one
 * entry slot, and possibly some amount of data capacity; the function
 * calculate_camera_metadata_entry_data_size() provides the amount of data
 * capacity that would be used up by an entry.
 *
 * Entries are not sorted by default, and are not forced to be unique - multiple
 * entries with the same tag are allowed. The packet will not dynamically resize
 * when full.
 *
 * The packet is contiguous in memory, with size in bytes given by
 * get_camera_metadata_size(). Therefore, it can be copied safely with memcpy()
 * to a buffer of sufficient size. The copy_camera_metadata() function is
 * intended for eliminating unused capacity in the destination packet.
 */
struct camera_metadata;
typedef struct camera_metadata camera_metadata_t;

/**
 * Functions for manipulating camera metadata
 * =============================================================================
 *
 * NOTE: Unless otherwise specified, functions that return type "int"
 * return 0 on success, and non-0 value on error.
 */

/**
 * Allocate a new camera_metadata structure, with some initial space for entries
 * and extra data. The entry_capacity is measured in entry counts, and
 * data_capacity in bytes. The resulting structure is all contiguous in memory,
 * and can be freed with free_camera_metadata().
 */
ANDROID_API
camera_metadata_t *allocate_camera_metadata(size_t entry_capacity,
        size_t data_capacity);

/**
 * Get the required alignment of a packet of camera metadata, which is the
 * maximal alignment of the embedded camera_metadata, camera_metadata_buffer_entry,
 * and camera_metadata_data.
 */
ANDROID_API
size_t get_camera_metadata_alignment();

/**
 * Allocate a new camera_metadata structure of size src_size. Copy the data,
 * ignoring alignment, and then attempt validation. If validation
 * fails, free the memory and return NULL. Otherwise return the pointer.
 *
 * The resulting pointer can be freed with free_camera_metadata().
 */
ANDROID_API
camera_metadata_t *allocate_copy_camera_metadata_checked(
        const camera_metadata_t *src,
        size_t src_size);

/**
 * Place a camera metadata structure into an existing buffer. Returns NULL if
 * the buffer is too small for the requested number of reserved entries and
 * bytes of data. The entry_capacity is measured in entry counts, and
 * data_capacity in bytes. If the buffer is larger than the required space,
 * unused space will be left at the end. If successful, returns a pointer to the
 * metadata header placed at the start of the buffer. It is the caller's
 * responsibility to free the original buffer; do not call
 * free_camera_metadata() with the returned pointer.
 */
ANDROID_API
camera_metadata_t *place_camera_metadata(void *dst, size_t dst_size,
        size_t entry_capacity,
        size_t data_capacity);

/**
 * Free a camera_metadata structure. Should only be used with structures
 * allocated with allocate_camera_metadata().
 */
ANDROID_API
void free_camera_metadata(camera_metadata_t *metadata);

/**
 * Calculate the buffer size needed for a metadata structure of entry_count
 * metadata entries, needing a total of data_count bytes of extra data storage.
 */
ANDROID_API
size_t calculate_camera_metadata_size(size_t entry_count,
        size_t data_count);

/**
 * Get current size of entire metadata structure in bytes, including reserved
 * but unused space.
 */
ANDROID_API
size_t get_camera_metadata_size(const camera_metadata_t *metadata);

/**
 * Get size of entire metadata buffer in bytes, not including reserved but
 * unused space. This is the amount of space needed by copy_camera_metadata for
 * its dst buffer.
 */
ANDROID_API
size_t get_camera_metadata_compact_size(const camera_metadata_t *metadata);

/**
 * Get the current number of entries in the metadata packet.
 *
 * metadata packet must be valid, which can be checked before the call with
 * validate_camera_metadata_structure().
 */
ANDROID_API
size_t get_camera_metadata_entry_count(const camera_metadata_t *metadata);

/**
 * Get the maximum number of entries that could fit in the metadata packet.
 */
ANDROID_API
size_t get_camera_metadata_entry_capacity(const camera_metadata_t *metadata);

/**
 * Get the current count of bytes used for value storage in the metadata packet.
 */
ANDROID_API
size_t get_camera_metadata_data_count(const camera_metadata_t *metadata);

/**
 * Get the maximum count of bytes that could be used for value storage in the
 * metadata packet.
 */
ANDROID_API
size_t get_camera_metadata_data_capacity(const camera_metadata_t *metadata);

/**
 * Copy a metadata structure to a memory buffer, compacting it along the
 * way. That is, in the copied structure, entry_count == entry_capacity, and
 * data_count == data_capacity.
 *
 * If dst_size > get_camera_metadata_compact_size(), the unused bytes are at the
 * end of the buffer. If dst_size < get_camera_metadata_compact_size(), returns
 * NULL. Otherwise returns a pointer to the metadata structure header placed at
 * the start of dst.
 *
 * Since the buffer was not allocated by allocate_camera_metadata, the caller is
 * responsible for freeing the underlying buffer when needed; do not call
 * free_camera_metadata.
 */
ANDROID_API
camera_metadata_t *copy_camera_metadata(void *dst, size_t dst_size,
        const camera_metadata_t *src);


// Non-zero return values for validate_camera_metadata_structure
enum {
    CAMERA_METADATA_VALIDATION_ERROR = 1,
    CAMERA_METADATA_VALIDATION_SHIFTED = 2,
};

/**
 * Validate that a metadata is structurally sane. That is, its internal
 * state is such that we won't get buffer overflows or run into other
 * 'impossible' issues when calling the other API functions.
 *
 * This is useful in particular after copying the binary metadata blob
 * from an untrusted source, since passing this check means the data is at least
 * consistent.
 *
 * The expected_size argument is optional.
 *
 * Returns 0: on success
 *         CAMERA_METADATA_VALIDATION_ERROR: on error
 *         CAMERA_METADATA_VALIDATION_SHIFTED: when the data is not properly aligned, but can be
 *                 used as input of clone_camera_metadata and the returned metadata will be valid.
 *
 */
ANDROID_API
int validate_camera_metadata_structure(const camera_metadata_t *metadata,
                                       const size_t *expected_size);

/**
 * Append camera metadata in src to an existing metadata structure in dst.  This
 * does not resize the destination structure, so if it is too small, a non-zero
 * value is returned. On success, 0 is returned. Appending onto a sorted
 * structure results in a non-sorted combined structure.
 */
ANDROID_API
int append_camera_metadata(camera_metadata_t *dst, const camera_metadata_t *src);

/**
 * Clone an existing metadata buffer, compacting along the way. This is
 * equivalent to allocating a new buffer of the minimum needed size, then
 * appending the buffer to be cloned into the new buffer. The resulting buffer
 * can be freed with free_camera_metadata(). Returns NULL if cloning failed.
 */
ANDROID_API
camera_metadata_t *clone_camera_metadata(const camera_metadata_t *src);

/**
 * Calculate the number of bytes of extra data a given metadata entry will take
 * up. That is, if entry of 'type' with a payload of 'data_count' values is
 * added, how much will the value returned by get_camera_metadata_data_count()
 * be increased? This value may be zero, if no extra data storage is needed.
 */
ANDROID_API
size_t calculate_camera_metadata_entry_data_size(uint8_t type,
        size_t data_count);

/**
 * Add a metadata entry to a metadata structure. Returns 0 if the addition
 * succeeded. Returns a non-zero value if there is insufficient reserved space
 * left to add the entry, or if the tag is unknown.  data_count is the number of
 * entries in the data array of the tag's type, not a count of
 * bytes. Vendor-defined tags can not be added using this method, unless
 * set_vendor_tag_query_ops() has been called first. Entries are always added to
 * the end of the structure (highest index), so after addition, a
 * previously-sorted array will be marked as unsorted.
 *
 * Returns 0 on success. A non-0 value is returned on error.
 */
ANDROID_API
int add_camera_metadata_entry(camera_metadata_t *dst,
        uint32_t tag,
        const void *data,
        size_t data_count);

/**
 * Sort the metadata buffer for fast searching. If already marked as sorted,
 * does nothing. Adding or appending entries to the buffer will place the buffer
 * back into an unsorted state.
 *
 * Returns 0 on success. A non-0 value is returned on error.
 */
ANDROID_API
int sort_camera_metadata(camera_metadata_t *dst);

/**
 * Get metadata entry at position index in the metadata buffer.
 * Index must be less than entry count, which is returned by
 * get_camera_metadata_entry_count().
 *
 * src and index are inputs; the passed-in entry is updated with the details of
 * the entry. The data pointer points to the real data in the buffer, and can be
 * updated as long as the data count does not change.
 *
 * Returns 0 on success. A non-0 value is returned on error.
 */
ANDROID_API
int get_camera_metadata_entry(camera_metadata_t *src,
        size_t index,
        camera_metadata_entry_t *entry);

/**
 * Get metadata entry at position index, but disallow editing the data.
 */
ANDROID_API
int get_camera_metadata_ro_entry(const camera_metadata_t *src,
        size_t index,
        camera_metadata_ro_entry_t *entry);

/**
 * Find an entry with given tag value. If not found, returns -ENOENT. Otherwise,
 * returns entry contents like get_camera_metadata_entry.
 *
 * If multiple entries with the same tag exist, does not have any guarantees on
 * which is returned. To speed up searching for tags, sort the metadata
 * structure first by calling sort_camera_metadata().
 */
ANDROID_API
int find_camera_metadata_entry(camera_metadata_t *src,
        uint32_t tag,
        camera_metadata_entry_t *entry);

/**
 * Find an entry with given tag value, but disallow editing the data
 */
ANDROID_API
int find_camera_metadata_ro_entry(const camera_metadata_t *src,
        uint32_t tag,
        camera_metadata_ro_entry_t *entry);

/**
 * Delete an entry at given index. This is an expensive operation, since it
 * requires repacking entries and possibly entry data. This also invalidates any
 * existing camera_metadata_entry.data pointers to this buffer. Sorting is
 * maintained.
 */
ANDROID_API
int delete_camera_metadata_entry(camera_metadata_t *dst,
        size_t index);

/**
 * Updates a metadata entry with new data. If the data size is changing, may
 * need to adjust the data array, making this an O(N) operation. If the data
 * size is the same or still fits in the entry space, this is O(1). Maintains
 * sorting, but invalidates camera_metadata_entry instances that point to the
 * updated entry. If a non-NULL value is passed in to entry, the entry structure
 * is updated to match the new buffer state.  Returns a non-zero value if there
 * is no room for the new data in the buffer.
 */
ANDROID_API
int update_camera_metadata_entry(camera_metadata_t *dst,
        size_t index,
        const void *data,
        size_t data_count,
        camera_metadata_entry_t *updated_entry);

/**
 * Retrieve human-readable name of section the tag is in. Returns NULL if
 * no such tag is defined. Returns NULL for tags in the vendor section, unless
 * set_vendor_tag_query_ops() has been used.
 */
ANDROID_API
const char *get_camera_metadata_section_name(uint32_t tag);

/**
 * Retrieve human-readable name of tag (not including section). Returns NULL if
 * no such tag is defined. Returns NULL for tags in the vendor section, unless
 * set_vendor_tag_query_ops() has been used.
 */
ANDROID_API
const char *get_camera_metadata_tag_name(uint32_t tag);

/**
 * Retrieve the type of a tag. Returns -1 if no such tag is defined. Returns -1
 * for tags in the vendor section, unless set_vendor_tag_query_ops() has been
 * used.
 */
ANDROID_API
int get_camera_metadata_tag_type(uint32_t tag);

/**
 * Retrieve human-readable name of section the tag is in. Returns NULL if
 * no such tag is defined.
 */
ANDROID_API
const char *get_local_camera_metadata_section_name(uint32_t tag,
        const camera_metadata_t *meta);

/**
 * Retrieve human-readable name of tag (not including section). Returns NULL if
 * no such tag is defined.
 */
ANDROID_API
const char *get_local_camera_metadata_tag_name(uint32_t tag,
        const camera_metadata_t *meta);

/**
 * Retrieve the type of a tag. Returns -1 if no such tag is defined.
 */
ANDROID_API
int get_local_camera_metadata_tag_type(uint32_t tag,
        const camera_metadata_t *meta);

/**
 * Set up vendor-specific tag query methods. These are needed to properly add
 * entries with vendor-specified tags and to use the
 * get_camera_metadata_section_name, _tag_name, and _tag_type methods with
 * vendor tags. Returns 0 on success.
 *
 * **DEPRECATED** - Please use vendor_tag_ops defined in camera_vendor_tags.h
 *        instead.
 */
typedef struct vendor_tag_query_ops vendor_tag_query_ops_t;
struct vendor_tag_query_ops {
    /**
     * Get vendor section name for a vendor-specified entry tag. Only called for
     * tags >= 0x80000000. The section name must start with the name of the
     * vendor in the Java package style. For example, CameraZoom inc must prefix
     * their sections with "com.camerazoom." Must return NULL if the tag is
     * outside the bounds of vendor-defined sections.
     */
    const char *(*get_camera_vendor_section_name)(
        const vendor_tag_query_ops_t *v,
        uint32_t tag);
    /**
     * Get tag name for a vendor-specified entry tag. Only called for tags >=
     * 0x80000000. Must return NULL if the tag is outside the bounds of
     * vendor-defined sections.
     */
    const char *(*get_camera_vendor_tag_name)(
        const vendor_tag_query_ops_t *v,
        uint32_t tag);
    /**
     * Get tag type for a vendor-specified entry tag. Only called for tags >=
     * 0x80000000. Must return -1 if the tag is outside the bounds of
     * vendor-defined sections.
     */
    int (*get_camera_vendor_tag_type)(
        const vendor_tag_query_ops_t *v,
        uint32_t tag);
    /**
     * Get the number of vendor tags supported on this platform. Used to
     * calculate the size of buffer needed for holding the array of all tags
     * returned by get_camera_vendor_tags().
     */
    int (*get_camera_vendor_tag_count)(
        const vendor_tag_query_ops_t *v);
    /**
     * Fill an array with all the supported vendor tags on this platform.
     * get_camera_vendor_tag_count() returns the number of tags supported, and
     * tag_array should be allocated with enough space to hold all of the tags.
     */
    void (*get_camera_vendor_tags)(
        const vendor_tag_query_ops_t *v,
        uint32_t *tag_array);
};

/**
 * **DEPRECATED** - This should only be used by the camera framework. Camera
 *      metadata will transition to using vendor_tag_ops defined in
 *      camera_vendor_tags.h instead.
 */
ANDROID_API
int set_camera_metadata_vendor_tag_ops(const vendor_tag_query_ops_t *query_ops);

/**
 * Print fields in the metadata to the log.
 * verbosity = 0: Only tag entry information
 * verbosity = 1: Tag entry information plus at most 16 data values
 * verbosity = 2: All information
 */
ANDROID_API
void dump_camera_metadata(const camera_metadata_t *metadata,
        int fd,
        int verbosity);

/**
 * Print fields in the metadata to the log; adds indentation parameter, which
 * specifies the number of spaces to insert before each line of the dump
 */
ANDROID_API
void dump_indented_camera_metadata(const camera_metadata_t *metadata,
        int fd,
        int verbosity,
        int indentation);

/**
 * Prints the specified tag value as a string. Only works for enum tags.
 * Returns 0 on success, -1 on failure.
 */
ANDROID_API
int camera_metadata_enum_snprint(uint32_t tag,
                                 uint32_t value,
                                 char *dst,
                                 size_t size);

#ifdef __cplusplus
}
#endif

#endif
