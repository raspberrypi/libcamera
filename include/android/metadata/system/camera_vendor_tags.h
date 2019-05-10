/*
 * Copyright 2014 The Android Open Source Project
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

#ifndef SYSTEM_MEDIA_INCLUDE_ANDROID_CAMERA_VENDOR_TAGS_H
#define SYSTEM_MEDIA_INCLUDE_ANDROID_CAMERA_VENDOR_TAGS_H

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_METADATA_VENDOR_TAG_BOUNDARY 0x80000000u
#define CAMERA_METADATA_INVALID_VENDOR_ID UINT64_MAX

typedef uint64_t metadata_vendor_id_t;

/**
 * Vendor tags:
 *
 * This structure contains basic functions for enumerating an immutable set of
 * vendor-defined camera metadata tags, and querying static information about
 * their structure/type.  The intended use of this information is to validate
 * the structure of metadata returned by the camera HAL, and to allow vendor-
 * defined metadata tags to be visible in application facing camera API.
 */
typedef struct vendor_tag_ops vendor_tag_ops_t;
struct vendor_tag_ops {
    /**
     * Get the number of vendor tags supported on this platform. Used to
     * calculate the size of buffer needed for holding the array of all tags
     * returned by get_all_tags().  This must return -1 on error.
     */
    int (*get_tag_count)(const vendor_tag_ops_t *v);

    /**
     * Fill an array with all of the supported vendor tags on this platform.
     * get_tag_count() must return the number of tags supported, and
     * tag_array will be allocated with enough space to hold the number of tags
     * returned by get_tag_count().
     */
    void (*get_all_tags)(const vendor_tag_ops_t *v, uint32_t *tag_array);

    /**
     * Get the vendor section name for a vendor-specified entry tag. This will
     * only be called for vendor-defined tags.
     *
     * The naming convention for the vendor-specific section names should
     * follow a style similar to the Java package style.  For example,
     * CameraZoom Inc. must prefix their sections with "com.camerazoom."
     * This must return NULL if the tag is outside the bounds of
     * vendor-defined sections.
     *
     * There may be different vendor-defined tag sections, for example the
     * phone maker, the chipset maker, and the camera module maker may each
     * have their own "com.vendor."-prefixed section.
     *
     * The memory pointed to by the return value must remain valid for the
     * lifetime of the module, and is owned by the module.
     */
    const char *(*get_section_name)(const vendor_tag_ops_t *v, uint32_t tag);

    /**
     * Get the tag name for a vendor-specified entry tag. This is only called
     * for vendor-defined tags, and must return NULL if it is not a
     * vendor-defined tag.
     *
     * The memory pointed to by the return value must remain valid for the
     * lifetime of the module, and is owned by the module.
     */
    const char *(*get_tag_name)(const vendor_tag_ops_t *v, uint32_t tag);

    /**
     * Get tag type for a vendor-specified entry tag. The type returned must be
     * a valid type defined in camera_metadata.h.  This method is only called
     * for tags >= CAMERA_METADATA_VENDOR_TAG_BOUNDARY, and must return
     * -1 if the tag is outside the bounds of the vendor-defined sections.
     */
    int (*get_tag_type)(const vendor_tag_ops_t *v, uint32_t tag);

    /* Reserved for future use.  These must be initialized to NULL. */
    void* reserved[8];
};

struct vendor_tag_cache_ops {
    /**
     * Get the number of vendor tags supported on this platform. Used to
     * calculate the size of buffer needed for holding the array of all tags
     * returned by get_all_tags().  This must return -1 on error.
     */
    int (*get_tag_count)(metadata_vendor_id_t id);

    /**
     * Fill an array with all of the supported vendor tags on this platform.
     * get_tag_count() must return the number of tags supported, and
     * tag_array will be allocated with enough space to hold the number of tags
     * returned by get_tag_count().
     */
    void (*get_all_tags)(uint32_t *tag_array, metadata_vendor_id_t id);

    /**
     * Get the vendor section name for a vendor-specified entry tag. This will
     * only be called for vendor-defined tags.
     *
     * The naming convention for the vendor-specific section names should
     * follow a style similar to the Java package style.  For example,
     * CameraZoom Inc. must prefix their sections with "com.camerazoom."
     * This must return NULL if the tag is outside the bounds of
     * vendor-defined sections.
     *
     * There may be different vendor-defined tag sections, for example the
     * phone maker, the chipset maker, and the camera module maker may each
     * have their own "com.vendor."-prefixed section.
     *
     * The memory pointed to by the return value must remain valid for the
     * lifetime of the module, and is owned by the module.
     */
    const char *(*get_section_name)(uint32_t tag, metadata_vendor_id_t id);

    /**
     * Get the tag name for a vendor-specified entry tag. This is only called
     * for vendor-defined tags, and must return NULL if it is not a
     * vendor-defined tag.
     *
     * The memory pointed to by the return value must remain valid for the
     * lifetime of the module, and is owned by the module.
     */
    const char *(*get_tag_name)(uint32_t tag, metadata_vendor_id_t id);

    /**
     * Get tag type for a vendor-specified entry tag. The type returned must be
     * a valid type defined in camera_metadata.h.  This method is only called
     * for tags >= CAMERA_METADATA_VENDOR_TAG_BOUNDARY, and must return
     * -1 if the tag is outside the bounds of the vendor-defined sections.
     */
    int (*get_tag_type)(uint32_t tag, metadata_vendor_id_t id);

    /* Reserved for future use.  These must be initialized to NULL. */
    void* reserved[8];
};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SYSTEM_MEDIA_INCLUDE_ANDROID_CAMERA_VENDOR_TAGS_H */

