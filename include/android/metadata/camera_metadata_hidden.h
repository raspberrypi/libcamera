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

#ifndef SYSTEM_MEDIA_PRIVATE_INCLUDE_CAMERA_METADATA_HIDDEN_H
#define SYSTEM_MEDIA_PRIVATE_INCLUDE_CAMERA_METADATA_HIDDEN_H

#include <system/camera_vendor_tags.h>

/**
 * Error codes returned by vendor tags ops operations. These are intended
 * to be used by all framework code that uses the return values from the
 * vendor operations object.
 */
#define VENDOR_SECTION_NAME_ERR   NULL
#define VENDOR_TAG_NAME_ERR       NULL
#define VENDOR_TAG_COUNT_ERR      (-1)
#define VENDOR_TAG_TYPE_ERR       (-1)

#ifdef __cplusplus
extern "C" {
#endif
/** **These are private functions for use only by the camera framework.** **/

/**
 * Set the global vendor tag operations object used to define vendor tag
 * structure when parsing camera metadata with functions defined in
 * system/media/camera/include/camera_metadata.h.
 */
ANDROID_API
int set_camera_metadata_vendor_ops(const vendor_tag_ops_t *query_ops);

/**
 * Set the global vendor tag cache operations object used to define vendor tag
 * structure when parsing camera metadata with functions defined in
 * system/media/camera/include/camera_metadata.h.
 */
ANDROID_API
int set_camera_metadata_vendor_cache_ops(
        const struct vendor_tag_cache_ops *query_cache_ops);

/**
 * Set the vendor id for a particular metadata buffer.
 */
ANDROID_API
void set_camera_metadata_vendor_id(camera_metadata_t *meta,
        metadata_vendor_id_t id);

/**
 * Retrieve the vendor id for a particular metadata buffer.
 */
ANDROID_API
metadata_vendor_id_t get_camera_metadata_vendor_id(
        const camera_metadata_t *meta);

/**
 * Retrieve the type of a tag. Returns -1 if no such tag is defined.
 */
ANDROID_API
int get_local_camera_metadata_tag_type_vendor_id(uint32_t tag,
        metadata_vendor_id_t id);

/**
 * Retrieve the name of a tag. Returns NULL if no such tag is defined.
 */
ANDROID_API
const char *get_local_camera_metadata_tag_name_vendor_id(uint32_t tag,
        metadata_vendor_id_t id);

/**
 * Retrieve the name of a tag section. Returns NULL if no such tag is defined.
 */
ANDROID_API
const char *get_local_camera_metadata_section_name_vendor_id(uint32_t tag,
        metadata_vendor_id_t id);

/**
 * Retrieve the type of a tag. Returns -1 if no such tag is defined.
 */
ANDROID_API
int get_local_camera_metadata_tag_type_vendor_id(uint32_t tag,
        metadata_vendor_id_t id);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SYSTEM_MEDIA_PRIVATE_INCLUDE_CAMERA_METADATA_HIDDEN_H */
