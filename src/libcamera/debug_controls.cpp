/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Helper to easily record debug metadata inside libcamera.
 */

#include "libcamera/internal/debug_controls.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(DebugControls)

/**
 * \file debug_controls.h
 * \brief Helper to easily record debug metadata inside libcamera
 */

/**
 * \class DebugMetadata
 * \brief Helper to record metadata for later use
 *
 * Metadata is a useful tool for debugging the internal state of libcamera. It
 * has the benefit that it is easy to use and related tooling is readily
 * available. The difficulty is that the metadata control list is often not
 * directly available (either because the variable to debug lives inside
 * process() of an IPA or inside a closed algorithm class with no direct access
 * to the IPA and therefore the metadata list).
 *
 * This class helps in both cases. It allows to forward the data to a parent or
 * alternatively record the data and at a later point in time copy it to the
 * metadata list when it becomes available. Both mechanisms allow easy reuse and
 * loose coupling.
 *
 * Typical usage is to instantiate a DebugMetadata object in every
 * class/algorithm where debug metadata shall be recorded (the inner object). If
 * the IPA doesn't support debug metadata, the object is still usable, but the
 * debug data gets dropped. If the IPA supports debug metadata it will either
 * register a parent DebugMetadata object on the inner object or manually
 * retrieve the data using enable()/moveToList().
 *
 * The concepts of forwarding to a parent and recording for later retrieval are
 * mutually exclusive and the parent takes precedence. E.g. it is not allowed to
 * enable a DebugMetadata object, log entries to it and later set the parent.
 *
 * This is done to keep the path open for using other means of data transport
 * (like tracing). For every tracing event a corresponding context needs to be
 * available on set() time. The parent can be treated as such, the top level
 * object (the one where enable() get's called) also lives in a place where that
 * information is also available.
 */

/**
 * \fn DebugMetadata::enableByControl()
 * \brief Enable based on controls::DebugMetadataEnable in the supplied
 * ControlList
 * \param[in] controls The supplied ControlList
 *
 * This function looks for controls::DebugMetadataEnable and enables or disables
 * debug metadata handling accordingly.
 */
void DebugMetadata::enableByControl(const ControlList &controls)
{
	const auto &ctrl = controls.get(controls::DebugMetadataEnable);
	if (ctrl)
		enable(*ctrl);
}

/**
 * \fn DebugMetadata::enable()
 * \brief Enable or disable metadata handling
 * \param[in] enable The enable state
 *
 * When \a enable is true, all calls to set() get cached and can later be
 * retrieved using moveEntries(). When \a enable is false, the cache gets
 * cleared and no further metadata is recorded.
 *
 * Forwarding to a parent is independent of the enabled state.
 */
void DebugMetadata::enable(bool enable)
{
	enabled_ = enable;
	if (!enabled_)
		cache_.clear();
}

/**
 * \fn DebugMetadata::setParent()
 * \brief Set the parent metadata handler to \a parent
 * \param[in] parent The parent handler
 *
 * When a \a parent is set, all further calls to set() are unconditionally
 * forwarded to that instance.
 *
 * The parent can be reset by passing a nullptr.
 */
void DebugMetadata::setParent(DebugMetadata *parent)
{
	parent_ = parent;

	if (!parent_)
		return;

	if (!cache_.empty())
		LOG(DebugControls, Error)
			<< "Controls were recorded before setting a parent."
			<< " These are dropped.";

	cache_.clear();
}

/**
 * \fn DebugMetadata::moveEntries()
 * \brief Move all cached entries into control list \a list
 * \param[in] list The control list
 *
 * This function moves all entries into the list specified by \a list. Duplicate
 * entries in \a list get overwritten.
 */
void DebugMetadata::moveEntries(ControlList &list)
{
	list.merge(std::move(cache_), ControlList::MergePolicy::OverwriteExisting);
	cache_.clear();
}

/**
 * \fn DebugMetadata::set(const Control<T> &ctrl, const V &value)
 * \brief Set the value of \a ctrl to \a value
 * \param[in] ctrl The control to set
 * \param[in] value The control value
 *
 * If a parent is set, the value gets passed there unconditionally. Otherwise it
 * gets cached if the instance is enabled or dropped silently when disabled.
 *
 * \sa enable()
 */

/**
 * \fn DebugMetadata::set(unsigned int id, const ControlValue &value)
 * \brief Set the value of control \a id to \a value
 * \param[in] id The id of the control
 * \param[in] value The control value
 *
 * If a parent is set, the value gets passed there unconditionally. Otherwise it
 * gets cached if the instance is enabled or dropped silently when disabled.
 *
 * \sa enable()
 */
void DebugMetadata::set(unsigned int id, const ControlValue &value)
{
	if (parent_) {
		parent_->set(id, value);
		return;
	}

	if (!enabled_)
		return;

	cache_.set(id, value);
}

} /* namespace libcamera */
