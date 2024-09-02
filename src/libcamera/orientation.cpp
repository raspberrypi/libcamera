/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas On Board Oy
 *
 * Image orientation
 */

#include <libcamera/orientation.h>

#include <array>

/**
 * \file orientation.h
 * \brief Image orientation definition
 */

namespace libcamera {

/**
 * \enum Orientation
 * \brief The image orientation in a memory buffer
 *
 * The Orientation enumeration describes the orientation of the images
 * produced by the camera pipeline as they get received by the application
 * inside memory buffers.
 *
 * The image orientation expressed using the Orientation enumeration can be then
 * inferred by applying to a naturally oriented image a multiple of a 90 degrees
 * rotation in the clockwise direction from the origin and then by applying an
 * optional horizontal mirroring.
 *
 * The enumeration numerical values follow the ones defined by the EXIF
 * Specification version 2.32, Tag 274 "Orientation", while the names of the
 * enumerated values report the rotation and mirroring operations performed.
 *
 * For example, Orientation::Rotate90Mirror describes the orientation obtained
 * by rotating the image 90 degrees clockwise first and then applying a
 * horizontal mirroring.
 *
 * \var CameraConfiguration::Rotate0
 * \image html rotation/rotate0.svg
 * \var CameraConfiguration::Rotate0Mirror
 * \image html rotation/rotate0Mirror.svg
 * \var CameraConfiguration::Rotate180
 * \image html rotation/rotate180.svg
 * \var CameraConfiguration::Rotate180Mirror
 * \image html rotation/rotate180Mirror.svg
 * \var CameraConfiguration::Rotate90Mirror
 * \image html rotation/rotate90Mirror.svg
 * \var CameraConfiguration::Rotate270
 * \image html rotation/rotate270.svg
 * \var CameraConfiguration::Rotate270Mirror
 * \image html rotation/rotate270Mirror.svg
 * \var CameraConfiguration::Rotate90
 * \image html rotation/rotate90.svg
 */

/**
 * \brief Return the orientation representing a rotation of the given angle
 * clockwise
 * \param[in] angle The angle of rotation in a clockwise sense. Negative values
 * can be used to represent anticlockwise rotations
 * \param[out] success Set to `true` if the angle is a multiple of 90 degrees,
 * otherwise `false`
 * \return The orientation corresponding to the rotation if \a success was set
 * to `true`, otherwise the `Rotate0` orientation
 */
Orientation orientationFromRotation(int angle, bool *success)
{
	angle = angle % 360;
	if (angle < 0)
		angle += 360;

	if (success != nullptr)
		*success = true;

	switch (angle) {
	case 0:
		return Orientation::Rotate0;
	case 90:
		return Orientation::Rotate90;
	case 180:
		return Orientation::Rotate180;
	case 270:
		return Orientation::Rotate270;
	}

	if (success != nullptr)
		*success = false;

	return Orientation::Rotate0;
}

/**
 * \brief Prints human-friendly names for Orientation items
 * \param[in] out The output stream
 * \param[in] orientation The Orientation item
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const Orientation &orientation)
{
	constexpr std::array<const char *, 9> orientationNames = {
		"", /* Orientation starts counting from 1. */
		"Rotate0",
		"Rotate0Mirror",
		"Rotate180",
		"Rotate180Mirror",
		"Rotate90Mirror",
		"Rotate270",
		"Rotate270Mirror",
		"Rotate90",
	};

	out << orientationNames[static_cast<unsigned int>(orientation)];
	return out;
}

} /* namespace libcamera */
