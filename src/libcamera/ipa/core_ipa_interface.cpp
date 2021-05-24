/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * core_ipa_interface.cpp - Docs file for core.mojom generated header
 */

namespace libcamera {

/**
 * \file core_ipa_interface.h
 * \brief Core IPA inteface
 */

/**
 * \struct IPABuffer
 * \brief Buffer information for the IPA interface
 *
 * The IPABuffer structure associates buffer memory with a unique ID. It is
 * used to map buffers to the IPA with IPAInterface::mapBuffers(), after which
 * buffers will be identified by their ID in the IPA interface.
 */

/**
 * \var IPABuffer::id
 * \brief The buffer unique ID
 *
 * Buffers mapped to the IPA are identified by numerical unique IDs. The IDs
 * are chosen by the pipeline handler to fulfil the following constraints:
 *
 * - IDs shall be positive integers different than zero
 * - IDs shall be unique among all mapped buffers
 *
 * When buffers are unmapped with IPAInterface::unmapBuffers() their IDs are
 * freed and may be reused for new buffer mappings.
 */

/**
 * \var IPABuffer::planes
 * \brief The buffer planes description
 *
 * Stores the dmabuf handle and length for each plane of the buffer.
 */

/**
 * \struct IPASettings
 * \brief IPA interface initialization settings
 *
 * The IPASettings structure stores data passed to the IPAInterface::init()
 * function. The data contains settings that don't depend on a particular camera
 * or pipeline configuration and are valid for the whole life time of the IPA
 * interface.
 */

/**
 * \var IPASettings::configurationFile
 * \brief The name of the IPA configuration file
 *
 * This field may be an empty string if the IPA doesn't require a configuration
 * file.
 */

/**
 * \var IPASettings::sensorModel
 * \brief The sensor model name
 *
 * Provides the sensor model name to the IPA.
 */

/**
 * \struct IPAStream
 * \brief Stream configuration for the IPA interface
 *
 * The IPAStream structure stores stream configuration parameters needed by the
 * IPAInterface::configure() method. It mirrors the StreamConfiguration class
 * that is not suitable for this purpose due to not being serializable.
 */

/**
 * \var IPAStream::pixelFormat
 * \brief The stream pixel format
 */

/**
 * \var IPAStream::size
 * \brief The stream size in pixels
 */

/**
 * \struct CameraSensorInfo
 * \brief Report the image sensor characteristics
 *
 * The structure reports image sensor characteristics used by IPA modules to
 * tune their algorithms based on the image sensor model currently in use and
 * its configuration.
 *
 * The reported information describes the sensor's intrinsics characteristics,
 * such as its pixel array size and the sensor model name, as well as
 * information relative to the currently configured mode, such as the produced
 * image size and the bit depth of the requested image format.
 *
 * Instances of this structure are meant to be assembled by the CameraSensor
 * class by inspecting the sensor static properties as well as information
 * relative to the current configuration.
 */

/**
 * \var CameraSensorInfo::model
 * \brief The image sensor model name
 *
 * The sensor model name is a free-formed string that uniquely identifies the
 * sensor model.
 */

/**
 * \var CameraSensorInfo::bitsPerPixel
 * \brief The number of bits per pixel of the image format produced by the
 * image sensor
 */

/**
 * \var CameraSensorInfo::activeAreaSize
 * \brief The size of the pixel array active area of the sensor
 */

/**
 * \var CameraSensorInfo::analogCrop
 * \brief The portion of the pixel array active area which is read-out and
 * processed
 *
 * The analog crop rectangle top-left corner is defined as the displacement
 * from the top-left corner of the pixel array active area. The rectangle
 * horizontal and vertical sizes define the portion of the pixel array which
 * is read-out and provided to the sensor's internal processing pipeline, before
 * any pixel sub-sampling method, such as pixel binning, skipping and averaging
 * take place.
 */

/**
 * \var CameraSensorInfo::outputSize
 * \brief The size of the images produced by the camera sensor
 *
 * The output image size defines the horizontal and vertical sizes of the images
 * produced by the image sensor. The output image size is defined as the end
 * result of the sensor's internal image processing pipeline stages, applied on
 * the pixel array portion defined by the analog crop rectangle. Each image
 * processing stage that performs pixel sub-sampling techniques, such as pixel
 * binning or skipping, or perform any additional digital scaling concur in the
 * definition of the output image size.
 */

/**
 * \var CameraSensorInfo::pixelRate
 * \brief The number of pixels produced in a second
 *
 * To obtain the read-out time in seconds of a full line:
 *
 * \verbatim
       lineDuration(s) = lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */

/**
 * \var CameraSensorInfo::lineLength
 * \brief Total line length in pixels
 *
 * The total line length in pixel clock periods, including blanking.
 */

/**
 * \var CameraSensorInfo::minFrameLength
 * \brief The minimum allowable frame length in units of lines
 *
 * The sensor frame length comprises of active output lines and blanking lines
 * in a frame. The minimum frame length value dictates the minimum allowable
 * frame duration of the sensor mode.
 *
 * To obtain the minimum frame duration:
 *
 * \verbatim
       frameDuration(s) = minFrameLength(lines) * lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */

/**
 * \var CameraSensorInfo::maxFrameLength
 * \brief The maximum allowable frame length in units of lines
 *
 * The sensor frame length comprises of active output lines and blanking lines
 * in a frame. The maximum frame length value dictates the maximum allowable
 * frame duration of the sensor mode.
 *
 * To obtain the maximum frame duration:
 *
 * \verbatim
       frameDuration(s) = maxFrameLength(lines) * lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */
} /* namespace libcamera */
