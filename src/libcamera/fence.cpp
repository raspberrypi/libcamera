/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * fence.cpp - Synchronization fence
 */

#include "libcamera/fence.h"

namespace libcamera {

/**
 *
 * \file libcamera/fence.h
 * \brief Definition of the Fence class
 */

/**
 * \class Fence
 * \brief Synchronization primitive to manage resources
 *
 * The Fence class models a synchronization primitive that can be used by
 * applications to explicitly synchronize resource usage, and can be shared by
 * multiple processes.
 *
 * Fences are most commonly used in association with frame buffers. A
 * FrameBuffer can be associated with a Fence so that the library can wait for
 * the Fence to be signalled before allowing the camera device to actually
 * access the memory area described by the FrameBuffer.
 *
 * \sa Request::addBuffer()
 *
 * By using a fence, applications can then synchronize between frame buffer
 * consumers and producers, as for example a display device and a camera, to
 * guarantee that a new data transfers only happen once the existing frames have
 * been displayed.
 *
 * A Fence can be realized by different event notification primitives, the most
 * common of which is represented by waiting for read events to happen on a
 * <a href="https://www.kernel.org/doc/html/latest/driver-api/sync_file.html">kernel sync file.</a>
 * This is currently the only mechanism supported by libcamera, but others can
 * be implemented by extending or subclassing this class and implementing
 * opportune handling in the core library.
 *
 * \internal
 *
 * The Fence class is a thin abstraction around a UniqueFD which simply allows
 * to access it as a const reference or to move its ownership to the caller.
 *
 * The usage of the Fence class allows to abstract the underlying
 * synchronization mechanism in use and implement an interface towards other
 * library components that will not change when new synchronization primitives
 * will be added as fences.
 *
 * A Fence is constructed with a UniqueFD whose ownership is moved in the Fence.
 * A FrameBuffer can be associated with a Fence by passing it to the
 * Request::addBuffer() function, which will move the Fence into the FrameBuffer
 * itself. Once a Request is queued to the Camera, a preparation phase
 * guarantees that before actually applying the Request to the hardware, all the
 * valid fences of the frame buffers in a Request are correctly signalled. Once
 * a Fence has completed, the library will release the FrameBuffer fence so that
 * application won't be allowed to access it.
 *
 * An optional timeout can be started while waiting for a fence to complete. If
 * waiting on a Fence fails for whatever reason, the FrameBuffer's fence is not
 * reset and is made available to application for them to handle it, by
 * releasing the Fence to correctly close the underlying UniqueFD.
 *
 * A failure in waiting for a Fence to complete will result in the Request to
 * complete in failed state.
 *
 * \sa Request::prepare()
 * \sa PipelineHandler::doQueueRequests()
 */

/**
 * \brief Create a Fence
 * \param[in] fd The fence file descriptor
 *
 * The file descriptor ownership is moved to the Fence.
 */
Fence::Fence(UniqueFD fd)
	: fd_(std::move(fd))
{
}

/**
 * \fn Fence::isValid()
 * \brief Check if a Fence is valid
 *
 * A Fence is valid if the file descriptor it wraps is valid.
 *
 * \return True if the Fence is valid, false otherwise
 */

/**
 * \fn Fence::fd()
 * \brief Retrieve a constant reference to the file descriptor
 * \return A const reference to the fence file descriptor
 */

/**
 * \fn Fence::release()
 * \brief Release the ownership of the file descriptor
 *
 * Release the ownership of the wrapped file descriptor by returning it to the
 * caller.
 *
 * \return The wrapper UniqueFD
 */

} /* namespace libcamera */
