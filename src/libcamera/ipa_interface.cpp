/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface.cpp - Image Processing Algorithm interface
 */

#include <ipa/ipa_interface.h>

/**
 * \file ipa_interface.h
 * \brief Image Processing Algorithm interface
 *
 * Pipeline handlers communicate with IPAs through a C++ interface defined by
 * the IPAInterface class. The class defines high-level methods and signals to
 * configure the IPA, notify it of events, and receive actions back from the
 * IPA.
 *
 * Pipeline handlers define the set of events and actions used to communicate
 * with their IPA. These are collectively referred to as IPA operations and
 * define the pipeline handler-specific IPA protocol. Each operation defines the
 * data that it carries, and how the data is encoded in the IPAOperationData
 * structure.
 *
 * IPAs can be isolated in a separate process. This implies that all arguments
 * to the IPA interface may need to be transferred over IPC. The IPA interface
 * thus uses serialisable data types only. The IPA interface defines custom data
 * structures that mirror core libcamera structures when the latter are not
 * suitable, such as IPAStream to carry StreamConfiguration data.
 *
 * Due to IPC, synchronous communication between pipeline handlers and IPAs can
 * be costly. For that reason, the interface operates asynchronously. This
 * implies that methods don't return a status, and that both methods and signals
 * may copy their arguments.
 */

namespace libcamera {

/**
 * \struct IPAStream
 * \brief Stream configuration for the IPA interface
 *
 * The IPAStream structure stores stream configuration parameters needed by the
 * IPAInterface::configure() method. It mirrors the StreamConfiguration class
 * that is not suitable for this purpose due to not being serialisable.
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
 * \var IPABuffer::memory
 * \brief The buffer memory description
 *
 * The memory field stores the dmabuf handle and size for each plane of the
 * buffer.
 */

/**
 * \struct IPAOperationData
 * \brief Parameters for IPA operations
 *
 * The IPAOperationData structure carries parameters for the IPA operations
 * performed through the IPAInterface::processEvent() method and the
 * IPAInterface::queueFrameAction signal.
 */

/**
 * \var IPAOperationData::operation
 * \brief IPA protocol operation
 *
 * The operation field describes which operation the receiver shall perform. It
 * defines, through the IPA protocol, how the other fields of the structure are
 * interpreted. The protocol freely assigns numerical values to operations.
 */

/**
 * \var IPAOperationData::data
 * \brief Operation integer data
 *
 * The interpretation and position of different values in the array are defined
 * by the IPA protocol.
 */

/**
 * \var IPAOperationData::controls
 * \brief Operation controls data
 *
 * The interpretation and position of different values in the array are defined
 * by the IPA protocol.
 */

/**
 * \var IPAOperationData::v4l2controls
 * \brief Operation V4L2 controls data
 *
 * The interpretation and position of different values in the array are defined
 * by the IPA protocol.
 */

/**
 * \class IPAInterface
 * \brief Interface for IPA implementation
 *
 * Every pipeline handler in libcamera may attach all or some of its cameras to
 * an Image Processing Algorithm (IPA) module. An IPA module is developed for a
 * specific pipeline handler and each pipeline handler may have multiple
 * compatible IPA implementations, both open and closed source.
 *
 * To allow for multiple IPA modules for the same pipeline handler, a standard
 * interface for the pipeline handler and IPA communication is needed.
 * IPAInterface is this interface.
 *
 * The interface defines base data types and methods to exchange data. On top of
 * this, each pipeline handler is responsible for defining specific operations
 * that make up its IPA protocol, shared by all IPA modules compatible with the
 * pipeline handler.
 *
 * The pipeline handler shall use the IPAManager to locate a compatible
 * IPAInterface. The interface may then be used to interact with the IPA module.
 *
 * \todo Add reference to how pipelines shall document their protocol.
 */

/**
 * \fn IPAInterface::init()
 * \brief Initialise the IPAInterface
 */

/**
 * \fn IPAInterface::configure()
 * \brief Configure the IPA stream and sensor settings
 * \param[in] streamConfig Configuration of all active streams
 * \param[in] entityControls Controls provided by the pipeline entities
 *
 * This method shall be called when the camera is started to inform the IPA of
 * the camera's streams and the sensor settings. The meaning of the numerical
 * keys in the \a streamConfig and \a entityControls maps is defined by the IPA
 * protocol.
 */

/**
 * \fn IPAInterface::mapBuffers()
 * \brief Map buffers shared between the pipeline handler and the IPA
 * \param[in] buffers List of buffers to map
 *
 * This method informs the IPA module of memory buffers set up by the pipeline
 * handler that the IPA needs to access. It provides dmabuf file handles for
 * each buffer, and associates the buffers with unique numerical IDs.
 *
 * IPAs shall map the dmabuf file handles to their address space and keep a
 * cache of the mappings, indexed by the buffer numerical IDs. The IDs are used
 * in all other IPA interface methods to refer to buffers, including the
 * unmapBuffers() method.
 *
 * All buffers that the pipeline handler wishes to share with an IPA shall be
 * mapped with this method. Buffers may be mapped all at once with a single
 * call, or mapped and unmapped dynamically at runtime, depending on the IPA
 * protocol. Regardless of the protocol, all buffers mapped at a given time
 * shall have unique numerical IDs.
 *
 * The numerical IDs have no meaning defined by the IPA interface, and IPA
 * protocols shall not give them any specific meaning either. They should be
 * treated as opaque handles by IPAs, with the only exception that ID zero is
 * invalid.
 *
 * \sa unmapBuffers()
 *
 * \todo Provide a generic implementation of mapBuffers and unmapBuffers for
 * IPAs
 */

/**
 * \fn IPAInterface::unmapBuffers()
 * \brief Unmap buffers shared by the pipeline to the IPA
 * \param[in] ids List of buffer IDs to unmap
 *
 * This method removes mappings set up with mapBuffers(). Buffers may be
 * unmapped all at once with a single call, or selectively at runtime, depending
 * on the IPA protocol. Numerical IDs of unmapped buffers may be reused when
 * mapping new buffers.
 *
 * \sa mapBuffers()
 */

/**
 * \fn IPAInterface::processEvent()
 * \brief Process an event from the pipeline handler
 * \param[in] data IPA operation data
 *
 * This operation is used by pipeline handlers to inform the IPA module of
 * events that occurred during the on-going capture operation.
 *
 * The event notified by the pipeline handler with this method is handled by the
 * IPA, which interprets the operation parameters according to the separately
 * documented IPA protocol.
 */

/**
 * \var IPAInterface::queueFrameAction
 * \brief Queue an action associated with a frame to the pipeline handler
 * \param[in] frame The frame number for the action
 * \param[in] data IPA operation data
 *
 * This signal is emitted when the IPA wishes to queue a FrameAction on the
 * pipeline. The pipeline is still responsible for the scheduling of the action
 * on its timeline.
 *
 * This signal is emitted by the IPA to queue an action to be executed by the
 * pipeline handler on a frame. The type of action is identified by the
 * \a data.operation field, as defined by the IPA protocol, and the rest of the
 * \a data is interpreted accordingly. The pipeline handler shall queue the
 * action and execute it as appropriate.
 */

} /* namespace libcamera */
