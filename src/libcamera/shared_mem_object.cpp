/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 * Copyright (C) 2024 Andrei Konovalov
 * Copyright (C) 2024 Dennis Bonke
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Helpers for shared memory allocations
 */

#include "libcamera/internal/shared_mem_object.h"

#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <libcamera/base/memfd.h>

/**
 * \file shared_mem_object.cpp
 * \brief Helpers for shared memory allocations
 */

namespace libcamera {

/**
 * \class SharedMem
 * \brief Helper class to allocate and manage memory shareable between processes
 *
 * SharedMem manages memory suitable for sharing between processes. When an
 * instance is constructed, it allocates a memory buffer of the requested size
 * backed by an anonymous file, using the memfd API.
 *
 * The allocated memory is exposed by the mem() function. If memory allocation
 * fails, the function returns an empty Span. This can be also checked using the
 * bool() operator.
 *
 * The file descriptor for the backing file is exposed as a SharedFD by the fd()
 * function. It can be shared with other processes across IPC boundaries, which
 * can then map the memory with mmap().
 *
 * A single memfd is created for every SharedMem. If there is a need to allocate
 * a large number of objects in shared memory, these objects should be grouped
 * together and use the shared memory allocated by a single SharedMem object if
 * possible. This will help to minimize the number of created memfd's.
 */

SharedMem::SharedMem() = default;

/**
 * \brief Construct a SharedMem with memory of the given \a size
 * \param[in] name Name of the SharedMem
 * \param[in] size Size of the shared memory to allocate and map
 *
 * The \a name is used for debugging purpose only. Multiple SharedMem instances
 * can have the same name.
 */
SharedMem::SharedMem(const std::string &name, std::size_t size)
{
	UniqueFD memfd = MemFd::create(name.c_str(), size,
				       MemFd::Seal::Shrink | MemFd::Seal::Grow);
	if (!memfd.isValid())
		return;

	fd_ = SharedFD(std::move(memfd));

	void *mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED,
			 fd_.get(), 0);
	if (mem == MAP_FAILED) {
		fd_ = SharedFD();
		return;
	}

	mem_ = { static_cast<uint8_t *>(mem), size };
}

/**
 * \brief Move constructor for SharedMem
 * \param[in] rhs The object to move
 */
SharedMem::SharedMem(SharedMem &&rhs)
{
	this->fd_ = std::move(rhs.fd_);
	this->mem_ = rhs.mem_;
	rhs.mem_ = {};
}

/**
 * \brief Destroy the SharedMem instance
 *
 * Destroying an instance invalidates the memory mapping exposed with mem().
 * Other mappings of the backing file, created in this or other processes with
 * mmap(), remain valid.
 *
 * Similarly, other references to the backing file descriptor created by copying
 * the SharedFD returned by fd() remain valid. The underlying memory will be
 * freed only when all file descriptors that reference the anonymous file get
 * closed.
 */
SharedMem::~SharedMem()
{
	if (!mem_.empty())
		munmap(mem_.data(), mem_.size_bytes());
}

/**
 * \brief Move assignment operator for SharedMem
 * \param[in] rhs The object to move
 */
SharedMem &SharedMem::operator=(SharedMem &&rhs)
{
	this->fd_ = std::move(rhs.fd_);
	this->mem_ = rhs.mem_;
	rhs.mem_ = {};
	return *this;
}

/**
 * \fn const SharedFD &SharedMem::fd() const
 * \brief Retrieve the file descriptor for the underlying shared memory
 * \return The file descriptor, or an invalid SharedFD if allocation failed
 */

/**
 * \fn Span<uint8_t> SharedMem::mem() const
 * \brief Retrieve the underlying shared memory
 * \return The memory buffer, or an empty Span if allocation failed
 */

/**
 * \fn SharedMem::operator bool()
 * \brief Check if the shared memory allocation succeeded
 * \return True if allocation of the shared memory succeeded, false otherwise
 */

/**
 * \class SharedMemObject
 * \brief Helper class to allocate an object in shareable memory
 * \tparam The object type
 *
 * The SharedMemObject class is a specialization of the SharedMem class that
 * wraps an object of type \a T and constructs it in shareable memory. It uses
 * the same underlying memory allocation and sharing mechanism as the SharedMem
 * class.
 *
 * The wrapped object is constructed at the same time as the SharedMemObject
 * instance, by forwarding the arguments passed to the SharedMemObject
 * constructor. The underlying memory allocation is sized to the object \a T
 * size. The bool() operator should be used to check the allocation was
 * successful. The object can be accessed using the dereference operators
 * operator*() and operator->().
 *
 * While no restriction on the type \a T is enforced, not all types are suitable
 * for sharing between multiple processes. Most notably, any object type that
 * contains pointer or reference members will likely cause issues. Even if those
 * members refer to other members of the same object, the shared memory will be
 * mapped at different addresses in different processes, and the pointers will
 * not be valid.
 *
 * A new anonymous file is created for every SharedMemObject instance. If there
 * is a need to share a large number of small objects, these objects should be
 * grouped into a single larger object to limit the number of file descriptors.
 *
 * To share the object with other processes, see the SharedMem documentation.
 */

/**
 * \var SharedMemObject::kSize
 * \brief The size of the object stored in shared memory
 */

/**
 * \fn SharedMemObject::SharedMemObject(const std::string &name, Args &&...args)
 * \brief Construct a SharedMemObject
 * \param[in] name Name of the SharedMemObject
 * \param[in] args Arguments to pass to the constructor of the object T
 *
 * The \a name is used for debugging purpose only. Multiple SharedMem instances
 * can have the same name.
 */

/**
 * \fn SharedMemObject::SharedMemObject(SharedMemObject<T> &&rhs)
 * \brief Move constructor for SharedMemObject
 * \param[in] rhs The object to move
 */

/**
 * \fn SharedMemObject::~SharedMemObject()
 * \brief Destroy the SharedMemObject instance
 *
 * Destroying a SharedMemObject calls the wrapped T object's destructor. While
 * the underlying memory may not be freed immediately if other mappings have
 * been created manually (see SharedMem::~SharedMem() for more information), the
 * stored object may be modified. Depending on the ~T() destructor, accessing
 * the object after destruction of the SharedMemObject causes undefined
 * behaviour. It is the responsibility of the user of this class to synchronize
 * with other users who have access to the shared object.
 */

/**
 * \fn SharedMemObject::operator=(SharedMemObject<T> &&rhs)
 * \brief Move assignment operator for SharedMemObject
 * \param[in] rhs The SharedMemObject object to take the data from
 *
 * Moving a SharedMemObject does not affect the stored object.
 */

/**
 * \fn SharedMemObject::operator->()
 * \brief Dereference the stored object
 * \return Pointer to the stored object
 */

/**
 * \fn const T *SharedMemObject::operator->() const
 * \copydoc SharedMemObject::operator->
 */

/**
 * \fn SharedMemObject::operator*()
 * \brief Dereference the stored object
 * \return Reference to the stored object
 */

/**
 * \fn const T &SharedMemObject::operator*() const
 * \copydoc SharedMemObject::operator*
 */

} /* namespace libcamera */
