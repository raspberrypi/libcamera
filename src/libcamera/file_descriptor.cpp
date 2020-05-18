/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_descriptor.cpp - File descriptor wrapper
 */

#include <libcamera/file_descriptor.h>

#include <string.h>
#include <unistd.h>
#include <utility>

#include "libcamera/internal/log.h"

/**
 * \file file_descriptor.h
 * \brief File descriptor wrapper
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(FileDescriptor)

/**
 * \class FileDescriptor
 * \brief RAII-style wrapper for file descriptors
 *
 * The FileDescriptor class provides RAII-style lifetime management of file
 * descriptors with an efficient mechanism for ownership sharing. At its core,
 * an internal Descriptor object wraps a file descriptor (expressed as a signed
 * integer) with an RAII-style interface. The Descriptor is then implicitly
 * shared with all FileDescriptor instances constructed as copies.
 *
 * When constructed from a numerical file descriptor, the FileDescriptor
 * instance either duplicates or takes over the file descriptor:
 *
 * - The FileDescriptor(const int &) constructor duplicates the numerical file
 *   descriptor and wraps the duplicate in a Descriptor. The caller is
 *   responsible for closing the original file descriptor, and the value
 *   returned by fd() will be different from the value passed to the
 *   constructor.
 *
 * - The FileDescriptor(int &&) constructor takes over the numerical file
 *   descriptor and wraps it in a Descriptor. The caller is shall not touch the
 *   original file descriptor once the function returns, and the value returned
 *   by fd() will be identical to the value passed to the constructor.
 *
 * The copy constructor and assignment operator create copies that share the
 * Descriptor, while the move versions of those methods additionally make the
 * other FileDescriptor invalid. When the last FileDescriptor that references a
 * Descriptor is destroyed, the file descriptor is closed.
 *
 * The numerical file descriptor is available through the fd() method. All
 * FileDescriptor instances created as copies of a FileDescriptor will report
 * the same fd() value. Callers can perform operations on the fd(), but shall
 * never close it manually.
 */

/**
 * \brief Create a FileDescriptor copying a given \a fd
 * \param[in] fd File descriptor
 *
 * Construct a FileDescriptor from a numerical file descriptor by duplicating
 * the \a fd, and take ownership of the copy. The original \a fd is left
 * untouched, and the caller is responsible for closing it when appropriate.
 * The duplicated file descriptor will be closed automatically when all
 * FileDescriptor instances that reference it are destroyed.
 *
 * If the \a fd is negative, the FileDescriptor is constructed as invalid and
 * the fd() method will return -1.
 */
FileDescriptor::FileDescriptor(const int &fd)
{
	if (fd < 0)
		return;

	fd_ = std::make_shared<Descriptor>(fd, true);
	if (fd_->fd() < 0)
		fd_.reset();
}

/**
 * \brief Create a FileDescriptor taking ownership of a given \a fd
 * \param[in] fd File descriptor
 *
 * Construct a FileDescriptor from a numerical file descriptor by taking
 * ownership of the \a fd. The original \a fd is set to -1 and shall not be
 * touched by the caller anymore. In particular, the caller shall not close the
 * original \a fd manually. The duplicated file descriptor will be closed
 * automatically when all FileDescriptor instances that reference it are
 * destroyed.
 *
 * If the \a fd is negative, the FileDescriptor is constructed as invalid and
 * the fd() method will return -1.
 */
FileDescriptor::FileDescriptor(int &&fd)
{
	if (fd < 0)
		return;

	fd_ = std::make_shared<Descriptor>(fd, false);
	/*
	 * The Descriptor constructor can't have failed here, as it took over
	 * the fd without duplicating it. Just set the original fd to -1 to
	 * implement move semantics.
	 */
	fd = -1;
}

/**
 * \brief Copy constructor, create a FileDescriptor from a copy of \a other
 * \param[in] other The other FileDescriptor
 *
 * Copying a FileDescriptor implicitly shares ownership of the wrapped file
 * descriptor. The original FileDescriptor is left untouched, and the caller is
 * responsible for destroying it when appropriate. The wrapped file descriptor
 * will be closed automatically when all FileDescriptor instances that
 * reference it are destroyed.
 */
FileDescriptor::FileDescriptor(const FileDescriptor &other)
	: fd_(other.fd_)
{
}

/**
 * \brief Move constructor, create a FileDescriptor by taking over \a other
 * \param[in] other The other FileDescriptor
 *
 * Moving a FileDescriptor moves the reference to the wrapped descriptor owned
 * by \a other to the new FileDescriptor. The \a other FileDescriptor is
 * invalidated and its fd() method will return -1. The wrapped file descriptor
 * will be closed automatically when all FileDescriptor instances that
 * reference it are destroyed.
 */
FileDescriptor::FileDescriptor(FileDescriptor &&other)
	: fd_(std::move(other.fd_))
{
}

/**
 * \brief Destroy the FileDescriptor instance
 *
 * Destroying a FileDescriptor instance releases its reference to the wrapped
 * descriptor, if any. When the last instance that references a wrapped
 * descriptor is destroyed, the file descriptor is automatically closed.
 */
FileDescriptor::~FileDescriptor()
{
}

/**
 * \brief Copy assignment operator, replace the wrapped file descriptor with a
 * copy of \a other
 * \param[in] other The other FileDescriptor
 *
 * Copying a FileDescriptor creates a new reference to the wrapped file
 * descriptor owner by \a other. If \a other is invalid, *this will also be
 * invalid. The original FileDescriptor is left untouched, and the caller is
 * responsible for destroying it when appropriate. The wrapped file descriptor
 * will be closed automatically when all FileDescriptor instances that
 * reference it are destroyed.
 *
 * \return A reference to this FileDescriptor
 */
FileDescriptor &FileDescriptor::operator=(const FileDescriptor &other)
{
	fd_ = other.fd_;

	return *this;
}

/**
 * \brief Move assignment operator, replace the wrapped file descriptor by
 * taking over \a other
 * \param[in] other The other FileDescriptor
 *
 * Moving a FileDescriptor moves the reference to the wrapped descriptor owned
 * by \a other to the new FileDescriptor. If \a other is invalid, *this will
 * also be invalid. The \a other FileDescriptor is invalidated and its fd()
 * method will return -1. The wrapped file descriptor will be closed
 * automatically when all FileDescriptor instances that reference it are
 * destroyed.
 *
 * \return A reference to this FileDescriptor
 */
FileDescriptor &FileDescriptor::operator=(FileDescriptor &&other)
{
	fd_ = std::move(other.fd_);

	return *this;
}

/**
 * \fn FileDescriptor::isValid()
 * \brief Check if the FileDescriptor instance is valid
 * \return True if the FileDescriptor is valid, false otherwise
 */

/**
 * \fn FileDescriptor::fd()
 * \brief Retrieve the numerical file descriptor
 * \return The numerical file descriptor, which may be -1 if the FileDescriptor
 * instance is invalid
 */

/**
 * \brief Duplicate a FileDescriptor
 *
 * Duplicating a FileDescriptor creates a duplicate of the wrapped file
 * descriptor and returns a new FileDescriptor instance that wraps the
 * duplicate. The fd() method of the original and duplicate instances will
 * return different values. The duplicate instance will not be affected by
 * destruction of the original instance or its copies.
 *
 * \return A new FileDescriptor instance wrapping a duplicate of the original
 * file descriptor
 */
FileDescriptor FileDescriptor::dup() const
{
	return FileDescriptor(fd());
}

FileDescriptor::Descriptor::Descriptor(int fd, bool duplicate)
{
	if (!duplicate) {
		fd_ = fd;
		return;
	}

	/* Failing to dup() a fd should not happen and is fatal. */
	fd_ = ::dup(fd);
	if (fd_ == -1) {
		int ret = -errno;
		LOG(FileDescriptor, Fatal)
			<< "Failed to dup() fd: " << strerror(-ret);
	}
}

FileDescriptor::Descriptor::~Descriptor()
{
	if (fd_ != -1)
		close(fd_);
}

} /* namespace libcamera */
