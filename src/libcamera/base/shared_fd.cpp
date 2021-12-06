/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * shared_fd.cpp - File descriptor wrapper with shared ownership
 */

#include <libcamera/base/shared_fd.h>

#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

#include <libcamera/base/log.h>
#include <libcamera/base/unique_fd.h>

/**
 * \file base/shared_fd.h
 * \brief File descriptor wrapper
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(SharedFD)

/**
 * \class SharedFD
 * \brief RAII-style wrapper for file descriptors
 *
 * The SharedFD class provides RAII-style lifetime management of file
 * descriptors with an efficient mechanism for ownership sharing. At its core,
 * an internal Descriptor object wraps a file descriptor (expressed as a signed
 * integer) with an RAII-style interface. The Descriptor is then implicitly
 * shared with all SharedFD instances constructed as copies.
 *
 * When constructed from a numerical file descriptor, the SharedFD instance
 * either duplicates or takes over the file descriptor:
 *
 * - The SharedFD(const int &) constructor duplicates the numerical file
 *   descriptor and wraps the duplicate in a Descriptor. The caller is
 *   responsible for closing the original file descriptor, and the value
 *   returned by fd() will be different from the value passed to the
 *   constructor.
 *
 * - The SharedFD(int &&) constructor takes over the numerical file descriptor
 *   and wraps it in a Descriptor. The caller shall not touch the original file
 *   descriptor once the function returns, and the value returned by fd() will
 *   be identical to the value passed to the constructor.
 *
 * The copy constructor and assignment operator create copies that share the
 * Descriptor, while the move versions of those functions additionally make the
 * other SharedFD invalid. When the last SharedFD that references a Descriptor
 * is destroyed, the file descriptor is closed.
 *
 * The numerical file descriptor is available through the fd() function. All
 * SharedFD instances created as copies of a SharedFD will report the same fd()
 * value. Callers can perform operations on the fd(), but shall never close it
 * manually.
 */

/**
 * \brief Create a SharedFD copying a given \a fd
 * \param[in] fd File descriptor
 *
 * Construct a SharedFD from a numerical file descriptor by duplicating the
 * \a fd, and take ownership of the copy. The original \a fd is left untouched,
 * and the caller is responsible for closing it when appropriate. The duplicated
 * file descriptor will be closed automatically when all SharedFD instances that
 * reference it are destroyed.
 *
 * If the \a fd is negative, the SharedFD is constructed as invalid and the fd()
 * function will return -1.
 */
SharedFD::SharedFD(const int &fd)
{
	if (fd < 0)
		return;

	fd_ = std::make_shared<Descriptor>(fd, true);
	if (fd_->fd() < 0)
		fd_.reset();
}

/**
 * \brief Create a SharedFD taking ownership of a given \a fd
 * \param[in] fd File descriptor
 *
 * Construct a SharedFD from a numerical file descriptor by taking ownership of
 * the \a fd. The original \a fd is set to -1 and shall not be touched by the
 * caller anymore. In particular, the caller shall not close the original \a fd
 * manually. The duplicated file descriptor will be closed automatically when
 * all SharedFD instances that reference it are destroyed.
 *
 * If the \a fd is negative, the SharedFD is constructed as invalid and the fd()
 * function will return -1.
 */
SharedFD::SharedFD(int &&fd)
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
 * \brief Create a SharedFD taking ownership of a given UniqueFD \a fd
 * \param[in] fd UniqueFD
 *
 * Construct a SharedFD from UniqueFD by taking ownership of the \a fd. The
 * original \a fd becomes invalid.
 */
SharedFD::SharedFD(UniqueFD fd)
	: SharedFD(fd.release())
{
}

/**
 * \brief Copy constructor, create a SharedFD from a copy of \a other
 * \param[in] other The other SharedFD
 *
 * Copying a SharedFD implicitly shares ownership of the wrapped file
 * descriptor. The original SharedFD is left untouched, and the caller is
 * responsible for destroying it when appropriate. The wrapped file descriptor
 * will be closed automatically when all SharedFD instances that reference it
 * are destroyed.
 */
SharedFD::SharedFD(const SharedFD &other)
	: fd_(other.fd_)
{
}

/**
 * \brief Move constructor, create a SharedFD by taking over \a other
 * \param[in] other The other SharedFD
 *
 * Moving a SharedFD moves the reference to the wrapped descriptor owned by
 * \a other to the new SharedFD. The \a other SharedFD is invalidated and its
 * fd() function will return -1. The wrapped file descriptor will be closed
 * automatically when all SharedFD instances that reference it are destroyed.
 */
SharedFD::SharedFD(SharedFD &&other)
	: fd_(std::move(other.fd_))
{
}

/**
 * \brief Destroy the SharedFD instance
 *
 * Destroying a SharedFD instance releases its reference to the wrapped
 * descriptor, if any. When the last instance that references a wrapped
 * descriptor is destroyed, the file descriptor is automatically closed.
 */
SharedFD::~SharedFD()
{
}

/**
 * \brief Copy assignment operator, replace the wrapped file descriptor with a
 * copy of \a other
 * \param[in] other The other SharedFD
 *
 * Copying a SharedFD creates a new reference to the wrapped file descriptor
 * owner by \a other. If \a other is invalid, *this will also be invalid. The
 * original SharedFD is left untouched, and the caller is responsible for
 * destroying it when appropriate. The wrapped file descriptor will be closed
 * automatically when all SharedFD instances that reference it are destroyed.
 *
 * \return A reference to this SharedFD
 */
SharedFD &SharedFD::operator=(const SharedFD &other)
{
	fd_ = other.fd_;

	return *this;
}

/**
 * \brief Move assignment operator, replace the wrapped file descriptor by
 * taking over \a other
 * \param[in] other The other SharedFD
 *
 * Moving a SharedFD moves the reference to the wrapped descriptor owned by
 * \a other to the new SharedFD. If \a other is invalid, *this will also be
 * invalid. The \a other SharedFD is invalidated and its fd() function will
 * return -1. The wrapped file descriptor will be closed automatically when
 * all SharedFD instances that reference it are destroyed.
 *
 * \return A reference to this SharedFD
 */
SharedFD &SharedFD::operator=(SharedFD &&other)
{
	fd_ = std::move(other.fd_);

	return *this;
}

/**
 * \fn SharedFD::isValid()
 * \brief Check if the SharedFD instance is valid
 * \return True if the SharedFD is valid, false otherwise
 */

/**
 * \fn SharedFD::get()
 * \brief Retrieve the numerical file descriptor
 * \return The numerical file descriptor, which may be -1 if the SharedFD
 * instance is invalid
 */

/**
 * \fn bool operator==(const SharedFD &lhs, const SharedFD &rhs)
 * \brief Compare the owned file descriptors of two SharedFD for equality
 * \param[in] lhs The first SharedFD
 * \param[in] rhs The second SharedFD
 *
 * Two file descriptors are considered equal if they have the same numerical
 * value. File descriptors with different values that both reference the same
 * file (for instance obtained using dup()) are considered not equal.
 *
 * \return True if the two file descriptors are equal, false otherwise
 */

/**
 * \fn bool operator!=(const SharedFD &lhs, const SharedFD &rhs)
 * \brief Compare the owned file descriptors of two SharedFD for equality
 * \param[in] lhs The first SharedFD
 * \param[in] rhs The second SharedFD
 *
 * Two file descriptors are considered equal if they have the same numerical
 * value. File descriptors with different values that both reference the same
 * file (for instance obtained using dup()) are considered not equal.
 *
 * \return True if the two file descriptors are not equal, false otherwise
 */

/**
 * \brief Duplicate a SharedFD
 *
 * Duplicating a SharedFD creates a duplicate of the wrapped file descriptor and
 * returns a UniqueFD that owns the duplicate. The fd() function of the original
 * and the get() function of the duplicate will return different values. The
 * duplicate instance will not be affected by destruction of the original
 * instance or its copies.
 *
 * \return A UniqueFD owning a duplicate of the original file descriptor
 */
UniqueFD SharedFD::dup() const
{
	if (!isValid())
		return {};

	UniqueFD dupFd(::dup(get()));
	if (!dupFd.isValid()) {
		int ret = -errno;
		LOG(SharedFD, Error)
			<< "Failed to dup() fd: " << strerror(-ret);
	}

	return dupFd;
}

SharedFD::Descriptor::Descriptor(int fd, bool duplicate)
{
	if (!duplicate) {
		fd_ = fd;
		return;
	}

	/* Failing to dup() a fd should not happen and is fatal. */
	fd_ = ::dup(fd);
	if (fd_ == -1) {
		int ret = -errno;
		LOG(SharedFD, Fatal)
			<< "Failed to dup() fd: " << strerror(-ret);
	}
}

SharedFD::Descriptor::~Descriptor()
{
	if (fd_ != -1)
		close(fd_);
}

} /* namespace libcamera */
