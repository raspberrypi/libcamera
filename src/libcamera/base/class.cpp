/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * class.cpp - Utilities and helpers for classes
 */

#include <libcamera/base/class.h>

/**
 * \file class.h
 * \brief Utilities to help constructing class interfaces
 *
 * The extensible class can be inherited to create public classes with stable
 * ABIs.
 */

namespace libcamera {

/**
 * \def LIBCAMERA_DISABLE_COPY
 * \brief Disable copy construction and assignment of the \a klass
 * \param klass The name of the class
 *
 * Example usage:
 * \code{.cpp}
 * class NonCopyable
 * {
 * public:
 * 	NonCopyable();
 * 	...
 *
 * private:
 * 	LIBCAMERA_DISABLE_COPY(NonCopyable)
 * };
 * \endcode
 */

/**
 * \def LIBCAMERA_DISABLE_MOVE
 * \brief Disable move construction and assignment of the \a klass
 * \param klass The name of the class
 *
 * Example usage:
 * \code{.cpp}
 * class NonMoveable
 * {
 * public:
 * 	NonMoveable();
 * 	...
 *
 * private:
 * 	LIBCAMERA_DISABLE_MOVE(NonMoveable)
 * };
 * \endcode
 */

/**
 * \def LIBCAMERA_DISABLE_COPY_AND_MOVE
 * \brief Disable copy and move construction and assignment of the \a klass
 * \param klass The name of the class
 *
 * Example usage:
 * \code{.cpp}
 * class NonCopyableNonMoveable
 * {
 * public:
 * 	NonCopyableNonMoveable();
 * 	...
 *
 * private:
 * 	LIBCAMERA_DISABLE_COPY_AND_MOVE(NonCopyableNonMoveable)
 * };
 * \endcode
 */

/**
 * \def LIBCAMERA_DECLARE_PRIVATE
 * \brief Declare private data for a public class
 *
 * The LIBCAMERA_DECLARE_PRIVATE() macro plumbs the infrastructure necessary to
 * make a class manage its private data through a d-pointer. It shall be used at
 * the very top of the class definition.
 */

/**
 * \def LIBCAMERA_DECLARE_PUBLIC
 * \brief Declare public data for a private class
 * \param klass The public class name
 *
 * The LIBCAMERA_DECLARE_PUBLIC() macro is the counterpart of
 * LIBCAMERA_DECLARE_PRIVATE() to be used in the private data class. It shall be
 * used at the very top of the private class definition, with the public class
 * name passed as the \a klass parameter.
 */

/**
 * \def LIBCAMERA_O_PTR()
 * \brief Retrieve the public instance corresponding to the private data
 *
 * This macro is part of the libcamera::Extensible class infrastructure. It may
 * be used in any member function of a libcamera::Extensible::Private subclass
 * to access the public class instance corresponding to the private data.
 */

/**
 * \class Extensible
 * \brief Base class to manage private data through a d-pointer
 *
 * The Extensible class provides a base class to implement the
 * <a href="https://wiki.qt.io/D-Pointer">d-pointer</a> design pattern (also
 * known as <a href="https://en.wikipedia.org/wiki/Opaque_pointer">opaque pointer</a>
 * or <a href="https://en.cppreference.com/w/cpp/language/pimpl">pImpl idiom</a>).
 * It helps creating public classes that can be extended without breaking their
 * ABI. Such classes store their private data in a separate private data object,
 * referenced by a pointer in the public class (hence the name d-pointer).
 *
 * Classes that follow this design pattern are referred herein as extensible
 * classes. To be extensible, a class PublicClass shall:
 *
 * - inherit from the Extensible class or from another extensible class
 * - invoke the LIBCAMERA_DECLARE_PRIVATE() macro at the very top of the class
 *   definition
 * - define a private data class named PublicClass::Private that inherits from
 *   the Private data class of the base class
 * - invoke the LIBCAMERA_DECLARE_PUBLIC() macro at the very top of the Private
 *   data class definition
 * - pass a pointer to a newly allocated Private data object to the constructor
 *   of the base class
 *
 * Additionally, if the PublicClass is not final, it shall expose one or more
 * constructors that takes a pointer to a Private data instance, to be used by
 * derived classes.
 *
 * The Private class is fully opaque to users of the libcamera public API.
 * Internally, it can be kept private to the implementation of PublicClass, or
 * be exposed to other classes. In the latter case, the members of the Private
 * class need to be qualified with appropriate access specifiers. The
 * PublicClass and Private classes always have full access to each other's
 * protected and private members.
 *
 * The PublicClass exposes its Private data pointer through the _d() function.
 * In the other direction, the pointer to the PublicClass can be retrieved in
 * functions of the Private class using the LIBCAMERA_O_PTR() macro.
 */

/**
 * \brief Construct an instance of an Extensible class
 * \param[in] d Pointer to the private data instance
 *
 * The private data lifetime is managed by the Extensible class, which destroys
 * it when the Extensible instance is destroyed.
 */
Extensible::Extensible(std::unique_ptr<Extensible::Private> d)
	: d_(std::move(d))
{
	*const_cast<Extensible **>(&d_->o_) = this;
}

/**
 * \fn Extensible::_d() const
 * \brief Retrieve the private data instance
 *
 * This template function isn't meant to be called directly. Instead, classes
 * derived from Extensible get, through the LIBCAMERA_DECLARE_PRIVATE() macro,
 * overriden _d() functions that return the correct pointer type to the
 * corresponding derived Private class.
 *
 * The lifetime of the private data is tied to the Extensible class. The caller
 * shall not retain any reference to the returned pointer for longer than it
 * holds a reference to the Extensible instance.
 *
 * \return A pointer to the private data instance
 */

/**
 * \fn Extensible::_d()
 * \copydoc Extensible::_d() const
 */

/**
 * \var Extensible::d_
 * \brief Pointer to the private data instance
 */

/**
 * \class Extensible::Private
 * \brief Base class for private data managed through a d-pointer
 */

/**
 * \brief Construct an instance of an Extensible class private data
 */
Extensible::Private::Private()
	: o_(nullptr)
{
}

Extensible::Private::~Private()
{
}

/**
 * \var Extensible::Private::o_
 * \brief Pointer to the public class object
 */

} /* namespace libcamera */
