.. SPDX-License-Identifier: CC-BY-SA-4.0

Contributing
============

libcamera is developed as a free software project and welcomes contributors.
Whether you would like to help with coding, documentation, testing, proposing
new features, or just discussing the project with the community, you can join
our official public communication channels, or simply check out the code.

The project adheres to a :ref:`code of conduct <code-of-conduct>` that
maintainers, contributors and community members are expected to follow in all
online and offline communication.

Mailing List
------------

We use a public mailing list as our main means of communication. You can find
subscription information and the messages archive on the `libcamera-devel`_
list information page.

.. _libcamera-devel: https://lists.libcamera.org/listinfo/libcamera-devel

IRC Channel
-----------

For informal and real time discussions, our IRC channel on irc.oftc.net is open
to the public. Point your IRC client to #libcamera to say hello, or use the
`WebChat`_.

.. _WebChat: https://webchat.oftc.net/?channels=libcamera

Source Code
-----------

libcamera is in early stages of development, and no releases are available yet.
The source code is available from the project's `git tree`_.

.. code-block:: shell

  $ git clone https://git.libcamera.org/libcamera/libcamera.git

.. _git tree: https://git.libcamera.org/libcamera/libcamera.git/

A mirror is also hosted on `LinuxTV`_.

.. _LinuxTV: https://git.linuxtv.org/libcamera.git/

Issue Tracker
-------------

Our `issue tracker`_ tracks all bugs, issues and feature requests. All issues
are publicly visible, and you can register for an account to create new issues.

.. _issue tracker: https://bugs.libcamera.org/

Documentation
-------------

Project documentation is created using `Sphinx`_.  Source level documentation
uses `Doxygen`_.  Please make sure to document all code during development.

.. _Sphinx: https://www.sphinx-doc.org
.. _Doxygen: https://www.doxygen.nl

Submitting Patches
------------------

The libcamera project has high standards of stability, efficiency and
reliability. To achieve those, the project goes to great length to produce
code that is as easy to read, understand and maintain as possible. This is
made possible by a set of :ref:`coding-style-guidelines` that all submissions
are expected to follow.

We also care about the quality of commit messages. A good commit message not
only describes what a commit does, but why it does so. By conveying clear
information about the purpose of the commit, it helps speeding up reviews.
Regardless of whether you're new to git or have years of experience,
https://cbea.ms/git-commit/ is always a good guide to read to improve your
commit message writing skills.

The patch submission process for libcamera is similar to the Linux kernel, and
goes through the `libcamera-devel`_ mailing list. If you have no previous
experience with ``git-send-email``, or just experience trouble configuring it
for your e-mail provider, the sourcehut developers have put together a detailed
guide available at https://git-send-email.io/.

Patches submitted to the libcamera project must be certified as suitable for
integration into an open source project. As such libcamera follows the same
model as utilised by the Linux kernel, and requires the use of 'Signed-off-by:'
tags in all patches.

By signing your contributions you are certifying your work in accordance with
the following:

`Developer's Certificate of Origin`_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Version 1.1

Copyright (C) 2004, 2006 The Linux Foundation and its contributors.
1 Letterman Drive
Suite D4700
San Francisco, CA, 94129

Everyone is permitted to copy and distribute verbatim copies of this
license document, but changing it is not allowed.

Developer's Certificate of Origin 1.1

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the best
    of my knowledge, is covered under an appropriate open source
    license and I have the right under that license to submit that
    work with modifications, whether created in whole or in part
    by me, under the same open source license (unless I am
    permitted to submit under a different license), as indicated
    in the file; or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including all
    personal information I submit with it, including my sign-off) is
    maintained indefinitely and may be redistributed consistent with
    this project or the open source license(s) involved.


.. _Developer's Certificate of Origin: https://developercertificate.org/

.. toctree::
   :hidden:

   Code of Conduct <code-of-conduct>
   Coding Style <coding-style>
