# libpisp

A helper library to generate run-time configuration for the Raspberry Pi ISP (PiSP), consisting of the Frontend and Backend hardware components.

## Building and installing
To build, setup the meson project as follows:

```sh
meson setup <build_dir>
```
To optionally disable the Boost logging library, add ``-Dlogging=disabled`` as an argument to the ``meson setup`` command.

To compile and install the ``libpisp.so`` artefact:
```sh
meson compile -C <build_dir>
sudo meson install -C <build_dir>
```

## Linking libpisp with an application
libpisp can be built and linked as a [meson subproject](https://mesonbuild.com/Subprojects.html) by using an appropriate [libpisp.wrap](utils/libpisp.wrap) file and the following dependency declaration in the target project:
```meson
libpisp_dep = dependency('libpisp', fallback : ['libpisp', 'libpisp_dep'])
```

Alternatively [pkg-config](https://www.freedesktop.org/wiki/Software/pkg-config/) can be used to locate ``libpisp.so`` installed in of the system directories for other build environments.

## License
Copyright © 2023, Raspberry Pi Ltd. Released under the BSD-2-Clause License.
