AVR3nk
================

AVR3nk is a program library collection for different kinds of [Atmel's 8-bit AVR](http://www.atmel.com/products/microcontrollers/avr/default.aspx) microcontrollers.
It is fuelled by a cross-platform build environment based on [GNU Make][make] and the AVR-GCC toolchain, which allows for development on Linux, Mac OS X, and Windows.

Up to now, AVR3nk includes an interrupt-driven and buffered driver for dual UART operation, a timer driver with a rich feature set (such as countdown, stopwatch and different PWM modes), as well as an interrupt-driven driver for the MCP2515 [CAN](http://en.wikipedia.org/wiki/CAN_bus) controller, which interfaces via [SPI](http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus).
In addition, there are two subsystems that can be stacked on top of these drivers in order to facilitate the process of building a fully fledged microcontroller application.
The first subsystem is a commandline interface, which allows for interactive and parameterized command invocation.
The second subsystem is a runloop, which allows to schedule tasks periodically in the background of the application.
Both subsystems can be combined in order to build an application that allows to launch periodic tasks from the interactive commandline interface.


Build Environment
----------------

Software running on a microcontroller has typically some project-specific requirements on clock frequency, pinouts and other settings.
Since many of these parameters do not change at runtime, they are ideally defined already at compile time.
In order to do so, libraries have to be compiled with project-specific adjustments prior to being linked to an application.
However, duplicating the library code and adjusting it in place to project-specific requirements is bad practice, as it compromises the maintainability of the library's source code.

AVR3nk's build environment allows to organize libraries in a single place and to compile them in different configurations for individual projects.
Static parameters (i.e., parameters which do not change at runtime) are defined as macros in the libraries' header files.
The default values of these macros can then be overriden through individual build settings of an application.
When an application is built, project-specific variants of the included libraries are accordingly built and the resulting object code is linked to the final application.
Furthermore, AVR3nk's build environment allows to declare libraries to be capable of handling various kinds of MCUs.
This allows to reuse the same libraries in different projects which target different AVR devices.

AVR3nk's build environment is based on [GNU Make][make] and allows for an easy setup of build settings through a few variables (see below).


Setup
----------------

You need to install the following packages in order to use AVR3nk:
* avr-gcc
* avr-libc
* avr-binutils
* avrdude

**Linux**: Most Linux distributions have an AVR-GCC toolchain available through their packet managers.

**Mac OS OX**: I recommend installing the toolchain via [MacPorts](http://www.macports.org). Alternatively, there is [CrossPack](http://www.obdev.at/products/crosspack/index.html) and [OSX-AVR](http://sourceforge.net/projects/osxavr) (I haven't tried the latter ones yet, though).

**Windows**: Download and install [WinAVR](http://winavr.sourceforge.net).

Once you've downloaded AVR3nk, you should create a custom configuration file in *env/config*.
This file will hold your user specific configuration, such as the location of your WinAVR installation (if you are running Windows) and your programming hardware settings.
You should use *config.mak* as a template and name your personal file *config.mak.&lt;yourLoginName&gt;*.
The template provides further instructions on each setting.
Note that you have to remove any whitespaces in &lt;yourLoginName&gt;.
E.g., if my login name was *Robin Klose*, my configuration file would be named *config.mak.RobinKlose*.
The build environment automatically reads your personalized configuration whenever you invoke *make*.
This mechanism allows you to keep the configuration files of your team members under revision control next to each other in a single place.


Directory Structure
----------------

AVR3nk's software is mainly located in three subdirectories: *drivers*, *subsystems*, and *applications*.
The *drivers* directory contains libraries that access a microcontroller's registers or peripheral hardware.
The *subsystems* directory contains libraries that are stacked on top of other libraries but which do not directly interact with hardware.
The *applications* directory contains software projects that can be compiled to an executable and be programmed to AVR hardware. <br/>
The *env* directory contains the build environment and user specific configuration files.
The *docs* directory contains coding guidelines, source code templates, makefile templates and other documents.
The build environment also creates an *install* directory which is added to the search path of the compiler to find header files and pre-compiled libraries.


Make Targets
----------------

The AVR3nk build environment allows to build libraries and applications by running *make* commands in a terminal, such as *make all*, *make build* or *make program*.
When one of the specified make targets is invoked in a directory, the command is executed recursively in all of its subdirectories.
In order to build all libraries and applications at once, execute *make all* in AVR3nk's top directory.

The following make targets are available:

* **clean**:
Removes all files generated locally by the build environment, such as object code, library archives, and Intel hex programming files.
When *make clean* is invoked in AVR3nk's top directory, the install directory is removed prior to executing the *clean* target recursively in the subdirectories.
Make a direcotry clean when you need to make sure that all source files are compiled anew in a subsequent call of the *build* target.

* **build**:
Compiles the source code.
For libraries, it will also create (or update) a library archive if source files were updated.
For an application, it will also build library dependencies into the application's build directory and link everything to an executable which can be programmed to AVR hardware.
The build command compiles only files that were changed since the last compilation.

* **headers**:
Copies the header files of a library to the install directory.

* **install**:
Copies the pre-compiled archive file of a library to the install directory.

* **all**:
Rebuilds everything anew.
For libraries, it expands to *clean* *headers* *build* *install*.
For applications, it expands to *clean* *build*.
In AVR3nk's top directory, it also removes all files in the install directory prior to processing of subdirectories.

* **program**:
This target is only available for applications.
It invokes avrdude to write the programming file to the AVR microcontroller.

* **size**:
This target displays the size of a program.
It is only available for applications.


Sample Projects
----------------

Some of the libraries contain a *test* subdirectory with sample code that can be used as a reference to see how to use the particular library.
The project in *subsystems/cmdl/test* can be examined to see how to register commands with the commandline and how to create a command execution loop.
When running the application, the registered commands can be invoked interactively via a UART interface.
For instance, the multiply command multiplies two numbers that are passed in as parameters via the commandline.
The project in *subsystems/runloop/test* can be examined to see how to work with the runloop subsystem.

The *CAN-inspector* application in the *applications* directory allows to interactively set up the mcp2515 CAN device's configuration.
Once the mcp2515 driver is initialized, the CAN-inspector can be used to sniff a CAN bus and to interactively inject CAN messages.


Writing Makefiles
----------------

The build environment requires a *Makefile* in each directory which contains compilable resources.
These makefiles provide the build environment with information about the project structure, source and header files, libraries' names, and dependencies, so that the build environment can process all resources correctly.
Each makefile declares its own location in the directory tree by the variables TOPDIR and SUBDIR.
These variables are required by the build environment to automatically navigate through directories and to resolve dependencies.
TOPDIR declares the path from the makefile's directory to the topmost directory in AVR3nk.
The SUBDIR variable declares the path from the topmost directory in AVR3nk to the makefile's directory.
For instance, drivers/uart/Makefile contains these lines:

    TOPDIR := ../..
    SUBDIR := drivers/uart

Each makefile also includes the build environment which makes the targets and their corresponding functionality available in place:

    include $(TOPDIR)/env/make/Make.conf

Depending on whether a makefile is associated with a *directory*, a *library* or an *application*, it must provide a specific set of variables:

* **Directory**:
A directory makefile manages subdirectories but does not directly manage compilable resources.
Besides TOPDIR and SUBDIR, it declares the DIRECTORIES variable, which may hold the names of various subdirectories.
On recursive builds, the invoked build target(s) will be processed within each of the directories listed in DIRECTORIES.
For instance, when executing *make build* in the drivers directory, *make build* will be executed for each of the drivers. <br/>
You can execute the following make targets for a directory: *clean*, *headers*, *build*, *install*, *all* (all is the default and typically expands to: *clean build install*).

* **Library**:
In AVR3nk, the *drivers* and the *subsystems* directories contain libraries.
A library makefile declares the variables LIBRARY, TOPDIR, SUBDIR, SOURCES, HEADERS, DEPENDENCIES, and MCU. <br/>
The LIBRARY variable declares the library's name, which should start with a lib prefix by convenience.
It defines the name of the pre-compiled library archive which is copied to the install directory when the *install* target is invoked.
SOURCES contains the names of all source files of the library.
The HEADERS variable holds all public headers of the library, i.e., headers that will be included by other modules.
These headers are copied to the install directory when the *headers* target is invoked.
The DEPENDENCIES variable declares dependencies on other libraries and contains the paths from the AVR3nk's top directory to the directories of these libraries.
It is required to build application dependencies recursively (see below).
The MCU variable declares all MCUs that are supported by that library.
If some of the supported MCUs differ in register layout, these differences must be taken into account by macro switches in the source code. <br/>
You can execute the following make targets for a library: *clean*, *headers*, *build*, *install*, *all* (default).

* **Application**:
An application always contains a module with a main() function.
It typically corresponds to a project with a specific microcontroller, a specific board and a specific pinout configuration.
An application makefile declares the variables APPLICATION, TOPDIR, SUBDIR, SOURCES, APP\_MACROS, LIBRARIES, DEPBUILDS\_ITERATIVE, DEPBUILDS\_RECURSIVE, and MCU. <br/>
The APPLICATION variable declares the application's name and should typically correspond to the application's directory name.
The APP\_MACROS variable holds macro definitions that are passed through to the compiler with the -D compiler flag.
This allows to override the default values of libraries' macros before these libraries are built and linked to the application.
For instance, the UART\_TX\_LED macro defaults to B,3 in drivers/uart/src/uart.h, but it can be overwritten by the application by adding UART\_TX\_LED=B,4 to APP\_MACROS.
The LIBRARIES variable declares pre-compiled libraries that are statically linked to the application.
It should contain the names of the libraries (the LIBRARY variable in the library makefile) without the lib prefix.
A library should be linked by means of the LIBRARIES variable whenever its default macro settings comply with the application's requirements.
Otherwise, if an application requires a custom configuration (e.g., pinout) of a library, the library should be referenced by means of DEPBUILDS\_ITERATIVE or DEPBUILDS\_RECURSIVE (but never both at the same time) in order to build it specifically for the application.
DEPBUILD\_ITERATIVE declares a set of directories (relative to the topmost directory of AVR3nk).
All libraries listed in DEPBUILD\_ITERATIVE are built in the build subdirectory of the application with the application's corresponding APP\_MACROS settings and are finally linked from there to the application.
Alternatively to DEPBUILD\_ITERATIVE, DEPBUILD\_RECURSIVE allows to set up dependency builds by only specifying the leaves in the dependency trees.
It automatically resolves dependencies by evaluating the DEPENDENCIES variables of the corresponding libraries.
Note that DEPBUILD\_RECURSIVE should not be used if DEPBUILD\_ITERATIVE is used and vice versa.
The MCU variable contains the supported MCU(s) of the application.
If more than one MCU is specified, the first MCU is used when invoking the *program* target, i.e., it is assumed that the first specified MCU type is connected to your programming hardware. <br/>
You can execute the following make targets for an application: *clean*, *build*, *all* (default), *program*, *size*.


Background
----------------

AVR3nk was initially created in 2009 at the [Sailing Team Darmstadt e.V.](http://www.st-darmstadt.de), a students' association at [Technische Universität Darmstadt](http://www.tu-darmstadt.de).
The team aims to build autonomous and energy-self-sufficient sailing robots.
AVR3nk contains the team's basic infrastructure for developing sensor and actuator devices.


Contributing
----------------

If you like to contribute to AVR3nk, feel free to contact me.


[make]: http://www.gnu.org/software/make/
