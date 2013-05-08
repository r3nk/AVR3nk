AVR3nk
================

AVR3nk is a cross-platform build environment and a library collection for 8-bit AVR microcontrollers. It allows to maintain libraries and applications for different kinds of 8-bit AVR microcontrollers and is especially useful for team development. You can use AVR3nk on Linux, Mac OS X, and Windows. AVR3nk includes an interrupt-driven and buffered driver for dual UART operation, a commandline interface (e.g., for interactive module testing), as well as an interrupt-driven driver for the MCP2515 [CAN](http://en.wikipedia.org/wiki/CAN_bus) controller, which interfaces via [SPI](http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus). 


Motivation
----------------

Typically, the software running on a microcontroller has project-specific requirements on clock frequency, pinout and other settings. Since many of these parameters do not change at runtime, these parameters are typically defined at compile time rather than at runtime for efficiency reasons. Therefore, libraries usually have to be compiled with project-specific adjustments prior to being linked to an application. I have seen many people copying library code to their project directories and adjusting it to project-specific requirements. However, having duplicate code in multiple locations compromises maintainability of the original library's source code, especially if it's written by yourself. 

AVR3nk's build environment allows to organize libraries in a single place. Static parameters (i.e., parameters which do not change at runtime) are defined as macros in the libraries' header files and can be overridden through individual build settings of applications. Furthermore, AVR3nk's build environment allows to declare libraries to be capable of handling various kinds of MCUs. This allows to reuse the same library code in different projects which target different AVR devices. 

The build environment is based on [Make][make] and is controlled via terminal. However, you do not have to write complicated makefiles to get your projects working. In order to build a new library or application, you only have to provide a makefile with some basic information, such as source and header files and the project's location in the directory tree. Lastly, a custom makefile must include the build environment (a one-liner) to provide the full power of cross-directory builds, recursive builds, device programming, and so on. 


Setup
----------------

You must install the following packages in order to use AVR3nk: 
* avr-gcc
* avr-libc
* avr-binutils
* avrdude

**Linux**: Most Linux distributions have an AVR-GCC toolchain in their packet managers. 

**Mac OS OX**: I personally prefer installing the toolchain via [MacPorts](http://www.macports.org). Alternatively, there is [CrossPack](http://www.obdev.at/products/crosspack/index.html) and [OSX-AVR](http://sourceforge.net/projects/osxavr) (I haven't tried these yet). 

**Windows**: Download and install [WinAVR](http://winavr.sourceforge.net). 

Once you've downloaded AVR3nk, you have to create a custom configuration file in *env/config*. This file will hold your user specific configuration, such as the location of your WinAVR installation (if you are on Windows) and your programming hardware settings. You should use *config.mak* as a template and name your personal file *config.mak.&lt;yourLoginName&gt;*. The template provides further instructions on each setting. Note that you have to remove any whitespaces in &lt;yourLoginName&gt;. E.g., if my login name was *Robin Klose*, my configuration file would be named *config.mak.RobinKlose*. The build environment automatically reads your personal configuration whenever you invoke any Make targets. You can also keep the configuration files of your team members under revision control and change them centrally if global changes need to be applied. 


Directory Structure
----------------

AVR3nk defines a directory structure that can easily be extended or adapted to individual needs. The drivers directory contains libraries that access a microcontroller's registers or peripheral hardware. The subsystems directory contains libraries that are stacked on top of other libraries but which do not directly interact with hardware. The applications directory contains software projects related to their corresponding hardware projects, i.e., applications are designed for a particular MCU and incorporate specific pinout settings. Applications can finally be linked to libraries and be programmed to AVR hardware. The env directory contains the build environment and user specific configuration files. The docs directory contains coding guidelines, templates and other documents. There are library templates (library.h and library.c) as well as an application template (main.c). Further, there are makefile templates for directories (Makefile\_dir), libraries (Makefile\_lib) and applications (Makefile\_app). When using one of these templates for a new makefile, the new makefile should just be named *Makefile*. The build environment also creates an install directory for header files and pre-compiled library archives. The build environment adds the install directory to the standard search path of the compiler and the linker in order to find header files and pre-compiled libraries. 


Make Targets
----------------

The AVR3nk build environment allows to build libraries and applications by running Make commands in a terminal, such as *make all*, *make build* or *make program*. When one of the specified Make targets is invoked in a directory, it is executed recursively within all subdirectories. E.g., in order to build all libraries and applications at once, execute *make all* in AVR3nk's top directory. 

The following Make targets are available: 
* **clean**: Removes all files generated locally by the build environment, such as object code, library archives, Intel hex programming files, etc. When *make clean* is invoked in AVR3nk's top directory, the install directory is removed prior to executing the clean command recursively in each subdirectory. Make a direcotry clean when you need to make sure that all source files are compiled anew in a subsequent call of the *build* target. 
* **build**: Compiles the source code. For libraries, it will also create (or update) a library archive if source files were updated. For an application, it will also build library dependencies into the application's build directory and link everything to an executable which can be programmed to AVR hardware. The build command compiles only files that were changed since the last compilation. 
* **headers**: Copies the header files of a library to the install directory. 
* **install**: Copies the pre-compiled archive file of a library to the install directory. 
* **all**: Rebuilds everything anew by invoking the targets *clean*, *build* and *install*. 
* **program**: This target is only valid for applications. It invokes avrdude to write the programming file to the AVR microcontroller. 
* **size**: Displays the size of a program. This target works only for applications. 


Sample Projects
----------------

The application directory contains three sample projects: *CAN-inspector*, *demo\_cmdl*, and *demo\_sensor*. 

The *demo\_cmdl* demonstrates the usage of the commandline subsystem. It registers a few functions with the commandline, which can be invoked interactively via a UART interface. For instance, the multiply command multiplies two numbers, which are passed to the command as parameters. 

The *CAN-inspector* allows to interactively set up the mcp2515 CAN device's parameters. Once the mcp2515 driver is initialized, the CAN-inspector can be used to sniff a CAN bus and to interactively inject messages. 

The *demo\_sensor* application demonstrates a possible concept to create sensor applications which communicate their sensor data via CAN. 


Build Environment
----------------

The build environment requires a *Makefile* in directories which contain compilable resources. These makefiles should provide information about the project structure, source and header files, libraries' names, etc. so that the build environment can process the right resources. Depending on whether a makefile is associated with a *directory*, a *library* or an *application*, it must provide a specific set of Make variables. In a last step, each makefile includes the build environment so that the Make targets and their functionality are available in place. 

Each makefile declares its location in the directory hierarchy by the Make variables TOPDIR and SUBDIR. These variables are required by the build environment to automatically navigate between directories and to resolve dependencies. TOPDIR declares the path from the makefile's directory to the topmost directory in AVR3nk. The SUBDIR variable declares the path from the topmost directory in AVR3nk to the makefile's directory. For instance, the makefile in drivers/uart contains these lines:

    TOPDIR := ../..
    SUBDIR := drivers/uart

Furthermore, each makefile contains the following line to include the build environment, i.e., build settings, targets and rules: 

    include $(TOPDIR)/env/make/Make.conf

Basically, there are three different kinds of makefiles in the directory structure: directory, library, and application. 

**Directory**: A directory makefile manages subdirectories but does not directly manage compilable resources. Besides TOPDIR and SUBDIR, it declares the DIRECTORIES variable, which may hold the names of various subdirectories of the makefile's directory. On recursive builds, Make will automatically perform its build target(s) within each of the directories listed in DIRECTORIES. For instance, when executing *make build* in the drivers directory, *make build* will be executed for each of the drivers. <br/>
You can execute the following make targets for a directory: *clean*, *headers*, *build*, *install*, *all* (all is the default, and typically expands to: *clean build install*).

**Library**: A library contains a collection of software modules which accomplish a specific purpose without being bound to a particular application. In AVR3nk, the drivers and the subsystems directories contain libraries. A library's makefile declares the variables LIBRARY, TOPDIR, SUBDIR, SOURCES, HEADERS, DEPENDENCIES, and MCU. <br/>
LIBRARY declares the library's name, which should start with a lib prefix by convenience. It defines the name of the pre-compiled library archive which is copied to the install directory when the *install* target is invoked. <br/>
SOURCES contains the names of all source files of the library. <br/>
The HEADERS variable holds all public headers of the library, i.e., headers that should be includable by other modules. These headers are copied to the install directory when the *headers* target is invoked. <br/>
The DEPENDENCIES variable declares dependencies on other libraries and contains the paths from the AVR3nk's top directory to the directories of these libraries. It is required when building application dependencies recursively (see below). The MCU variable declares all MCUs that are supported by that library. If MCUs differ in register configuration, these differences must be taken into account for all supported MCUs, e.g., by macro switches in the source code. <br/>
You can execute the following make targets for a library: *clean*, *headers*, *build*, *install*, *all* (default).

**Application**: An application always contains a module with a main() function. It typically corresponds to a project with a specific microcontroller, a specific board and a specific pinout configuration. In AVR3nk, an application's makefile declares the variables APPLICATION, TOPDIR, SUBDIR, SOURCES, APP\_MACROS, LIBRARIES, DEPBUILDS\_ITERATIVE, DEPBUILDS\_RECURSIVE, and MCU. <br/>
APPLICATION declares the application's name and should typically correspond to the application's directory name. <br/>
The APP\_MACROS variable holds values that are passed through to the compiler with the -D compiler flag. This allows to override libraries' macros' default values before these libraries are built and linked to the application. As macro-defined library settings are hard-coded in precompiled library archives, AVR3nk provides a dependency build mechanism to allow to compile a library for an application with its application-specific settings. For instance, the UART\_TX\_LED macro defaults to B,3 in drivers/uart/src/uart.h, but it can be overwritten by the application by adding UART\_TX\_LED=B,4 to APP\_MACROS. This is especially useful to configure pinouts of drivers in different projects. <br/>
The LIBRARIES variable declares pre-compiled libraries that are statically linked to the application. It should contain the names of the libraries specified in the LIBRARY variables of the libraries' makefiles, but without the lib prefix. A library can be linked by means of the LIBRARIES variable whenever its default settings comply with the application's requirements. Otherwise, if an application requires a custom configuration (e.g., pinout) of a library, the library should be referenced by means of DEPBUILDS\_ITERATIVE or DEPBUILDS\_RECURSIVE (but never both at the same time) in order to build it specifically for the application. <br/>
DEPBUILD\_ITERATIVE declares a set of directories (relative to the topmost directory of AVR3nk). All libraries listed in DEPBUILD\_ITERATIVE are built in the build subdirectory of the application with the application's corresponding APP\_MACROS settings and are finally linked from there to the application. <br/>
Alternative to DEPBUILD\_ITERATIVE, DEPBUILD\_RECURSIVE allows to set up dependency builds by only specifying the latest dependencies in the dependency trees. It automatically resolves further dependencies by evaluating the DEPENDENCIES variables of the corresponding libraries. Note that DEPBUILD\_RECURSIVE should not be used if DEPBUILD\_ITERATIVE is used and vice versa. <br/>
The MCU variable contains the supported MCU(s) of the application. If more than one MCU is specified, the first MCU is used when invoking the *program* target, i.e., it is assumed that the first specified MCU type is connected to your programming hardware. 
You can execute the following make targets for an application: *clean*, *build*, *all* (default), *program*, *size*. 


Background
----------------

AVR3nk was initially created in 2009 at the [Sailing Team Darmstadt e.V.](http://www.st-darmstadt.de), a students' association at [Technische Universität Darmstadt](http://www.tu-darmstadt.de). The team aims to build autonomous and energy-self-sufficient sailing robots. AVR3nk contains the team's basic infrastructure for developing sensor and actuator devices. 


Contributing
----------------

If you like to contribute to AVR3nk, feel free to contact me. I'm willing to add further libraries.

So far, I've mainly tested the libraries on atmega644p, atmega644 and atmega16 devices. Adding support for other devices is easy, but I won't buy all the different microcontrollers just to test my code. So if you have successfully patched one or several libraries for other AVR devices, drop me a line. 


[make]: http://en.wikipedia.org/wiki/Make_(software)
