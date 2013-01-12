====== Board Support Package and Examples ======

This package includes the board support package, drivers and examples 
for the EFM32GG_DK3750 Giant Gecko development kit from Energy Micro.

====== Dependencies ======

This package _requires_ the EFM32 CMSIS package to be installed at the
same level as this package. If you did not get this as part of the 
Simplicity Studio package, you should also download and install the
EFM32 CMSIS package. See the Changes file for required version.

The EFM32 CMSIS package includes the necessary EFM32 drivers, register,
and bit field definitions that are required for the included BSP and 
example projects.

The CMSIS package requires C99 support, and so does this package.

====== File structure ======

kits/EFM32GG_DK3750/bsp
   C source and header files for kit specific functionality, such as 
   enabling kit specific peripherals to be accessible from the EFM32 
   (configures on board analog switches - that are there to prevent 
   current leakage, gives access to LEDs, dip switches, joystick, i2c 
   devices and so on).

kits/EFM32GG_DK3750/bspdoc
   Doxygen documentation of the BSP. Use a web browser and open the
   index.html file in the html directory.

kits/EFM32GG_DK3750/drivers
   Various drivers for kit specific components.

kits/EFM32GG_DK3750/examples
   Several example projects demonstrating various capabilities of the
   EFM32GG990F1024 kit specific functionality.
   Project files for various IDEs/compilers are in subdirectories of
   each example. Use these as a starting point for your own development
   and prototyping of EFM32 Giant Gecko software.

====== Updates ======

The latest version of this package is always available from 
    http://www.energymicro.com/downloads
or through Simplicity Studio, see
    http://www.energymicro.com/simplicity

====== License ======

License information for use of the source code is given at the top of 
all C files.

(C) Copyright Energy Micro 2012. All rights reserved. 
