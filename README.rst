.. raw:: html

   <a href="https://www.ambiq.com">
     <p align="center">
       <picture>
         <img src="docs/Ambiq Banner.png">
       </picture>
     </p>
   </a>


This module provides the required Ambiq Hal drivers, utility files and libraries
needed to build a Zephyr application running on Apollo silicon.

This module is updated regularly in order for Ambiq users to benefit from the
latest AmbiqSuite versions, features and bug fixes.
Updates are generally done once in each Zephyr release, preferably soon after
the opening of the merge window to give time to users to use it, potentially
report issues if some are faced and get them fixed before the new version in
released.

URL
===

   https://github.com/AmbiqMicro/ambiqhal_ambiq/tree/ambiq-stable

Status
======

+------------------------+--------------------------------+
| SOC                    | AmbiqSuite SDK Revision        |
+========================+================================+
| Apollo3 Blue           | v3.1.1                         |
+------------------------+--------------------------------+
| Apollo3 Blue Plus      | v3.1.1                         |
+------------------------+--------------------------------+
| Apollo4 Blue Plus      | v4.4.0                         |
+------------------------+--------------------------------+

Purpose
=======

   With added proper shims adapting it to Zephyr's APIs, ambiq will provide
   peripheral drivers for Apollo SoCs.

Description
===========

   ambiq hal is a standalone set of drivers for peripherals present in Ambiq
   Micro's SoCs. It originated as an extract from the AmbiqSuite SDK.
   The intention was to provide drivers that can be used in various
   environments without the necessity to integrate other parts of the SDK
   into them. This package also contains definitions of register structures
   and bitfields for all supported SoCs.

Use Ambiq HAL in your application
=================================

   It may happen that you want to access Ambiq HAL APIs in your application,
   either because it is not available as a zephyr API or because you want to
   by-pass use of existing Zephyr API.
   In this case, you need to create a Kconfig file in your application which
   contains the following:

.. code-block:: none

   mainmenu "MYAPP"

   source "Kconfig.zephyr"
   config MYAPP_AMBIQ
     default y
     bool
     select AMBIQ_HAL_USE_FOO

Additionally, you need to add the following includes:

.. code-block:: c

   #include <soc.h>
   #include <am_mcu_apollo.h>

Dependencies
============

   CMSIS header files

Maintained-by
=============

   External

License
=======

   BSD-3-Clause
