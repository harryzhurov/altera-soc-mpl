
>>>>>>>>>>>>>>>>>>>>>>>  Intel (Altera) MPL (Minimal Preloader)   <<<<<<<<<<<<<<<<<<

********
Summary:
********

    The Minimal Preloader (MPL) is an alternative for the GPL Preloader.  It
    uses the BSD license and may be freely distributed and modified according to
    the terms of that license.  The MPL supports a subset of features supported
    by Altera's GPL Preloader.

    The MPL initializes the PLLs, reset signals, configures IOCSR and pinmuxing
    and performs other configuration based on the preloader generator file
    settings. It can also load the FPGA from a boot source if desired.  It then
    reads a secondary image from a boot source into RAM and hands control to
    that image.

    This version of MPL supports booting from QSPI, SDMMC, NAND and FPGA. Most
    of the MPL testing used CSEL set to 0.  Other CSEL settings may work but
    are not tested.

    For SDMMC, you may put the secondary image on a FAT partition or on the raw
    partition that contains the MPL image.  For loading the FPGA in this case,
    the secondary image and the FPGA image must both be on the FAT partition.

    The MPL uses Altera HWLib drivers for most of its functionality. It also
    uses Altera HWLib SoCAL folders for the memory map definitions and basic
    read/write commands.

    The MPL supports both the ARMCC and GNU GCC compilers. The MPL supports 
    Cyclone V (CV), Arria V (AV), & Arria 10 (A10) SoC devices. The example 
    commands shown in this file are for Cyclone V SoC. Please modify the 
    examples as needed to select appropriate file names.

    For Arria 10, UEFI provides a full-featured bootloader and is the preferred
    non-GPL method of booting the SoC.  The MPL provides a minimal set of 
    functionality and should be considered by those wanting a basic
    loader, or a small application to do chip initialization.

    For this release for Arria 10, Windows MPL builds do not compile.

    You can find more information about UEFI for Arria 10 here:

    http://www.altera.com/en_US/pdfs/literature/ug/ug-a10-soc-boot.pdf

    http://github.com/altera-opensource/uefi-socfpga

******
Setup:
******

    For Windows builds, you must have Altera's SoC Embedded Design Suite
    (SoCEDS) installed.  After installing SoCEDS, you will need to understand
    how to start the Embedded Command Shell.  Typically this is done with a
    shortcut in the Altera folder, under the SoC EDS Embedded Design Suite.

    In Windows, you must first import the project into DS-5 before building.  In
    Embedded Command Shell, all commands are executed under the DS-5 workspace
    directory.  (e.g. ~/Documents/DS-5\ Workspace/Altera-SoCFPGA-HardwareLib-MPL)

    The MPL project can be built in 3 ways:

    (1) In Embedded Command Shell in Windows
    (2) In DS-5.
    (3) From a Linux host machine.

    The serial port should be set to 115200 baud, 8N1.

    All development work and regression testing for MPL is done on the latest
    version of Altera's development kits.  Currently, development is done on:

    - The Cyclone V SoC development kit, revision E
    - The Arria V SoC development kit, revision C
    - The Arria 10 SoC development kit, revision C1

********************
Directory Structure:
********************

  ./
     - Makefile
          -- Contains commands in makefile syntax to build the MPL project
     - config.mk
          -- Sets variables used by the MPL Makefile.
     - mpl.scat
          -- ARM Compiler linker script that links to Cyclone/Arria V OCRAM 
                 Cyclone/Arria V:   0xffff0000
     - mpl_a10.scat
          -- ARM Compiler linker script that links to Arria 10 OCRAM
                 Arria 10:          0xfffe0000
     - mpl_gcc.lds
          -- GCC Compiler linker script that links to Cyclone/Arria V OCRAM
                 Cyclone/Arria V:   0xffff0000
     - mpl_a10.lds
          -- ARM Compiler linker script that links to Arria 10 OCRAM
                 Arria 10:          0xfffe0000
     - mpl_config.h
          -- A header file to include configuration options which are not part
             of the handoff files.  This is mainly for selecting FPGA
             configuration options.
     - alt_config.h 
          -- A header file to include configuration options that are different
             between CV/AV/A10, and to include or exclude code sections for
             certain QSPI devices
     - mpl_fpga.scat
          -- ARM Compiler linker script for locating the program in FPGA Memory
                Cyclone/Arria V:    0xc0000000
                Arria 10:           0xc0000000
     - mpl_gcc_fpga.lds
          -- GNU Compiler linker script for locating the program in FPGA memory
                Cyclone/Arria V:    0xc0000000
                Arria 10:           0xc0000000


  ./core
     - board_init.c
          -- Performs initialization such as PLLs, IOSDC, resets, etc. for
                Cyclone/Arria V 
     - board_init_a10.c
          -- Performs initialization such as PLLs, IOSDC, resets, etc.
             for Arria 10
     - memory_init_a10.c
          -- Initializes/calibrates the A10 SDRAM through the EMIF
     - mpl.c
          -- The main MPL file for Cyclone/Arria V. Reads passdown boot source 
             and jumps to the appropriate boot source to load the next image 
             (QSPI, SDMMC, NAND)
     - mpl_a10.c
          -- The main MPL file for Arria 10.  Reads passdown boot source 
             and jumps to the appropriate boot source to load the next image 
             (QSPI, SDMMC)
     - startup.S
          -- A low level startup file that initializes basic Cortex-A9
             components
     - qspi_load.c
          -- For QSPI boot, reads and interprets the header from the secondary
             image, then reads the secondary image from flash into RAM, then
             jumps to it
     - qspi_load_a10.c
          -- For QSPI boot on A10, reads and interprets the header from a single
             monolithic RBF, reads the image into the FPGA Manager, then reads and
             interprets the header from the secondary image, loads the image into RAM,
             and then jumps to it. 
     - sdmmc_load.c
          -- Like qspi_load.c, but for SDMMC
     - nand_load.c
          -- Like qspi_load.c, but for NAND. Does not support FPGA load.
     - alt_printf.c
          -- Outputs characters to a stream
     - semihost.c/mpl_log.c
          -- Outputs characters to a stream using semihosting
     - system_manager_pinmux_a10.c
          -- Contains the Pin Multiplexing for Arria10. This file should be
             edited/replaced with the output from your Quartus project
     - clk_data.c
          -- Contains the clock configuration for Arria10. This file should be
             edited/replaced with the output from your Quartus project

  ./core/Altera_ip
     -- Drivers for core Altera SoC blocks for IOCSR/pinmux programming
        and SDRAM initialization and calibration

  ./fatfs
     -- A copy of the FatFs open source FAT implementation.  Read the file
        00README.txt in this directory for more information, including the
        license terms for FatFs.  You can remove this directory if
        you do not wish to use FAT with MPL.


**************
General Notes:
**************

  The MPL output file name varies based on options passed. For example,

     - mpl_C5_q-mkpimage.bin  => generated for Cyclone V SoC & QSPI boot source
     - mpl_C5_s-mkpimage.bin  => generated for Cyclone V SoC & SDMMC boot source
     - mpl_C5_n-mkpimage.bin  => generated for Cyclone V SoC & NAND boot source
     - mpl_A5_q-mkpimage.bin  => generated for Arria V SoC & QSPI boot source
     - mpl_A5_s-mkpimage.bin  => generated for Arria V SoC & SDMMC boot source
     - mpl_A5_n-mkpimage.bin  => generated for Arria V SoC & NAND boot source
     - mpl_A10_s-mkpimage.bin => generated for Arria 10 SoC & SDMMC boot source
     - mpl_A10_q-mkpimage.bin => generated for Arria 10 SoC & QSPI boot source


***********************************
Building and Booting HPS From QSPI:
***********************************

    QSPI boot is currently supported for Cyclone V SoC, Arria V SoC and Arria 10 SoC.

    CYCLONE V/ARRIA V METHOD:

    (1) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/Altera/socfpga/build.h'
        matches the boot selection below. For QSPI the
        CONFIG_PRELOADER_BOOT_FROM_QSPI must be set to 1 in 'build.h'.

    (2) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/Altera/socfpga/sdram/sequencer_defines.h'
        is either configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC
        (set CYCLONEV to 1).

    (3) Build mpl_CV_q-mkpimage.bin or mpl_AV_q-mkpimage.bin. This example
        builds the MPL for Cyclone V SoC.

        (3.1) In DS-5, Clean Project, or
              In Embedded Command Shell or Linux, type "make clean"

        (3.2) Set the correct values in 'config.mk':

              BOOT_SOURCE ?= QSPI
              DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5,
                                     A10   = build code for Arria10)
              COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain     )

              HANDOFF_BASE      ?= [Root directory of FPGA project]
              HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
              HANDOFF_DIR       ?= [Preloader generated files directory]

        (3.3) In DS-5 click Build Project; or
              In Embedded Command Shell or Linux, type make.

        This gives you an MPL image that will boot from QSPI.

    (4) Build your application binary and process it with the mkimage tool.
        Please consult the SoCEDS documentation for more information on this step:

          http://www.altera.com/literature/ug/ug_soc_eds.pdf

        For the purposes of this example, application.img is the name of the
        application image.

    (5) Set the BOOTSEL jumpers to QSPI:

        -- BOOTSEL0: . [. .]
        -- BOOTSEL1: [. .] .
        -- BOOTSEL2: [. .] .

    (6) Open a Embedded Command Shell.  Connect a USB cable to the USB-BlasterII
        port on the development board.

        Make sure that the USB-BlasterII is visible on the host computer:

        (6.1) jtagconfig

        Copy the MPL and secondary images to the QSPI:

        (6.2) quartus_hps -c USB-BlasterII -o P -a 0x0000 mpl_C5_q-mkpimage.bin

        (6.3) quartus_hps -c USB-BlasterII -o P -a 0x60000 application.img

        The value 0x60000 comes from the handoff file variable
        CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE.  It is the starting address in
        QSPI of the secondary image.

        The -c option instructs the programmer to use USB-BlasterII as the
        programming port.  If there are multiple USB-BlasterIIs connected to the
        computer, you must use the cable number (1,2,etc) to designate which USB
        Blaster to use (e.g. quartus_hps -c 1 instead of -c USB-BlasterII)

    (7) Power up the board while monitoring the UART.

    ARRIA 10 METHOD:
   
    (1) Modify the Makefile to have HANDOFF_BASE point to the directory of your Arria 10
        QSPI FPGA design. The Arria 10 QSPI FPGA design needs to have the
        EARLY_IO_RELEASE option turned off before compilation. This is an option
        inside Quartus.

    (2) Build mpl_A10_q-mkpimage.bin. This example builds the MPL for Arria 10 SoC.

        (2.1) In DS-5, Clean Project, or
              In Embedded Command Shell or Linux, type "make clean"

        (2.2) Set the correct values in 'config.mk':

              BOOT_SOURCE ?= QSPI
              DEVICE      ?= A10
              COMPILER    ?= GNU   (GNU = GCC toolchain, ARM  = ARM toolchain)

        (2.3) Set "CONFIG_MPL_FPGA_LOAD" to 1 in "mpl_config.h"

        (2.3) In DS-5 click Build Project; or
              In Embedded Command Shell or Linux, type make.

    This gives you an MPL image that will boot from QSPI.

    (3) Build your application binary and process it with the mkimage tool.
        Please consult the SoCEDS documentation for more information on this step:

          http://www.altera.com/literature/ug/ug_soc_eds.pdf

        For the purposes of this example, application.img is the name of the
        application image.

    (4) With the FPGA design compiled with EARLY_IO_RELEASE set to off, convert
        the .sof file to a single .rbf file and add the mkimage header.
 
        (4.1) Open Embedded Command Shell
 
        (4.2) quartus_cpf -c -o bitstream_compression=on fpga.sof fpga.rbf

        (4.3) mkimage mkimage -A arm -T firmware -C none -O u-boot -a 0 -e 0 \
              -n "RBF" -d fpga.rbf fpga.rbf.mkimage
   
    (5) Plug the QSPI card into your Arria 10 development kit. The card sets the BSELs
        to QSPI.

    (6) Open an Embedded Command Shell.  Connect a USB cable to the USB-BlasterII
        port on the development board.

        Make sure that the USB-BlasterII is visible on the host computer:

        (6.1) jtagconfig

        Copy the MPL and secondary images to the QSPI:

        (6.2) quartus_hps -c USB-BlasterII -o P -a 0x0000 mpl_A10_q-mkpimage.bin

        (6.3) quartus_hps -c USB-BlasterII -o P -a 0x720000 fpga.rbf.mkimage

        (6.4) quartus_hps -c USB-BlasterII -o P -a 0x120000 application.img

        The value 0x120000 is the starting address in QSPI of the secondary image.

        The value 0x720000 is the starting address in QSPI of the fpga image.

        The -c option instructs the programmer to use USB-BlasterII as the
        programming port.  If there are multiple USB-BlasterIIs connected to the
        computer, you must use the cable number (1,2,etc) to designate which USB
        Blaster to use (e.g. quartus_hps -c 1 instead of -c USB-BlasterII)

    (7) Power up the board while monitoring the UART.


************************************
Building and Booting HPS From SDMMC:
************************************

    SDMMC boot is currently supported for Cyclone V SoC and Arria V SoC.  It is possible
    that it will work on Arria 10, but it has not been tested.

    (1) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/build.h'
        matches the boot selection. For SDMMC, CONFIG_PRELOADER_BOOT_FROM_SDMMC
        must be set to 1.

    (2) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/sdram/sequencer_defines.h'
        is configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC (set
        CYCLONEV to 1).

    (3) Build mpl_A5_s-mkpimage.bin or mpl_C5_s-mkpimage.bin. This example
        builds the MPL for Cyclone V SoC.

        (3.1) In DS-5, Clean Project; or
              In Embedded Command Shell or Linux, type "make clean"

        (3.2) Set the correct values in 'config.mk':

              BOOT_SOURCE ?= SDMMC
              DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5 )
              COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain     )

              HANDOFF_BASE      ?= [Root directory of FPGA project]
              HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
              HANDOFF_DIR       ?= [Preloader generated files directory]

        (3.3) Then in DS-5 click Build Project; or
              In Embedded Command Shell or Linux, type make.

        This gives you an MPL image that will boot from SDMMC.

    (4) Build your application binary and process it with the mkimage tool.
        Please consult the SoCEDS documentation for more information on this step:

          http://www.altera.com/literature/ug/ug_soc_eds.pdf

        For the purposes of this example, application.img is the name of the
        application image.

    (5) Set the BOOTSEL jumpers to SDMMC:

        -- BOOTSEL0: . [. .]
        -- BOOTSEL1: . [. .]
        -- BOOTSEL2: [. .] .

    (6) Open a SoCEDS Command Shell.
        Copy the MPL and secondary images to the SD card:

        (6.1) Plug in your SD card and determine which partition it is.

              Linux: Type "dmesg | tail":
                [2736576.121602] sd 15:0:0:0: [sdb] Attached SCSI removable disk

              Windows: The system should create a new drive letter

        (6.2) Flash the MPL image to sdb3 using dd command:

              Linux:
              >>> sudo alt-boot-disk-util -p mpl_C5_s-mkpimage.bin -a write -d /dev/sdb

              Windows: (assuming the system assigned drive E)
              >>> alt-boot-disk-util.exe -p mpl_C5_s-mkpimage.bin -a write -d e

        (6.3) Copy the application.img to sdb3 after the mpl_C5_s.bin

              Linux:
              >>> sudo alt-boot-disk-util -b application.img -a write -d /dev/sdb

              Windows: (assuming drive E was assigned)
              >>> alt-boot-disk-util.exe -b application.img -a write -d e

        If you have configured the MPL to include FAT support, you may put the
        application.img file on the FAT partition of the SD card rather than the
        RAW partition.

    (7) Insert the SDMMC card into the DevKit and power up the board.


***********************************
Building and Booting HPS From NAND:
***********************************

    NAND boot is currently supported for Cyclone V SoC.  It is possible that it will
    work on Arria V and Arria 10, but it has not been tested.

    (1) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/Altera/socfpga/build.h'
        matches the boot selection below. For NAND the
        CONFIG_PRELOADER_BOOT_FROM_NAND must be set to 1 in 'build.h'.

    (2) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/Altera/socfpga/sdram/sequencer_defines.h'
        is either configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC
        (set CYCLONEV to 1).

    (3) Build mpl_C5_n-mkpimage.bin or mpl_A5_n-mkpimage.bin. This example
        builds the MPL for Cyclone V SoC.

        (3.1) In DS-5, Clean Project, or
              In Embedded Command Shell or Linux, type "make clean"

        (3.2) Set the correct values in 'config.mk':

              BOOT_SOURCE ?= NAND ( SDMMC = Boot from SDMMC,   QSPI = Boot from QSPI,
                                     NAND = Boot from NAND,    
                                     NONE  = Do not boot. This is for combined MPL/Apps )
              DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5,
                                     A10   = build code for Arria10)
              COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain    )

              HANDOFF_BASE      ?= [Root directory of FPGA project]
              HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
              HANDOFF_DIR       ?= [Preloader generated files directory]

        (3.3) In DS-5 click Build Project; or
              In Embedded Command Shell or Linux, type make.

        This gives you an MPL image that will boot from NAND.

    (4) Build your application binary and process it with the mkimage tool.
        Please consult the SoCEDS documentation for more information on this step:

          http://www.altera.com/literature/ug/ug_soc_eds.pdf

        For the purposes of this example, application.img is the name of the
        application image.

    (5) Set the BOOTSEL jumpers to NAND:

        -- BOOTSEL0: . [. .]
        -- BOOTSEL1: [. .] .
        -- BOOTSEL2: . [. .]

    (6) Open a Embedded Command Shell.  Connect a USB cable to the USB-BlasterII
        port on the development board. Open DS-5 and connect to the processor.

        Copy the MPL and secondary images to the NAND:

        (6.1) Either load or boot to u-boot on the target processor
        (6.2) Type these commands in u-boot:
            (6.2.1) nand info - Gets the info of your NAND device
            (6.2.2) nand erase 0x0 0x40000 - Erases the MPL mkpimage area
            (6.2.3) mw.b 0x00008000 0xFF 0x40000 - Clear out some memory
        (6.3) Stop the processor in DS-5
        (6.4) Load the MPL NAND mkpimage into memory
            (6.4.1) DS-5 command: restore mpl_C5_n-mkpimage.bin binary 0x00008000
        (6.5) Click continue in DS-5 to continue running u-boot
        (6.6) Type these commands in u-boot:
            (6.6.1) nand write 0x00008000 0x0 0x40000 - Write MPL to NAND
            (6.6.2) nand read 0x08008000 0x0 0x40000 - Read MPL back into memory
            (6.6.3) cmp.b 0x00008000 0x08008000 0x40000 - Verify it was written
            (6.6.4) nand erase 0xC0000 0x60000 - Erases the second image area
            (6.6.5) mw.b 0x00008000 0xFF 0x60000 - Clear out some memory
        (6.7) Stop the processor in DS-5
        (6.8) Load the second NAND image (the image MPL is loading) into memory
            (6.8.1) DS-5 command: restore (second image) binary 0x00008000
        (6.9) Click continue in DS-5 to continue running u-boot
        (6.10) Type these commands in u-boot:
            (6.10.1) nand write 0x00008000 0xC0000 0x60000 - Write image to NAND
            (6.10.2) nand read 0x08008000 0xC0000 0x60000 - Read image back into memory
            (6.10.3) cmp.b 0x00008000 0x08008000 0x60000 - Verify it was written


        The value 0xC0000 comes from the handoff file variable
        CONFIG_PRELOADER_NAND_NEXT_BOOT_IMAGE.  It is the starting address in
        NAND of the secondary image.

    (7) Reset/Power up the board while monitoring the UART.


*****************************************************
Building and Booting HPS Directly From an FPGA Image:
*****************************************************

    Booting from FPGA is different from other methods of booting.  For this
    method, the FPGA is programmed before the ARM processor boots.  The FPGA
    contains a small ROM containing the preloader image. During boot, the internal
    ROM hands control to the preloader in FPGA memory, and then the preloader
    loads the secondary image from another boot source, usually QSPI or SDMMC.

    This means that to use FPGA boot with MPL, you must:

      - design an FPGA with a ROM with a preloader
      - build your FPGA image
      - build the MPL, based on the FPGA handoff information
            (see directions below)
      - put the MPL image in the FPGA project
      - rebuild the FPGA, now with the MPL binary inside
      - use the FPGA image to boot

    Before running this MPL, you should put the secondary image on QSPI or SDMMC
    using the steps outline in the previous sections.

    To build the MPL for this use case:

    (1) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/build.h'
        selects where the next stage of boot will be found.

        For SDMMC, the CONFIG_PRELOADER_BOOT_FROM_SDMMC must be set to 1.
        For QSPI, the CONFIG_PRELOADER_BOOT_FROM_QSPI must be set to 1.

    (2) Ensure the preloader header file
        '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/sdram/sequencer_defines.h'
        is either configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC
        (set CYCLONEV to 1).

    (3) Build the MPL hex file (mpl_C5_s.hex for example). This example
        builds the MPL for Cyclone V SoC.

        (3.1) In DS-5, Clean Project; In Embedded Command Shell or Linux, type "make clean"

        (3.2) Set correct values in 'config.mk':

            BOOT_SOURCE ?= QSPI  ( SDMMC = Boot from SDMMC,   QSPI = Boot from QSPI    )
            DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5 )
            COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain     )

            HANDOFF_BASE      ?= [Root directory of FPGA project]
            HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
            HANDOFF_DIR       ?= [Preloader generated files directory]

        (3.3) Add BOOT_FROM_FPGA to 'config.mk' to indicate that MPL will be running from the FPGA

        (3.4) In DS-5, click Build Project; in Embedded Command Shell or Linux, type make.

        These steps create a MPL hex file for SDMMC or QSPI boot.  This file should be added
        to the Qsys design.

    (4) In Qsys, select the hex file as the contents of memory for the
        FPGA and run "Generate".

        For a more in-depth description of creating an FPGA image with a memory
        section, see:

        http://www.alterawiki.com/wiki/SocBootFromFPGA

    (5) In Quartus, compile the project to produce a SOF file.

    (6) Set the BOOTSEL jumpers to boot from FPGA:

        -- BOOTSEL0: [. .] .
        -- BOOTSEL1: . [. .]
        -- BOOTSEL2: . [. .]

    (7) Use the Quartus programmer to load the Qsys file to the FPGA.

    (8) Use the cold reset button to reset the processor and observe the boot
        sequence on the UART.

*****************************************************
Adding FPGA Configuration Programming During Startup:
*****************************************************

    The MPL can be configured to program the FPGA upon boot.  This allows any
    additional hardware in the FPGA to be up and visible in the memory space
    before the secondary image runs.

    The file mpl_config.h defines the location of the FPGA file in QSPI or
    SDMMC.  For SDMMC with FAT support, it also defines the name of the FPGA
    file on the FAT parition.
  
    The FPGA file should be an RBF file from Quartus, wrapped with the mkimage
    program to create the appropriate header.

    To configure the MPL to program the FPGA:

    (1) Update the 'mpl_config.h' settings:

        - CONFIG_MPL_FPGA_LOAD must be set to 1
        - CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR or
          CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR
          must be set correctly for the boot source.

        - CONFIG_MPL_FAT_LOAD_FPGA_NAME must be set to the name of the FPGA
          file if the file is on the FAT partiion.

    (2) If you changed CONFIG_MPL_FPGA_LOAD, rebuild the MPL.

    (3) Obtain the FPGA configuration file in the raw format, with the '.rbf'
        file extension.  This example assumes the file is name 'fpga.rbf'.

    (4) Process the file with the mkimage tool to create the FPGA image file.

        >>> mkimage -A arm -O u-boot -T firmware -a 0xA00000 -e 0 -name fpga-rbf -d fpga.rbf -C none fpga.img

        The load address (-a) is where the MPL will first copy the entire RBF
        image from flash to RAM before programming the FPGA.

    (5) Program the RBF file to flash media.

        QSPI:

        The FPGA configuration location in QSPI is set to 0x800000.
        (see CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR in mpl_config.h)

        To program the FPGA image file into the QSPI flash:

        >>> quartus_hps -c USB-BlasterII -o P -a 0x800000 fpga.img

        SDMMC:

        For SDMMC, the address is set to offset 0x100000 of the A2 partition.
        (see CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR in mpl_config.h). If the
        secondary image is larger than 768 KiB (256 KiB for preloader + 768 KiB
        for the secondary image), this value may need to be increased to a
        larger value.

        To program the RBF file into SDMMC on the raw partition:

        >>> sudo dd if=fpga.img of=/dev/sdb3 bs=64k seek=16

        In this example since the address was set to 0x100000 (1MB) and in
        command bs=64k, this corresponds to setting seek to 16 in the dd
        command.

        If the MPL is configured to load the FPGA from FAT, you may copy the
        fpga.rbf file to the FAT partition of the SD card rather than copying
        it to the raw partition.

    (6) Set the MSEL settings on the DevKit (SW3) according to the FPGA
        configuration compression.


********************************************************************
Updating the MPL for Arria 10, based on the FPGA handoff information:
********************************************************************

    Arria 10 SoC is different from Cyclone V SoC and Arria V SoC in the way
    that information is handed from the FPGA build to the MPL.  For Cyclone V
    and Arria V, the FPGA build creates handoff information in the form of
    C code and the MPL compiles this handoff information into the code.

    Arria 10 uses a special boot device tree, which is generated from the
    FPGA's hps.xml file.  The u-boot and UEFI loaders parse this device tree
    at run-time.  MPL does not have a device tree parser.  Instead, you must
    either manually copy the information from the FPGA hps.xml file or use
    the provided scripts to extract the information from the FPGA files and regenerate
    the C files, and then rebuild the MPL binary. 
    
    For Arria 10, the MPL needs to be updated after building the FPGA image
    to reflect the values exported from the Clock Manager and the
    System Manager.  This section is not required for Arria V or Cyclone V.

    Before compiling the MPL, two files must be modified to update
    the values from the hardware design.
 
    - clk_data.c
    - system_manager_pinmux_a10.c

    There are two ways to do this, manually or using scripts.

    To generate these from a script, you need a valid u-boot dtb or dts file
    generated from the BSP Generator.

    (see http://www.alterawiki.com/wiki/SoCEDSGettingStarted#Generating_and_Compiling_Arria_10_Bootloader).

    You can then use scripts to recreate the C files based on the device tree file:

    ./gen_clk.sh /project/a10_project/software/bootloader/devicetree.dtb > core/clk_data.c
    ./gen_pinmux.sh /project/a10_soc_project/software/bootloader/devicetree.dtb > core/system_manager_pinmux_a10.c

    To manually update these files, you must first find the FPGA's $(HANDOFF_DIR)/hps.xml file,
    and then update these two files with data from the XML file:

    (1) core/clk_data.c
 
        Update the Data Structures with the values from hps.xml to 
        program the Clock Manager.

           C Source File                    hps.XML
          ______________                   ________
      psrc->clk_freq_of_eosc1          eosc1_clk_hz
      psrc->clk_freq_of_f2h_free       f2h_free_clk_hz
      psrc->clk_freq_of_cb_intosc_ls   b_intosc_ls_clk_hz

      pMgr->mainpll.vco0_psrc          i_clk_mgr_mainpllgrp.vco0.psrc
      pMgr->mainpll.vco1_denom         i_clk_mgr_mainpllgrp.vco1.denom
      pMgr->mainpll.vco1_numer         i_clk_mgr_mainpllgrp.vco1.numer
      pMgr->mainpll.mpuclk_cnt         i_clk_mgr_mainpllgrp.mpuclk.cnt
      pMgr->mainpll.mpuclk_src         i_clk_mgr_mainpllgrp.mpuclk.src
      pMgr->mainpll.nocclk_cnt         i_clk_mgr_mainpllgrp.nocclk.cnt
      pMgr->mainpll.nocclk_src         i_clk_mgr_mainpllgrp.nocclk.src
      pMgr->mainpll.cntr2clk_cnt       i_clk_mgr_mainpllgrp.cntr2clk.cnt
      pMgr->mainpll.cntr3clk_cnt       i_clk_mgr_mainpllgrp.cntr3clk.cnt
      pMgr->mainpll.cntr4clk_cnt       i_clk_mgr_mainpllgrp.cntr4clk.cnt
      pMgr->mainpll.cntr5clk_cnt       i_clk_mgr_mainpllgrp.cntr5clk.cnt
      pMgr->mainpll.cntr6clk_cnt       i_clk_mgr_mainpllgrp.cntr6clk.cnt
      pMgr->mainpll.cntr7clk_cnt       i_clk_mgr_mainpllgrp.cntr7clk.cnt
      pMgr->mainpll.cntr7clk_src       i_clk_mgr_mainpllgrp.cntr7clk.src
      pMgr->mainpll.cntr8clk_cnt       i_clk_mgr_mainpllgrp.cntr8clk.cnt
      pMgr->mainpll.cntr9clk_cnt       i_clk_mgr_mainpllgrp.cntr9clk.cnt
      pMgr->mainpll.cntr9clk_src       i_clk_mgr_mainpllgrp.cntr9clk.src
      pMgr->mainpll.cntr15clk_cnt      i_clk_mgr_mainpllgrp.cntr15clk.cnt
      pMgr->mainpll.nocdiv_l4mainclk   i_clk_mgr_mainpllgrp.nocdiv.l4mainclk
      pMgr->mainpll.nocdiv_l4mpclk     i_clk_mgr_mainpllgrp.nocdiv.l4mpclk
      pMgr->mainpll.nocdiv_l4spclk     i_clk_mgr_mainpllgrp.nocdiv.l4spclk
      pMgr->mainpll.nocdiv_csatclk     i_clk_mgr_mainpllgrp.nocdiv.csatclk
      pMgr->mainpll.nocdiv_cstraceclk  i_clk_mgr_mainpllgrp.nocdiv.cstraceclk
      pMgr->mainpll.nocdiv_cspdbgclk   i_clk_mgr_mainpllgrp.nocdiv.cspdbgclk

      pMgr->perpll.vco0_psrc           i_clk_mgr_perpllgrp.vco0.psrc
      pMgr->perpll.vco1_denom          i_clk_mgr_perpllgrp.vco1.denom
      pMgr->perpll.vco1_numer          i_clk_mgr_perpllgrp.vco1.numer
      pMgr->perpll.cntr2clk_cnt        i_clk_mgr_perpllgrp.cntr2clk.cnt
      pMgr->perpll.cntr2clk_src        i_clk_mgr_perpllgrp.cntr2clk.src
      pMgr->perpll.cntr3clk_cnt        i_clk_mgr_perpllgrp.cntr3clk.cnt
      pMgr->perpll.cntr3clk_src        i_clk_mgr_perpllgrp.cntr2clk.src
      pMgr->perpll.cntr4clk_cnt        i_clk_mgr_perpllgrp.cntr4clk.cnt
      pMgr->perpll.cntr4clk_src        i_clk_mgr_perpllgrp.cntr4clk.src
      pMgr->perpll.cntr5clk_cnt        i_clk_mgr_perpllgrp.cntr5clk.cnt
      pMgr->perpll.cntr5clk_src        i_clk_mgr_perpllgrp.cntr5clk.src
      pMgr->perpll.cntr6clk_cnt        i_clk_mgr_perpllgrp.cntr6clk.cnt
      pMgr->perpll.cntr6clk_src        i_clk_mgr_perpllgrp.cntr6clk.src
      pMgr->perpll.cntr7clk_cnt        i_clk_mgr_perpllgrp.cntr7clk.cnt
      pMgr->perpll.cntr8clk_cnt        i_clk_mgr_perpllgrp.cntr8clk.cnt
      pMgr->perpll.cntr8clk_src        i_clk_mgr_perpllgrp.cntr8clk.src
      pMgr->perpll.cntr9clk_cnt        i_clk_mgr_perpllgrp.cntr9clk.cnt
      pMgr->perpll.emacctl_emac0sel    i_clk_mgr_perpllgrp.emacctl.emac0sel
      pMgr->perpll.emacctl_emac1sel    i_clk_mgr_perpllgrp.emacctl.emac1sel
      pMgr->perpll.emacctl_emac2sel    i_clk_mgr_perpllgrp.emacctl.emac2sel
      pMgr->perpll.gpiodiv_gpiodbclk   i_clk_mgr_perpllgrp.gpiodiv.gpiodbclk

      pMgr->alteragrp.nocclk =         i_clk_mgr_alteragrp.nocclk.pericnt << 16 |
                                       i_clk_mgr_alteragrp.nocclk.maincnt
      pMgr->alteragrp.mpuclk =         i_clk_mgr_alteragrp.mpuclk.pericnt << 16 |
                                       i_clk_mgr_alteragrp.mpuclk.maincnt

    (2) core/system_manager_pinmux_a10.c

        Update the Data Structures with the values from hps.xml to 
        program the System Manager.

           C Source File                    hps.XML
          ______________                   ________

      addr = ALT_PINMUX_SHARED_3V_IO_GRP_ADDR
      addr[0x00/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_1.sel
      addr[0x04/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_2.sel
      addr[0x08/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_3.sel
      addr[0x0C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_4.sel
      addr[0x10/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_5.sel
      addr[0x14/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_6.sel
      addr[0x18/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_7.sel
      addr[0x1C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_8.sel
      addr[0x20/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_9.sel
      addr[0x24/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_10.sel
      addr[0x28/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_11.sel
      addr[0x2C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q1_12.sel

      addr[0x30/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_1.sel
      addr[0x34/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_2.sel
      addr[0x38/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_3.sel
      addr[0x3C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_4.sel
      addr[0x40/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_5.sel
      addr[0x44/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_6.sel
      addr[0x48/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_7.sel
      addr[0x4C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_8.sel
      addr[0x50/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_9.sel
      addr[0x54/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_10.sel
      addr[0x58/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_11.sel
      addr[0x5C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q2_12.sel

      addr[0x60/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_1.sel
      addr[0x64/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_2.sel
      addr[0x68/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_3.sel
      addr[0x6C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_4.sel
      addr[0x70/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_5.sel
      addr[0x74/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_6.sel
      addr[0x78/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_7.sel
      addr[0x7C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_8.sel
      addr[0x80/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_9.sel
      addr[0x84/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_10.sel
      addr[0x88/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_11.sel
      addr[0x8C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q3_12.sel

      addr[0x90/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_1.sel
      addr[0x94/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_2.sel
      addr[0x98/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_3.sel
      addr[0x9C/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_4.sel
      addr[0xA0/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_5.sel
      addr[0xA4/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_6.sel
      addr[0xA8/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_7.sel
      addr[0xAC/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_8.sel
      addr[0xB0/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_9.sel
      addr[0xB4/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_10.sel
      addr[0xB8/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_11.sel
      addr[0xBC/4]                     i_io48_pin_mux_shared_3v_io_grp.pinmux_shared_io_q4_12.sel


      addr = ALT_PINMUX_DCTD_IO_GRP_ADDR;   /* 0xffd07200 */
      addr[0x00/4]                     0
      addr[0x04/4]                     0
      addr[0x08/4]                     0
      addr[0x0C/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_4.sel
      addr[0x10/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_5.sel
      addr[0x14/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_6.sel
      addr[0x18/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_7.sel
      addr[0x1C/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_8.sel
      addr[0x20/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_9.sel
      addr[0x24/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_10.sel
      addr[0x28/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_11.sel
      addr[0x2C/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_12.sel
      addr[0x30/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_13.sel
      addr[0x34/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_14.sel
      addr[0x38/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_15.sel
      addr[0x3C/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_16.sel
      addr[0x40/4]                     i_io48_pin_mux_dedicated_io_grp.pinmux_dedicated_io_17.sel


      uint32_t *addr = ALT_PINMUX_DCTD_IO_CFG_BANK_ADDR; /* 0xffd07300 */
      addr[0x00/4]                     i_io48_pin_mux.configuration_dedicated_io_bank
                                       (voltage_sel_clkrst_io << 8 | \
                                        voltage_sel_peri_io   << 0)


      addr[0x04/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_1
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pu_drv_strg   << 8  |\
                                        pd_slw_rt     << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x08/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_2
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pu_drv_strg   << 8  |\
                                        pd_slw_rt     << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x0C/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_3
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x10/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_4
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x14/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_5
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x18/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_6
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x1C/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_7
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x20/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_8
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x24/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_9
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x28/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_10
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x2C/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_11
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x30/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_12
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x34/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_13
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x38/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_14
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x3C/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_15

                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x40/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_16
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0)

      addr[0x44/4]                     i_io48_pin_mux_dedicated_io_grp.configuration_dedicated_io_17
                                       (rtrim         << 19 |\
                                        input_buf_en  << 17 |\
                                        wk_pu_en      << 16 |\
                                        pu_slw_rt     << 13 |\
                                        pd_slw_rt     << 8  |\
                                        pu_drv_strg   << 5  |\
                                        pd_drv_strg   << 0

     
      addr = ALT_PINMUX_SHARED_3V_IO_GRP_ADDR; /* 0xffd07400 */
      addr[0x00/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_rgmii0_usefpga.sel
      addr[0x04/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_rgmii1_usefpga.sel
      addr[0x08/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_rgmii2_usefpga.sel
      addr[0x0C/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_i2c0_usefpga.sel
      addr[0x10/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_i2c1_usefpga.sel
      addr[0x14/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_i2cemac0_usefpga.sel
      addr[0x18/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_i2cemac1_usefpga.sel
      addr[0x1C/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_i2cemac2_usefpga.sel
      addr[0x20/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_nand_usefpga.sel
      addr[0x24/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_qspi_usefpga.sel
      addr[0x28/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_sdmmc_usefpga.sel
      addr[0x2C/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_spim0_usefpga.sel
      addr[0x30/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_spim1_usefpga.sel
      addr[0x34/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_spis0_usefpga.sel
      addr[0x38/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_spis1_usefpga.sel
      addr[0x3C/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_uart0_usefpga.sel
      addr[0x40/4]                     i_io48_pin_mux_fpga_interface_grp.pinmux_uart1_usefpga.sel

      (3) After these files are modified, recompile the MPL

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
