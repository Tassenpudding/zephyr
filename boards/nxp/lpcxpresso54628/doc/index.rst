.. zephyr:board:: lpcxpresso54628

Overview
********

The LPCXpresso54628 development board (OM13098) uses an NXP LPC54628 MCU based
on an Arm Cortex-M4F core.

Hardware
********

- LPC54628 M4F running at up to 220 MHz
- Memory

  - 512KB of on-chip flash memory
  - 200KB of on-chip SRAM (160KB main + 32KB SRAMX + 8KB USB)
- On-board high-speed USB based debug probe (LPC-Link2) with CMSIS-DAP and
  J-Link protocol support
- External debug probe option
- Three user LEDs, target reset, ISP and interrupt/user buttons
- Expansion options based on Arduino UNO and PMOD, plus additional expansion
  port pins
- 16MB SPIFI serial flash, 128Mb SDRAM
- 10/100 Mbps Ethernet, full-speed and high-speed USB, CAN

More information can be found here:

- `LPC546xx SoC Website`_
- `LPC546xx Datasheet`_
- `LPCXpresso54628 Board Website`_
- `LPCXpresso54628 Board User Manual`_

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

The IOCON controller can be used to configure the LPC54628 pins.

+---------+-----------------+----------------------------+
| Name    | Function        | Usage                      |
+=========+=================+============================+
| PIO0_29 | UART            | USART RX (VCOM)            |
+---------+-----------------+----------------------------+
| PIO0_30 | UART            | USART TX (VCOM)            |
+---------+-----------------+----------------------------+
| PIO3_14 | GPIO            | User LED1                  |
+---------+-----------------+----------------------------+
| PIO3_3  | GPIO            | User LED2                  |
+---------+-----------------+----------------------------+
| PIO2_2  | GPIO            | User LED3                  |
+---------+-----------------+----------------------------+
| PIO1_1  | GPIO            | User button SW5            |
+---------+-----------------+----------------------------+

.. note::

   The push buttons labelled SW2, SW3 and SW4 double as the ISP2/ISP1/ISP0
   boot-mode selectors and are wired to PIO0_6, PIO0_5 and PIO0_4. On this
   board those pins are used as the EMC data bus (EMC_D4/EMC_D3/EMC_D2) for the
   on-board SDRAM, so they cannot be used as GPIO inputs while the external
   memory controller is enabled. Only SW5 (PIO1_1) is available as a general
   user button.

Audio codec and the shared I2C bus
==================================

The on-board WM8904 audio codec is controlled over FlexComm2 I2C (PIO3_23 SDA /
PIO3_24 SCL), a bus it shares with the FXOS8700-compatible accelerometer and the
Arduino header. Its audio master clock, MCLK, is driven from the SoC on PIO3_11.

PIO3_11 (MCLK, ball B2) sits next to PIO3_23 (I2C SDA, ball C2), and the
24.576 MHz MCLK couples onto the SDA line. Once MCLK is running, I2C transfers to
*any* device on FlexComm2 (codec and accelerometer alike) can intermittently lose
arbitration. The effect is dominated by bench wiring (a jack-to-jack audio
loopback cable and jumper leads roughly quadruple the error rate), so on a
cleanly cabled board it is usually within margin, but it is present.

Because the codec's I2C control interface does not itself need MCLK -- only the
codec's internal clocking and the I2S audio stream do -- applications that drive
audio on this board should configure the codec with MCLK kept off PIO3_11, then
route MCLK to the pin before starting the I2S stream. The MCLK clock source stays
running throughout (so the codec driver still reads the correct MCLK rate); only
the pin mux is gated:

.. code-block:: c

   #include <soc.h>

   /* FUNC1 = MCLK, FUNC0 = GPIO (no clock on the pin). */
   #define MCLK_PIN_FUNC(f) \
      (IOCON->PIO[3][11] = (IOCON->PIO[3][11] & ~IOCON_PIO_FUNC_MASK) | IOCON_PIO_FUNC(f))

   MCLK_PIN_FUNC(0);                    /* MCLK off the pin: clean I2C */
   audio_codec_configure(codec, &cfg);
   MCLK_PIN_FUNC(1);                    /* MCLK back to the codec for streaming */
   /* ... configure and start the I2S stream ... */

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Configuring a Debug Probe
=========================

LinkServer is the default runner for this board. A debug probe is used for both
flashing and debugging the board. This board is configured by default to use the
on-board :ref:`lpclink2-cmsis-onboard-debug-probe` in the CMSIS-DAP mode. To use
this probe with Zephyr, you need to install the :ref:`linkserver-debug-host-tools`
and make sure they are in your search path. Refer to the detailed overview about
:ref:`application_debugging` for additional information.

Configuring a Console
=====================

Connect a USB cable from your PC to the LPC-Link2 USB connector, and use the
serial terminal of your choice (minicom, putty, etc.) with the following
settings:

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

Flashing
========

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: lpcxpresso54628
   :goals: flash

Open a serial terminal, reset the board (press the RESET button), and you
should see the following message in the terminal:

.. code-block:: console

   *** Booting Zephyr OS build v4.4.0 ***
   Hello World! lpcxpresso54628/lpc54628

Debugging
=========

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: lpcxpresso54628
   :goals: debug

.. include:: ../../common/board-footer.rst.inc

.. _LPC546xx SoC Website:
   https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc-cortex-m4-mcus/lpc546xx-arm-cortex-m4-based-microcontroller-family:LPC546XX

.. _LPC546xx Datasheet:
   https://www.nxp.com/docs/en/data-sheet/LPC546XX.pdf

.. _LPCXpresso54628 Board Website:
   https://www.nxp.com/products/developer-resources/software-development-tools/developer-resources-/lpcxpresso-boards/lpcxpresso54628-development-board:OM13098

.. _LPCXpresso54628 Board User Manual:
   https://www.nxp.com/webapp/Download?colCode=UM11035
