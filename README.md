# pic-uart
Software Transparent UART Emulator for PIC24 with PMP

This project grew out of an interest in having a breadboardable UART capable of working with older 8/16-bit CPUs.  While
a handful of UARTs are still made, they typically do not come in DIP format.  Of the few that do, many are no longer
manufactured and thus increasingly difficult to find.  Or in at least one case (65C51), the chip has (known) bugs
that reduce its functionality.

The PIC24 was chosen for several reasons.  It is inexpensive, available in DIP format (breadboardable), and perhaps most
importantly, it has a Parallel Master Port (PMP) on it.  This allows it to be directly interfaced to a typical 8-bit
microprocessor bus with a minimum of external logic.  When operated in slave mode, the PMP on the PIC24FJ32GA002
(which is the target I initially selected to develop on) supports two address lines allowing four distinct memory
addresses to be accessed (both reading and writing).

One caveat of using the PIC24 is that it requirees operation with a 3.3V power supply.  Many of the older CPUs out there
require a 5V supply.  This is a regrettable result of the evolution of technology over the past three plus decades.
Fortunately, there are technical solutions which are not difficult to implement.  I use a 74LVX4245 level converting
transceiver mounted on a SOIC-to-DIP adapter myself, though other similar solutions should work equally well.

This version of the code presently supports a variant of the MC6850 UART.  Exact duplication of functionality is not
possible for several reasons.  First, the PIC does not support a 7-bit mode (though it can probably be emulated with
8-bit mode and software parity).  Second, rather than forcing the user to provide a separate oscillator from which
the serial timing is derived, the PICs on-board oscillator is used for this purpose.  This allows the user to select
virtually any standard baud rate from 300 baud though 921600 baud through an extension.  See the source code for further
details.  The default baud rate is 9600 baud, though that may be easily changed.

This source code represents a work in progress and has not yet been properly tested beyond error-free compilation.  I
plan to test it over the next week or two and post any fixes that I find.  Time allowing, I will also create a
6551 mode as well and allow the user to select from among the two with a compile-time flag.

I also hope to use this project as a starting point for an 8-bit CPU interface to SPI and I2C which I'll call
PIC-SPI and PIC-I2C respectively.  These should provide convenient interfaces to these two populare serial protocols
for those who wish to connect modern devices to their retro-CPUs.

Bug fixes, comments, and suggestions are welcome.

See http://forum.6502.org/viewtopic.php?f=4&t=3488 for more details and ongoing discussion related to this project.
