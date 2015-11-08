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
details.  The default baud rate is 9600/38400 baud (depending on the 6850 mode register settings), though that may be
easily changed by writing to the added baud rate generator register.

While this software has undergone limited testing and appears to be functional within some limits, it should at best
be considered beta quality at the present time.  Additional testing is necessary in order to determine the precise
operating limitations of the code so that other systems may be designed around these limitations.

The testing that has been performed so far consists of polled input and output at baud rates from 1200 through
38400 baud.  Bus speeds of up to 3.68 MHz were tested using a 65C02 CPU with one wait state inserted.  I expect
higher bus speeds can be easily reached if some usage guidelines are followed, and I expect that it will work at
lower speeds without the need for any wait states to be inserted on the bus.  Additional testing is scheduled and
any problems found will be addressed as quickly as is practicable.

Bug fixes, comments, and suggestions are welcome.

See http://forum.6502.org/viewtopic.php?f=4&t=3488 for more details and ongoing discussion related to this project.
