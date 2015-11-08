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

This version of the code presently supports a mostly compatible derivative of the MC6850 ACIA.  Exact duplication of
functionality is not possible for several reasons.  First, the PIC does not support a 7-bit mode (though it can
probably be emulated with 8-bit mode and software parity).  Second, rather than forcing the user to provide a separate
oscillator from which the serial timing is derived, the PICs on-board oscillator is used for this purpose.  This
allows the user to select virtually any standard baud rate from 300 baud though 921600 baud through an extension.  See
the source code for further details.  The default baud rate is 9600/38400 baud (depending on the 6850 mode register
settings), though that may be easily changed by writing to the added baud rate generator register.

While this software has undergone limited testing and appears to work, it should at best be considered beta quality
at the present time.  So far polled-I/O has been fairly extensively tested and found to work, but interrupt-driven
I/O remains untested.  As far as operating limitations go, the system is able to successfully transmit and receive
at all baud rates from 300 through 921600 baud with no crystal and operating at room temperature only.  CPU bus
speeds up to 4 MHz (TX FIFO enabled) and 5 MHz (TX FIFO disabled) are confirmed to work using a 65C02 CPU.  Wait
states do not appear to be necessary at these bus speeds, though most testing was performed with a single wait
state inserted.

Bug fixes, comments, and suggestions are welcome.

See http://forum.6502.org/viewtopic.php?f=4&t=3488 for more details and ongoing discussion related to this project.
