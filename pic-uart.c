/*
 * File:   pic-uart.c
 *
 * Copyright 2015 by Jason Burke
 *
 * This file is part of PIC-UART
 *
 * PIC-UART is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PIC-UART is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pic-uart.c.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * 6551/6850 ACIA Emulation on Microchip PIC24FJ32GA002 MCU
 *
 * PIN NAME     DESCRIPTION
 * ---------------------------------------------------------------------
 *  1  MCLRn    Reserved for programming/debugging
 *  2  RA0      (AN0) DCD input
 *  3  RA1      (AN1) CTS input
 *  4  RB0      (RP0) PGD (available I/O when development complete)
 *  5  RB1      (RP1) PGC (available I/O when development complete)
 *  6  RB2      (RP2) TX Data
 *  7  RB3      (RP3) RX Data
 *  8  VSS      Connect to ground
 *  9  RA2      IRQ output (open drain)
 * 10  RA3      (PMA0) A0
 * 11  RB4      (RP4) RTS output
 * 12  RA4      (PMA1) A1
 * 13  VDD      Connect to +3.3V
 * 14  RB5      *(RP5, PMD7) D7 (parallel I/O)
 * 15  RB6      *(RP6, PMD6) D6 (parallel I/O)
 * 16  RB7      *(RP7, PMD5) D5 (parallel I/O)
 * 17  RB8      *(RP8, PMD4) D4 (parallel I/O)
 * 18  RB9      *(RP9, PMD3) D3 (parallel I/O)
 * 19  DISVR    Connect to ground to enable on-board regulator
 * 20  VCAP     10uF or higher low-ESR capacitor to ground
 * 21  RB10     *(RP10, PMD2) D2 (parallel I/O)
 * 22  RB11     *(RP11, PMD1) D1 (parellel I/O)
 * 23  RB12     (RP12, PMD0) D0 (parallel I/O)
 * 24  RB13     (RP13. PMRD) RD
 * 25  RB14     (RP14, PMWR) WR
 * 26  RB15     (RP15, PMCS) CS
 * 27  AVSS     Connect to ground
 * 28  AVDD     Connect to +3.3V
 * (*) designates pins which are 5V tolerant
 *
 * 6850 Mode
 * ---------
 *
 * Addr Bit Dir Description             Dir Description
 * -------------------------------------------------------------------------
 *    0   0 W   Counter Divide Sel 1    R   Receive Data Register Full
 *    0   1 W   Counter Divide Sel 2    R   Transmit Data Register Empty
 *    0   2 W   Word Select 1           R   Data Carrier Detect
 *    0   3 W   Word Select 2           R   Clear to Send
 *    0   4 W   Word Select 3           R   Framing Error
 *    0   5 W   Transmit Control 1      R   Receiver Overrun
 *    0   6 W   Transmit Control 2      R   Parity Error
 *    0   7 W   Recv Interrupt Enable   R   Interrupt Request
 *    1   0 W   TDR Bit 0               R   RDR Bit 0
 *    1   1 W   TDR Bit 1               R   RDR Bit 1
 *    1   2 W   TDR Bit 2               R   RDR Bit 2
 *    1   3 W   TDR Bit 3               R   RDR Bit 3
 *    1   4 W   TDR Bit 4               R   RDR Bit 4
 *    1   5 W   TDR Bit 5               R   RDR Bit 5
 *    1   6 W   TDR Bit 6               R   RDR Bit 6
 *    1   7 W   TDR Bit 7               R   RDR Bit 7
 *    2 7-0 W   Baud Rate Gen LSB       R   Baud Rate Gen LSB
 *    3 7-0 W   Baud Rate Gen MSB       R   Baud Rate Gen MSB
 * Note: Original 6850 used external clock to select baud rate with limited
 *       divisor values of 1, 16, and 64.
 *
 * Counter Divide Select Bits:
 * B1   B0  Description
 * -------------------------------------------------------------------------
 * 0    0   BRG in divide-by-4 mode (U1MODEbits.BRGH = 1)
 * 0    1   BRG in divide-by-16 mode (U1MODEbits.BRGH = 0)
 * 1    0   BRG in divide-by-16 mode (U1MODEbits.BRGH = 0)
 * 1    1   RESET
 *
 * Word Select Bits (B2 is don't care because 7-bit mode not supported by PIC):
 * B2   B1  B0  Description
 * -------------------------------------------------------------------------
 * X    0   0   8 bits + no parity + 2 stop bits
 * X    0   1   8 bits + no parity + 1 stop bit
 * X    1   0   8 bits + Even parity + 1 stop bit
 * X    1   1   8 bits + Odd parity + 1 stop bit
 *
 * Transmit Control Bits:
 * B1   B0  Description
 * -------------------------------------------------------------------------
 * 0    0   RTSn = low, transmitting interrupt disabled
 * 0    1   RTSn = low, transmitting interrupt enabled
 * 1    0   RTSn = high, transmitting interrupt disabled
 * 1    1   RTSn = low, transmits a single break, transmitting intr disabled
 *
 * Baud Rate Table (for baud rate generator word):
 * -----------------------------------------------
 *
 * Baud Rate    BRG-16  %err    BRG-4   %err
 * -----------------------------------------------------
 *       300      3071  0.00    12287   0.00
 *       600      1535  0.00     6143   0.00
 *      1200       767  0.00     3071   0.00
 *      2400       383  0.00     1535   0.00
 *      4800       191  0.00      767   0.00
 *      9600        95  0.00      383   0.00
 *     14400        63  0.00      255   0.00
 *     19200        47  0.00      191   0.00
 *     38400        23  0.00       95   0.00
 *     57600        15  0.00       63   0.00
 *     76800        23  0.00       47   0.00
 *    115200         7  0.00       31   0.00
 *    230400         3  0.00       15   0.00
 *    460800         1  0.00        7   0.00
 *    921600         0  0.00        3   0.00
 *
 * TODO:
 *     TESTING:  Test interrupt operation
 *     PERFORMANCE:  Speedup main loop; speedup critical section
 *     FUNCTIONALITY:  Add software parity (?), 6551 mode (?)
 */

// Chip Configuration
#pragma config POSCMOD = NONE       // primary oscillator disabled
#pragma config I2C1SEL = PRI        // use default SCL1/SDA1 pins
#pragma config IOL1WAY = OFF        // IOLOCK may be changed via unlock sequence
#pragma config OSCIOFNC = ON        // CLKO functions as port I/O
#pragma config FCKSM = CSDCMD       // clock switching & fail-safe clk mon disabled
#pragma config FNOSC = FRCPLL       // fast RC oscillator with 4x PLL selected
#pragma config SOSCSEL = SOSC       // default secondary oscillator
#pragma config WUTSEL = LEG         // legacy wake-up timer
#pragma config IESO = OFF           // two speed start-up disabled
#pragma config WDTPS = PS256        // watchdog timer postscaler
#pragma config FWPSA = PR128        // watchdog timer prescaler
#pragma config WINDIS = ON          // standard watchdog timer selected (no window)
#pragma config FWDTEN = OFF         // watchdog timer disabled
#pragma config ICS = PGx1           // Emu EMUC1/EMUD1 pins shared with PGC1/PGD1
#pragma config COE = OFF            // reset into operational mode
#pragma config BKBUG = OFF          // device resets into operational mode
#pragma config GWRP = OFF           // writes to program memory are allowed
#pragma config GCP = OFF            // code protection is disabled
#pragma config JTAGEN = OFF         // JTAG port is disabled

#ifndef FCY
#define FCY     14745600            // FCY = FOSC / 2
#endif

// Defining USE_TX_FIFO will increase output speeds at the cost of reducing the
// maximum bus write speeds.  Use it with interrupt-driven I/O on the cpu side.
#define USE_TX_FIFO

// Standard includes
#include <xc.h>
#include <libpic30.h>               // __delay() macros
#include <stdint.h>

// Program constants
const uint16_t DEF_DIVISOR = 95;    // Default baud rate is 9600/38400

// Typedefs
typedef struct {
    uint8_t ctrl;                   // 6850 control register (write only)
    uint8_t status;                 // 6850 status register (read only)
    uint8_t rdr;                    // 6850 receive data register (read only)
    uint16_t divisor;               // baud rate selection (read/write)
} regs_6850_t;

// Program globals
volatile regs_6850_t regs_6850;

////////////////////////////////////
// Configure hardware peripherals //
////////////////////////////////////
static void initHardware(void)
{
    // Configure PIC Oscillator
    // RC oscillator runs at 8 MHz, but we tune it to approx. 7.3728 MHz
    // If PLL is enabled, output frequency is multiplied by four
    CLKDIVbits.RCDIV = 0b000;               // Disable postscaler

    // Tweak the oscillator frequency to approx. 7.3728 MHz
    OSCTUNbits.TUN = 0b101110;

    // Wait for PLL to lock
    while (OSCCONbits.LOCK == 0)
        ;

    // Interrupt configuration
    INTCON1bits.NSTDIS = 1;         // disable nested interrupts

    // Configure pins as needed by application
    OSCCONL = 0x46;                 // unlock sequence 1
    OSCCONL = 0x57;                 // unlock sequence 2
    OSCCONbits.IOLOCK = 0;          // unlock RPINRx/RPORx registers
    RPOR1bits.RP2R = 3;             // map UART1 TX to pin 6 (RP2)
    RPINR18bits.U1RXR = 3;          // map UART1 RX to pin 7 (RP3)
    OSCCONL = 0x46;                 // unlock sequence 1
    OSCCONL = 0x57;                 // unlock sequence 2
    OSCCONbits.IOLOCK = 1;          // lock RPINRx/RPORx registers

    // Configure the PMP (Parallel Master Port)
    PMCONbits.PSIDL = 0;            // run in idle mode
    PMCONbits.ADRMUX = 0b00;        // address and data separate
    PMCONbits.PTBEEN = 0;           // byte enable port DISABLED
    PMCONbits.PTWREN = 1;           // PMWR port enabled (on in slave mode)
    PMCONbits.PTRDEN = 1;           // PMRD port enabled (on in slave mode)
    PMCONbits.CSF = 0b10;           // PMCS1 functions as chip select
    PMCONbits.CS1P = 0;             // set chip select polarity to active low
    PMCONbits.WRSP = 0;             // set PMWR strobe polarity to active low
    PMCONbits.RDSP = 0;             // set PMRD strobe polarity to active low
    PMMODEbits.IRQM = 0b01;         // PMP intrs generated at end of R/W cycle
    PMMODEbits.INCM = 0b00;         // auto-{inc,dec}rement disabled
    PMMODEbits.MODE16 = 0;          // 8-bit mode
    PMMODEbits.MODE = 0b01;         // select enhanced PSP mode
    PMMODEbits.WAITB = 0b00;        // wait state config bits
    PMMODEbits.WAITE = 0b00;        // read to byte enable wait state config
    PMAENbits.PTEN14 = 1;           // PMCS1 strobe enable bit
    PMAENbits.PTEN1 = 1;            // PMA1 strobe enable bit
    PMAENbits.PTEN0 = 1;            // PMA0 strobe enable bit
    IFS2bits.PMPIF = 0;             // clear PMP interrupt flag
    IPC11bits.PMPIP = 6;            // set PMP interrupt priority level
    IEC2bits.PMPIE = 1;             // enable PMP interrupts
    PMCONbits.PMPEN = 1;            // enable PMP

    // Configure UART1
    // Remaining to be configured by the application code:
    // - baud rate (U1MODEbits.BRGH, U1BRG)
    // - # data bits, parity, # stop bits
    // - enable UART
    U1MODEbits.USIDL = 0;           // run in idle mode
    U1MODEbits.IREN = 0;            // disable IrDA encoder/decoder
    U1MODEbits.RTSMD = 0;           // U1RTS pin in flow control mode
    U1MODEbits.UEN = 0b00;          // U1TX & U1RX pins are enabled & used
    U1MODEbits.WAKE = 0;            // wake up on start bit detect disabled
    U1MODEbits.LPBACK = 0;          // loopback mode disabled
    U1MODEbits.ABAUD = 0;           // auto-baud detection disabled
    U1MODEbits.RXINV = 0;           // receive polarity inversion bit (normal)
    U1MODEbits.BRGH = 0;            // BRG generates 16 clocks per bit period
    U1STAbits.UTXISEL0 = 0;         // intr generated when any char xfered
    U1STAbits.UTXISEL1 = 0;         // to the transmit shift register
    U1STAbits.UTXINV = 0;           // transmit polarity inversion bit
    U1STAbits.URXISEL = 0b00;       // rx intr flag set when a char received
    IFS0bits.U1RXIF = 0;            // clear U1RX interrupt flag
    IPC2bits.U1RXIP = 6;            // set U1RX interrupr priority level
    IEC0bits.U1RXIE = 1;            // enable U1RX interrupts
    U1BRG = DEF_DIVISOR;            // configure baud rate
    U1MODEbits.UARTEN = 0;          // start with UART disabled

    // Configure analog inputs
    AD1PCFG = 0b1001111000111111;   // disable all A/D ports

    // Configure PORTA
    // Port RA2 is the IRQ output
    LATAbits.LATA2 = 1;             // set initial value of RA2 output high
    ODCAbits.ODA2 = 1;              // config port RA2 for open-drain output
    TRISA = 0b11011;                // config RA2 as output, others as inputs

    // Configure PORTB
    // Port RB4 is the RTS output
    LATBbits.LATB4 = 1;             // set initial value of RB4 output high
    TRISB = 0b1111111111101111;     // config RB4 as output, others as inputs
}

///////////////////////////////////////////////////////////////
// UART1 RX ISR
///////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv, shadow)) _U1RXInterrupt(void)
{
    // A new character is available.  If the receive data register is not
    // already full, copy the new character to it.
    if ((regs_6850.status&0x1) == 0) {
        regs_6850.rdr = U1RXREG;
    }

    // Update the following bits in the status register:
    // b0 - set (receive data available)
    // b4 - framing error (from uart status)
    // b5 - overrun error (from uart status)
    // b6 - parity error (from uart status)
    // b7 - irq state
    regs_6850.status &= 0b00001110;
    regs_6850.status |= 1 |
                        (U1STAbits.FERR<<4) | (U1STAbits.OERR<<5) |
                        (U1STAbits.PERR<<6) | (regs_6850.ctrl&0x80);

    // Clear UART1 RX interrupt flag
    IFS0bits.U1RXIF = 0;
}

///////////////////////////////////////////////////////////////
// PMP ISR
///////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv, shadow)) _PMPInterrupt(void)
{
    uint8_t uart_rxda;

    // Process writes to address 0 (control register) first
    if (PMSTATbits.IB0F) {
        // read in new value for control register
        regs_6850.ctrl = PMDIN1;

        // check for master reset
        if ((regs_6850.ctrl&0x03) == 0x03) {
            // user requesting master reset
            U1BRG = regs_6850.divisor;      // update UART baud rate generator
            U1MODEbits.UARTEN = 1;          // enable UART
            U1STAbits.UTXEN = 1;            // enable UART transmitter
            // XXX -- check status register for additional things to do here
        }
        // check for break request
        else if ((regs_6850.ctrl&0x60) == 0x60) {
            // user request to send break character
            if (U1MODEbits.UARTEN) {
                U1STAbits.UTXBRK = 1;       // set transmit break bit
                U1TXREG = 0;                // initiate break with dummy write
            }
        }
    }
    // Process writes to address 1 (transmit data register)
    else if (PMSTATbits.IB1F) {
        // Clear any existing interrupts on writes to the TDR
        LATAbits.LATA2 = 1;

        // Copy new value to the uart transmit register to queue it for
        // transmmission.  Note:  if U1STAbits.UTXBF is set, the previous
        // contents of this register will be lost.
        U1TXREG = PMDIN1 >> 8;

        // Update the following bits in the status register:
        // b1 - clear TDRE flag
        // b7 - clear IRQ state
        regs_6850.status &= 0b01111101;

        // Update PMDOUT1
        PMDOUT1 = (uint16_t)regs_6850.status | ((uint16_t)regs_6850.rdr << 8);
    }

    // Process reads from address 1 (receive data register)
    if (PMSTATbits.OB1E) {
        // Clear any existing interrupts on reads from the RDR
        LATAbits.LATA2 = 1;

        // Check for more data in FIFO
        if (U1STAbits.URXDA) {
            // rdr is empty and more data is available -- copy it to
            // regs_6850.rdr and set the receive data available flag
            regs_6850.rdr = U1RXREG;
            uart_rxda = 1;
        }
        else {
            // no more data available -- clear the receive data available flag
            uart_rxda = 0;
        }

        // Update the following bits in the status register:
        // b0 - update RDRF flag
        // b7 - clear IRQ state
        regs_6850.status &= 0b01111110;
        regs_6850.status |= uart_rxda;

        // Update PMDOUT1 (also clears the OB1E flag)
        PMDOUT1 = (uint16_t)regs_6850.status | ((uint16_t)regs_6850.rdr << 8);

        // Clear any overflow errors on reads from the RDR
        U1STAbits.OERR = 0;
    }

    // Process writes to address 2 (baud rate generator low byte)
    if (PMSTATbits.IB2F) {
        // Read in new value for low byte of divisor
        regs_6850.divisor &= 0xff00;
        regs_6850.divisor |= PMDIN2 & 0xff;
    }
    // Process writes to address 3 (buad rate generator high byte)
    else if (PMSTATbits.IB3F) {
        // Read in new value for high byte of divisor
        regs_6850.divisor &= 0x00ff;
        regs_6850.divisor |= PMDIN2 & 0xff00;
    }

    // Clear input buffer overflow status bit (otherwise an overflow will
    // prevent PMDIN1 and PMDIN2 from being updated by further writes).
    PMSTATbits.IBOV = 0;

    // Clear PMP interrupt flag
    IFS2bits.PMPIF = 0;
}

int main(void)
{
    uint8_t ctrl;
    uint8_t last_status;
    uint8_t rx_intr_arm;
    uint8_t tx_intr_arm;
    uint16_t temp;

    initHardware();                 // initialize PIC ports & peripherals

    // Set initial values of variables
    regs_6850.rdr = 0;              // initial contents of data register
    regs_6850.divisor = DEF_DIVISOR;// 115200 baud at 7.3728 * 4 MHz
    last_status = 0;

    // Serial Event Dispatch Loop
    for (;;) {
        // Avoid race conditions by reading uart status and 6850 control
        // register only once per iteration
        ctrl = regs_6850.ctrl;

        // Configure baud rate generator based on CR1 and CR0
        U1MODEbits.BRGH = (regs_6850.ctrl&0x3) == 0;

        // Configure parity and stop bits based on CR3 and CR2
        switch (ctrl & 0xc) {
            case 0:
                U1MODEbits.PDSEL = 0b00;        // 8 bits, no parity
                U1MODEbits.STSEL = 1;           // 2 stop bits
                break;
            case 4:
                U1MODEbits.PDSEL = 0b00;        // 8 bits, no parity
                U1MODEbits.STSEL = 0;           // 1 stop bit
                break;
            case 8:
                U1MODEbits.PDSEL = 0b01;        // 8 bits, even parity
                U1MODEbits.STSEL = 0;           // 1 stop bit
                break;
            case 12:
                U1MODEbits.PDSEL = 0b10;        // 8 bits, odd parity
                U1MODEbits.STSEL = 0;           // 1 stop bit
                break;
        }

        // Set RTS pin state based on CR6 and CR5
        LATBbits.LATB4 = (ctrl&0x60) == 0x40;

        // Cache interrupt predicates ahead of critical section
        rx_intr_arm = (ctrl&0x80) && ((last_status&0x4) == 0);
        tx_intr_arm = ((ctrl&0x60) == 0x20) && ((last_status&0x2) == 0);

        // BEGIN CRITICAL SECTION
        // Minimize time required by this section as it will determine
        // the maximmum rate at which some bits in the status register
		// can change in response to uart events.
        __builtin_disi(0x3fff);

        // Update regs_6850.status:
        // b1 - TDRE (asynchronous)
        // b2 - DCDn (asynchronous) (optional)
        // b3 - CTSn (asynchronous) (optional)
        // b4 - FE (asynchronous) (optional)

        // Mask off bits we're updating
        regs_6850.status &= 0b11100001;

        // If CTS is high, inhibit TDRE (bit 1)
        if (PORTAbits.RA1 == 0) {
            regs_6850.status |= (U1STAbits.TRMT<<1);
        }

        regs_6850.status |= (PORTAbits.RA0<<2) | (PORTAbits.RA1<<3) |
                            (U1STAbits.FERR<<4);

        // If 6850 receive interrupts are enabled, test for additional
        // events which can generate them and do so if detected.
        if (rx_intr_arm) {
            if (regs_6850.status&0x4) {
                regs_6850.status |= 0x80;
            }
        }

        // If transmit interrupts are enabled, generate an interrupt if
        // transmit data empty (not full) event occurs.
        if (tx_intr_arm) {
            if (regs_6850.status&0x2) {
                regs_6850.status |= 0x80;
            }
        }

        // Set IRQ pin to be the inverse of b7 of regs_6850.status
        // (this triggers an interrupt on the connected CPU)
        LATAbits.LATA2 = (regs_6850.status&0x80) ? 0 : 1;

        // Copy 6850 status register & rdr to PMP output port
        last_status = regs_6850.status;
        temp = (uint16_t)regs_6850.status | ((uint16_t)regs_6850.rdr << 8);
        if ((temp != PMDOUT1) && (PMSTATbits.OB1E == 0))
            PMDOUT1 = temp;

        // END CRITICAL SECTION
        __builtin_disi(0);

        // Copy 6850 baud rate generator to PMP output port
        PMDOUT2 = regs_6850.divisor;
    }
}