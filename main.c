/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F5529LP:  simpleUsbBackchannel example
//
//   Description: 	Demonstrates simple sending over USB, as well as the F5529's
//                  backchannel UART.
//
//   Texas Instruments Inc.
//   August 2013
//******************************************************************************

// Basic MSP430 and driverLib #includes
#include "msp430.h"
#include "driverlib/MSP430F5xx_6xx/wdt_a.h"
#include "driverlib/MSP430F5xx_6xx/ucs.h"
#include "driverlib/MSP430F5xx_6xx/pmm.h"
#include "driverlib/MSP430F5xx_6xx/sfr.h"

// USB API #includes
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"

#include "USB_app/usbConstructs.h"

// Application #includes
//#include "BCUart.h"           // Include the backchannel UART "library"
#include "hal.h"              // Modify hal.h to select your hardware

#include "stdio.h"

//#include "D:\Users\pleroy\Documents\PoSAR\RSC\HexRegisterValues_9-800_10-100.h" // July 2019
//#include "D:\Users\pleroy\Documents\PoSAR\RSC\HexRegisterValues_9-700_10-300.h" // 2020
//#include "D:\Users\pleroy\Documents\PoSAR\RSC\HexRegisterValues_9-700_10-300_2020_02_20.h" // 2020 02 20
#include "D:\Users\pleroy\Documents\PoSAR\RSC\HexRegisterValues_9-700_10-300_2020_03_02_flag0_downRamp.h" // 2020 03 02

#define DELAY 8*8*7
#define TOGGLE_P8_1 P8OUT = (P8OUT & 0x02) ? P8OUT & ~0x02 : P8OUT | 0x02;
#define P81_ON  P8OUT |=  0x02;
#define P81_OFF P8OUT &= ~0x02;
#define P22_ON  P2OUT |=  0x04;
#define P22_OFF P2OUT &= ~0x04;
#define TIMEOUT 6

unsigned char test1[5];
unsigned int test[4];

/* You have a choice between implementing this as a CDC USB device, or a HID-
 * Datapipe device.  With CDC, the USB device presents a COM port on the host;
 * you interact with it with a terminal application, like Hyperterminal or
 * Docklight.  With HID-Datapipe, you interact with it using the Java HID Demo
 * App available within the MSP430 USB Developers Package.
 *
 * By default, this app uses CDC.  The HID calls are included, but commented
 * out.
 *
 * See the F5529 LaunchPad User's Guide for simple instructions to convert
 * this demo to HID-Datapipe.  For deeper information on CDC and HID-Datapipe,
 * see the USB API Programmer's Guide in the USB Developers Package.
 */

void init_spi(void);

void main(void)
{
   	WDTCTL = WDTPW + WDTHOLD;		// Halt the dog

    // MSP430 USB requires a Vcore setting of at least 2.  2 is high enough
	// for 8MHz MCLK, below.
    PMM_setVCore(PMM_CORE_LEVEL_2);

    initPorts();           // Config all the GPIOS for low-power (output low)
    initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz

    // INIT THE BACK-CHANNEL UART
    //bcUartInit();

    // INIT SPI
    init_spi();

    // launch the timer for the firmware upload
    TA2CCTL0 = CCIE;            // CCR0 interrupt enable
    TA2CCR0 = 0x3fff;           // start Timer_A2

 //   USB_setup(TRUE,TRUE);  // Init USB; if a USB host (PC) is present, connect
    __enable_interrupt();  // Enable interrupts globally
    __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
    __no_operation();                         // For debugger
}

//*****************************************************************
//*****************************************************************
// REFERENCE MSP430F55xx_Code_Examples/CMSP430F55xx_uscia0_spi_09.c

unsigned char MST_Data,SLV_Data;
unsigned char temp;

void init_spi(void)
{
  volatile unsigned int i;

//  WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

  // P1.0 //
  P1DIR |= 0x01;    // Set P1.0 to output direction
  // P1.6 //
  P1DIR |= 0x40;    // Set P1.6 to output direction
  // P1.1 //
  P1DIR &= ~0x02;   // Set P1DIR to 0 for P1.1 => this is an INPUT pin
  P1REN |=  0x02;   // Set P1.1 pullup / pulldow resistor enable
  P1OUT |=  0x02;   // Set P1.1 input with pullup resistor (P1DIR is 0 for this bit)
  P1IES |=  0x02;   // Set P1.1 high to low transition
  // P2.0 //
  P2DIR &= ~0x01;   // Set P2DIR to 0 for P2.0 => this is an INPUT pin
  P2REN |=  0x01;   // Set P2.0 pullup / pulldow resistor enable
  P2OUT |=  0x01;   // Set P2.0 input with pullup resistor (P2DIR is 0 for this bit)
  P2IES |=  0x01;   // Set P2.0 high to low transition
  // P2.1 //
  P2DIR &= ~0x02;   // Set P2DIR to 0 for P2.1 => this is an INPUT pin
  P2REN |=  0x02;   // Set P2.1 pullup / pulldow resistor enable
  P2OUT |=  0x02;   // Set P2.1 input with pullup resistor (P2DIR is 0 for this bit)
  P2IES |=  0x02;   // Set P2.1 high to low transition
  // P2.2 //
  P2DIR |= 0x04;    // Set P2.2 to output direction
  // P3.2 //
  P3DIR |= 0x04;    // Set P3.2 to output direction
  P3OUT |= 0x04;    // P3.2 => 1
  // P4.7 //
  P4DIR |= 0x80;    // Set P4.7 to output direction
  // P8.1 //
  P8DIR |= 0x02;    // Set P8.1 to output direction

  // PxSEL Port Select Register [0] I/O function is selected [1] peripheral module function is selected
  P2SEL |= BIT7;                            // P2.7 option select
  P3SEL |= BIT3;                            // P3.3 option select
  P3SEL |= BIT4;                            // P3.4 option select

  // TIMER CONFIGURATION
  TA0CCR0 = 0;                      // Timer_A0 Capture/Compare 0 register
  TA0CTL |= TASSEL_1 + MC_1 + TACLR;// ACLK 32768 Hz
  TA1CCR0 = 0;                      // Timer_A1 Capture/Compare 0 register
  TA1CCR1 = 0x3fff;                 // Timer_A1 Capture/Compare 1 Register
  TA1CTL |= TASSEL_1 + MC_1 + TACLR;// ACLK 32768 Hz
  TA2CCR0 = 0;                      // Timer_A2 Capture/Compare 0 register
  TA2CTL |= TASSEL_1 + MC_1 + TACLR;// ACLK 32768 Hz

  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL0 |= UCMST + UCSYNC + UCCKPH + UCMSB;    // 3-pin
                                            // 8-bit SPI master
                                            // Clock polarity low
                                            // Clock phase
                                            // MSB

  // USCI clock source select. These bits select the BRCLK (baud rate) source clock.
  UCA0CTL1 |= UCSSEL__SMCLK;                // 00 UCLK => UCAxCLK (external USCI clock)
                                            // 01 ACLK
                                            // 10 SMCLK => Susystem Master Clock
                                            // 11 SMCLK
  // (UCAxBR0 + UCAxBR1 × 256) forms the prescaler value UCBRx
  //UCA0BR0 = 0x02;                           // /2
  UCA0BR0 = 0x40;                           // /64
  UCA0BR1 = 0;                              //
  UCA0MCTL = 0;                             // No modulation - MCTL Modulation Control Register
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  //UCA0IE |= UCRXIE|UCTXIE;                  // Enable USCI_A0 RX interrupt

  P1IFG &= ~0x02;   // Clear  P1.1 interrupt flag
  P1IE  |=  0x02;   // Enable P1.1 interrupt

  P1OUT |= 0x40;    // P1.6 => 1
}

int ledIsOn_P1_0 = 0;
int ledIsOn_P4_7 = 0;
int bytesSent = 1;

//************
//************
// USCI A0 ISR
//************
//************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile unsigned int i;

    switch(__even_in_range(UCA0IV,4))
    {
    case 0:     // Vector 0 - no interrupt
        break;
    case 2:     // Vector 2 - Interrupt source: data received / interrupt flag UCRXIFG / interrupt priority highest
        if (ledIsOn_P1_0) {
            ledIsOn_P1_0 = 0;
            P1OUT |= 0x01;    // light LED
            }
        else {
            ledIsOn_P1_0 = 1;
            P1OUT &= ~0x01;   // clear LED
            }
        break;

    case 4:     // Vector 4 - Interrupt source: transmit buffer empty / interrupt flag UCTXIFG / interrupt priority lowest
        break;

    default:
        break;
    }
}

//**********
//**********
// PORT1 ISR
//**********
//**********

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile unsigned int i;

    switch(__even_in_range(P1IV,16))
        {
        case 0:     // Vector 0 - No interrupt pending
            break;
        case 2:     // Vector 2 - Interrupt Source: Port 1.0 interrupt; Interrupt Flag: P1IFG.0; Interrupt Priority: Highest
            break;
        case 4:     // Vector 4 - Interrupt Source: Port 1.1 interrupt; Interrupt Flag: P1IFG.1
            P4OUT |= 0x80;              // P4.7 => 1
            P1IE &= ~0x02;              // disable P1.1 interrupt
            TA0CCTL0 = CCIE;            // CCR0 interrupt enable
            TA0CCR0 = 0x3fff;           // start Timer_A0
            break;
        case 6:     // Vector 6 - Interrupt Source: Port 1.2 interrupt; Interrupt Flag: P1IFG.2
            P4OUT ^= 0x80;              // Toggle P4.7
            break;
        default:
            break;
        }
}

//**********
//**********
// PORT2 ISR
//**********
//**********

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) PORT2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile unsigned int i;

    switch(__even_in_range(P2IV,16))
        {
        case 0:     // Vector 0 - no interrupt
            break;
        case 2:     // Vector 2 - Interrupt Source: Port 2.0 interrupt; Interrupt Flag: P2IFG.0; Interrupt Priority: Highest
            P4OUT |= 0x80;              // P4.7 => 1
            P2IE &= ~0x01;              // disable P2.0 interrupt
            TA1CCTL1 = CCIE;            // CCR1 interrupt enable
            TA1CCR0 = 0x3fff;           // start Timer_A1
            break;
        case 4:     // Vector 4 - Interrupt Source: Port 2.1 interrupt; Interrupt Flag: P2IFG.1
            P4OUT |= 0x80;              // P4.7 => 1
            P2IE &= ~0x02;              // disable P2.1 interrupt
            TA1CCTL0 = CCIE;            // CCR0 interrupt enable
            TA1CCR0 = 0x3fff;           // start Timer_A1
            break;
        default:
            break;
        }
}

//*************
//*************
// TIMER A1 ISR
//*************
//*************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile unsigned int i;
    static int counter = 0;
    static int isEnabled = 0;

    switch(__even_in_range(TA1IV,16))
        {
        case 0:     // Vector 0 - no interrupt
            break; //
        case 2:     // Vector 2 - Interrupt Source: Capture/compare 1; Interrupt Flag: TAxCCR1 CCIFG; Interrupt Priority: Highest
            if (counter == TIMEOUT)
            {
                TA1CCR0 = 0x00;     // stop the timer
                TA1CCTL1 &= ~CCIE;  // disable Capture/compare interrupt
                TA1CCTL1 &= ~CCIFG; // clear Capture/compare interrupt flag
                counter = 0;
                // OUT
                P4OUT &= ~0x80;     // P4.7 => 0
                // upload the configuration to the LMW2492 via SPI
                P1OUT |= 0x01;      // Toggle P1.0

                P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                if (isEnabled)
                {
                    // DISABLE RAMP
                    P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x3A;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    __delay_cycles(DELAY);
                    P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
                    // POWER DOWN
                    P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x02;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    __delay_cycles(DELAY);
                    P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
                    isEnabled = 0;
                    P22_OFF
                    P2IES |=  0x01;     // P2.0 HIGH to LOW transition
                }
                else
                {
                    // POWER UP
                    P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x02;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x01;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    __delay_cycles(DELAY);
                    P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
                    // ENABLE RAMP
                    P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                    UCA0TXBUF = 0x00;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x3A;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    UCA0TXBUF = 0x01;
                    while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                    __delay_cycles(DELAY);
                    P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
                    isEnabled = 1;
                    P22_ON
                    P2IES &= ~0x01;     // P2.0 LOW to HIGH transition
                }
                P2IFG &= ~0x01;     // clear  P2.0 interrupt flag
                P2IE  |=  0x01;     // enable P2.0 interrupt
            }
            else
            {
                if ( !(P2IN & 0x01) && (P2IES & 0x01) )         // check that P2.0 = 0, waiting for HIGH to LOW transitions
                {
                    counter++;
                    TOGGLE_P8_1
                }
                else if ( (P2IN & 0x01) && !(P2IES & 0x01) )    // check that P  = 1, waiting for LOW to HIGH transitions
                {
                    counter++;
                    TOGGLE_P8_1
                }
                else
                {
                    TA1CCR0 = 0x00;     // stop the timer
                    TA1CCTL1 &= ~CCIE;  // disable Capture/compare interrupt
                    TA1CCTL1 &= ~CCIFG; // clear Capture/compare interrupt flag
                    P2IFG &= ~0x01;     // clear  P2.0 interrupt flag
                    P2IE  |=  0x01;     // enable P2.0 interrupt
                    counter = 0;
                    // OUT
                    P1OUT &= ~0x01;     // P1.0 => 0
                }
            }
            break;
        case 4:     // Vector 4 - Interrupt Source: Capture/compare 2; Interrupt Flag: TAxCCR2 CCIFG
            break;
        case 6:     // Vector 6 - Interrupt Source: Capture/compare 3; Interrupt Flag: TAxCCR3 CCIFG
            break;
        case 8:     // Vector 8 - Interrupt Source: Capture/compare 4; Interrupt Flag: TAxCCR4 CCIFG
            break;
        case 10:    // Vector a - Interrupt Source: Capture/compare 5; Interrupt Flag: TAxCCR5 CCIFG
            break;
        case 12:    // Vector c - Interrupt Source: Capture/compare 6; Interrupt Flag: TAxCCR6 CCIFG
            break;
        case 14:    // Vector e - Interrupt Source: Timer overflow; Interrupt Flag: TAxCTL TAIFG; Interrupt Priority: Lowest
            break;
        default:
            break;
        }
}

//*******************************
//*******************************
// TIMER0 A0 / LOAD CONFIGURATION
//*******************************
//*******************************

// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static int counter = 0;
    unsigned char *ptr;
    unsigned int r;
    if (counter == TIMEOUT)
    {
        TA0CCR0 = 0x00;     // stop the timer
        TA0CCTL0 &= ~CCIE;  // disable Capture/compare interrupt
        TA0CCTL0 &= ~CCIFG; // clear Capture/compare interrupt flag
        P1IFG &= ~0x02;     // clear P1.1 interrupt flag
        P1IE |= 0x02;       // enable P1.1 interrupt
        counter = 0;
        // OUT
        P4OUT &= ~0x80;     // P4.7 => 0
        // upload the configuration to the LMW2492 via SPI
        P1OUT |= 0x01;      // Toggle P1.0
        for (r = 0; r < nbRegs; r++)
        {
            P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
            ptr = (unsigned char*) &config[r];
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = ptr[1];
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = ptr[0];
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            __delay_cycles(DELAY);
            P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
        }
        P1OUT &= ~0x01;         // P1.0 => 0
    }
    else
    {
        if (!(P1IN & 0x02))   // check that the button is still pushed, P1.1 = 0
        {
            counter++;
        }
        else
        {
            TA0CCR0 = 0x00;     // stop the timer
            TA0CCTL0 &= ~CCIE;  // disable Capture/compare interrupt
            TA0CCTL0 &= ~CCIFG; // clear Capture/compare interrupt flag
            P1IFG &= ~0x02;     // clear P1.1 interrupt flag
            P1IE |= 0x02;       // enable P1.1 interrupt
            counter = 0;
            // OUT
            P1OUT &= ~0x01;     // P1.0 => 0
        }
    }
}

//*******************
//*******************
// TIMER0 A1 / ON OFF
//*******************
//*******************

// Timer0 A1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static int counter = 0;
    static int isEnabled = 0;
    if (counter == TIMEOUT)
    {
        TA1CCR0 = 0x00;     // stop the timer
        TA1CCTL0 &= ~CCIE;  // disable Capture/compare interrupt
        TA1CCTL0 &= ~CCIFG; // clear Capture/compare interrupt flag
        counter = 0;
        // OUT
        P4OUT &= ~0x80;     // P4.7 => 0
        // upload the configuration to the LMW2492 via SPI
        P1OUT |= 0x01;      // Toggle P1.0

        P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
        if (isEnabled)
        {
            // DISABLE RAMP
            P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x3A;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            __delay_cycles(DELAY);
            P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
            // POWER DOWN
            P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x02;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            __delay_cycles(DELAY);
            P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
            isEnabled = 0;
            P22_OFF
        }
        else
        {
            // POWER UP
            P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x02;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x01;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            __delay_cycles(DELAY);
            P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
            // ENABLE RAMP
            P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
            UCA0TXBUF = 0x00;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x3A;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            UCA0TXBUF = 0x01;
            while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
            __delay_cycles(DELAY);
            P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
            isEnabled = 1;
            P22_ON
        }
        P2IFG &= ~0x02;     // clear  P2.1 interrupt flag
        P2IE  |=  0x02;     // enable P2.1 interrupt
        P1OUT &= ~0x01;     // P1.0 => 0
    }
    else
    {
        if (!(P2IN & 0x02))     // check that the button is still pushed, P2.1 = 0
        {
            counter++;
        }
        else
        {
            TA1CCR0 = 0x00;     // stop the timer
            TA1CCTL0 &= ~CCIE;  // disable Capture/compare interrupt
            TA1CCTL0 &= ~CCIFG; // clear Capture/compare interrupt flag
            P2IFG &= ~0x02;     // clear  P2.1 interrupt flag
            P2IE  |=  0x02;     // enable P2.1 interrupt
            counter = 0;
            // OUT
            P1OUT &= ~0x01;     // P1.0 => 0
        }
    }
}

//****************************************
//****************************************
// TIMER0 A2 LOAD CONFIGURATION AT STARTUP
//****************************************
//****************************************

// Timer0 A2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) TIMER2_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    {
        unsigned char *ptr;
        unsigned int r;
        static int counter = 0;
        static int ledIsOn = 0;
        if (counter == TIMEOUT)
        {
            TA2CCR0 = 0x00;     // stop the timer
            TA2CCTL0 &= ~CCIE;  // disable Capture/compare interrupt
            TA2CCTL0 &= ~CCIFG; // clear Capture/compare interrupt flag
            counter = 0;

            P1OUT |= 0x01;      // Toggle P1.0

            // upload the configuration to the LMW2492 via SPI
            for (r = 0; r < nbRegs; r++)
            {
                P3OUT &= ~0x04;     // P3.2 => 0 / LE signal of the LMX2492
                ptr = (unsigned char*) &config[r];
                UCA0TXBUF = 0x00;
                while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                UCA0TXBUF = ptr[1];
                while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                UCA0TXBUF = ptr[0];
                while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
                __delay_cycles(DELAY);
                P3OUT |= 0x04;      // P3.2 => 1 / LE signal of the LMX2492
                }
            // enable interruptions from buttons
            P2IFG &= ~0x01;   // Clear  P2.0 interrupt flag
            P2IE  |=  0x01;   // Enable P2.0 interrupt
            P2IFG &= ~0x02;   // Clear  P2.1 interrupt flag
            P2IE  |=  0x02;   // Enable P2.1 interrupt
            P1OUT &= ~0x01;         // P1.0 => 0
        }
        else
        {
            counter++;
            if (ledIsOn==1)
            {
                ledIsOn = 0;
                P1OUT &= ~0x01;     // P1.0 => 0
                TOGGLE_P8_1
            }
            else
            {
                ledIsOn = 1;
                P1OUT |= 0x01;      // Toggle P1.0
                TOGGLE_P8_1
            }
        }
    }
}
