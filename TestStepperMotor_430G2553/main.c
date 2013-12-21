//*******************************************************************************
// 2 stepper motors controller using A4988 driver.
// Platform: MSP-EXP430G2
// M.M. c00@wp.pl
// Connect motors to P2.0-P2.3, control buttons to P1.2, P1.3, P1.3, P1.7.
// See code for details
// <<< Below is original TI license from application note that this
// <<< code is based on.
//*******************************************************************************
//  MSP430 Stepper Motor Controller Board
//  G. Morton
//  Texas Instruments, Inc
//  October 2004
//  Built with Code Composer Essentials for MSP430 V1.00
//******************************************************************************
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//
//******************************************************************************

#include  <msp430g2553.h>

#define DCO_FREQ        1000000      // DCO frequency
#define ONE_SEC_CNT     122          // Number of WDT interrupts in 1 second
#define DEBOUNCE_CNT    0x02         // (1/512)*5 = ~10 msec debounce

#define DIR_MASK        0x01         // 0x01 is clockwise, 0x00 is counter-clockwise
#define STEP_MASK       0x02         // 0x00 is full-stepping, 0x02 is half-stepping
#define MOTION_MASK     0x04         // 0x00 is continuous, 0x04 is single-step

#define DEFAULT_RATE    0x4000       // Default stepping rate
#define MIN_RATE        0x4000       // Minimum stepping rate
#define MAX_RATE        0x0100       // Maximum stepping rate
#define FREQ_STEPS		128		 	 // Number of steps for gradual frequency increase

// Default state is full-stepping, clockwise, continuous
// IMPORTANT: all global variables MUST be initialized
unsigned char state = 1;             // State variable
unsigned int  rate = DEFAULT_RATE;   // Stepping rate
unsigned char change_rate_flag = 0;  // Flag indicating rate change
unsigned int  max_rate = MAX_RATE;   // Maximum stepping rate
unsigned int  min_rate = MIN_RATE;   // Minimum stepping rate
unsigned char ramping_flag = 0;
volatile unsigned char ramp_steps = 0;
volatile unsigned long freq_step = 0;
volatile unsigned long start_freq = 0;
unsigned int SW[4] = {0, 0, 0, 0};   // Switch state counter


//
// Initialization Functions
//
void sys_init(void);
void Set_DCO(unsigned long freq);
void timerA_Init(void);
//void uart0_Init(void);
void wdt_Init(void);


//
// Board operation functions
//
void toggle_stepping_mode(void);
void increase_stepping_rate(void);
void decrease_stepping_rate(void);
void toggle_motion(void);
void toggle_direction(void);


//
// Interrupt Service Routines
//
__interrupt void timerA0_ISR(void);
//#pragma vector=USCIAB0RX_VECTOR
//__interrupt void usart0_Rx_Isr(void);
__interrupt void wdt_ISR(void);
__interrupt void port1_ISR(void);

//  Low-level System Initialization - called prior to main()
//
int _system_pre_init(void)
{
  WDTCTL = WDTPW + WDTHOLD;            // Stop WDT

  return(1);                           // Return 1 to perform C data
                                       // initialization

  // return(0);                        // Return 0 to bypass C data
                                       // initialization
}


//
// Main entry point
//
void main(void)
{
  // Initialize System
  sys_init();

  // Enter low-power mode 0 with interrupts enabled
  __enable_interrupt();
  __bis_SR_register(LPM0 + GIE);
}


//
//  System Initialization
//
void sys_init(void)
{
  // Configure P1.3,4,5,7 to generate an
  // interrupt on high-to-low signal
  // transition
  P1IES = 0xB8;
  P1IFG = 0x00;
  P1IE = 0xB8;

  // Clear P1.0,6 outputs, set pullups on P1.3,4,5,7
  // Configure P1.0,6 as outputs
  // Configure P2.0,1,2,3 as outputs
  P1OUT = 0xB8;
  P1DIR |= 0x41;
  P1REN = 0xB8;

  P2OUT = 0x00;
  P2DIR |= 0x0F;

  P3DIR = 0xFF;

  // Set DCO frequency
  Set_DCO(DCO_FREQ);

  // Initialize Timer A
  timerA_Init();

  // Initialize UART0
//  uart0_Init();

  // Initialize WDT
  wdt_Init();
}


//
// Set DCO frequency
//
void Set_DCO(unsigned long freq)
{
//  unsigned int clkCnt;
//  unsigned int numDcoClks;
//  unsigned int prevCnt = 0;

  // PUC value for DCOx = 3
  // PUC value for RSELx = 4

  // Basic Clock system Control Register 1 p. 4-15
//  BCSCTL1 |= DIVA_3;                   // ACLK = LFXT1CLK/8 = 4096 Hz

//  numDcoClks = freq/4096;              // Number of DCO clocks in one
//                                       // ACLK/8 period
//
//  // Timer A Capture/Compare Control Register p. 11-22
//  TACCTL2 = CM_1 + CCIS_1 + CAP;       // Capture on rising Edge
//                                       // Capture input is CCI2B = ACLK
//                                       // Async capture
//                                       // Capture mode
//                                       // Output mode is OUT bit
//                                       // Interrupt disabled
//                                       // OUT bit is 0
//                                       // Clear capture overflow bit (COV)
//                                       // Clear interrupt pending flag (CCIFG)
//
//  // Timer A Control Register p. 11-20
//  TACTL = TASSEL_2 + MC_2 + TACLR;     // Timer A clock source is SMCLK
//                                       // Input clock divider is 1,
//                                       // Continuous mode (counts up to 0xffff)
//                                       // Reset TAR, TACLK divider, and count direction
//                                       // Interrupt is disabled
//                                       // Clear interrupt pending flag (TAIFG)
//
//  while(1)
//  {
//    while( !(TACCTL2 & CCIFG) )
//    {
//      // Wait for capture event
//    }
//
//    TACCTL2 &= ~CCIFG;                 // Capture occured, clear flag
//
//    clkCnt = TACCR2 - prevCnt;         // Number of clocks since last capture
//
//    prevCnt = TACCR2;                  // Save current clock count
//
//    if( numDcoClks == clkCnt )
//    {
//      break;                           // If equal, leave "while(1)"
//    }
//    else if( clkCnt > numDcoClks )     // DCO is too fast, slow it down
//    {
//      DCOCTL--;
//
//      if( DCOCTL == 0xFF )
//      {
//        if( BCSCTL1 & 0x07 )
//        {
//          BCSCTL1--;                   // DCO role under?, dec RSEL
//        }
//        else
//        {
//          break;                       // Error condition, break loop
//        }
//      }
//    }
//    else                               // DCO is too slow, speed it up
//    {
//      DCOCTL++;
//
//      if( DCOCTL == 0x00 )
//      {
//        if( (BCSCTL1 & 0x07) != 0x07 )
//        {
//          BCSCTL1++;                   // DCO role over? higher RSEL
//        }
//        else
//        {
//          break;                       // Error condition, break loop
//        }
//      }
//    }
//  }
//
  BCSCTL1 = XT2OFF
//		  | XTS
//		  | DIVA_0	                  // ACLK = LFXT1CLK/1 = 32768 Hz
		  | RSEL0
		  | RSEL1
		  | RSEL2
//		  | RSEL3
		  ;
  BCSCTL3 = LFXT1S0 | XCAP0;
//
//  TACCTL2 = 0;                         // Stop TACCR2
//  TACTL = 0;                           // Stop Timer_A
}


//
// WDT Initialization
//
void wdt_Init(void)
{
  // Configure watchdog timer
  // to generate an interrupt
  // 512 times a second (once every ~2 msec)
  // using ACLK as the clock source
  WDTCTL = WDTPW       // Watchdog timer password
           // + WDTHOLD   // Watchdog timer hold
           // + WDTNMIES  // Watchdog timer NMI edge select
           // + WDTNMI    // Watchdog timer NMI select
           + WDTTMSEL  // Watchdog timer mode select
           + WDTCNTCL  // Watchdog timer count clear
//           + WDTSSEL   // Watchdog timer clock source select
           + WDTIS0    // Watchdog timer interval select
//           + WDTIS1    // Watchdog timer interval select
           ;
}


//
// UART0 Initialization
//
//void uart0_Init(void)
//{
//  // Configure P3.5,4 as URXD0 and UTXD0
//  P3SEL |= 0x30;
//
//  // Configure USART0 Control Register
//  // Parity Enable: Off (0)
//  // Parity Select: Odd (0)
//  // Stop Bit Select: 1 (0)
//  // Character Length: 8 (1)
//  // Listen Enable: Off (0)
//  // Sync Mode: UART (0)
//  // Multiprocessor Mode: Idle-line (0)
//  // Software Reset Enable: On (1)
//  U0CTL = CHAR + SWRST;
//
//  // Module Enable Register 2
//  ME2 |= UTXE0 + URXE0;
//
//  // USART0 Transmit Control Register
//  // Clock Polarity: UCLKI = UCLK (0)
//  // BRCLK Source: ACLK (1)
//  // UART Receive Start-Edge: Disabled (0)
//  // Transmitter Wake: data (0)
//  U0TCTL = SSEL0;
//
//  //
//  // Configure 9600 baud from 32.768 kHz BRCLK
//  //
//
//  // USART0 Baud Rate Control Register 0
//  U0BR0 = 0x03;
//
//  // USART0 Baud Rate Control Register 1
//  U0BR1 = 0x00;
//
//  // USART0 Modulation Control Register
//  U0MCTL = 0x4A;
//
//  // Release USART0 for Operation
//  U0CTL &= ~SWRST;
//
//  // Enable USART0 RX interrupts
//  IE2 |= URXIE0;
//}


//
// Timer_A Initialization
//
void timerA_Init(void)
{
  // Initialize Compare Register 0
  TACCR0 = rate;

  // Configure Capture/Control Register 0
  // Capture Mode: Off (0)
  // Input Select: CCIxA (0)
  // Synchronize: No (0)
  // Mode: Compare (0)
  // Output Mode: OUT bit (0)
  // Interrupt Enable: On (1)
  TACCTL0 = CCIE;

  // Configure Timer A Control Register
  // Clock source: SMCLK (TASSEL_2)
  // Input divider: /1   (0)
  // Mode Control: Up (MC_1)
  // Clear: Yes (TACLR)
  // Interrupt Enable: No (0)
  TACTL = TASSEL_2 + MC_1 + TACLR;
}


//
//  Timer A0 Interrupt Service Routine
//
#pragma vector=TIMER0_A0_VECTOR
__interrupt void timerA0_ISR(void)
{
//  unsigned char index;
  unsigned char p2 = P2OUT;
//  unsigned char p3 = 0;

  // Check to see if stepping rate
  // needs to be changed
  if( change_rate_flag )
  {
    TACCR0 = rate;
    change_rate_flag = 0;
  }

//  //
//  // Check to see if LED D1 is illuminated
//  //
//  if( P1OUT & 0x01 )
//  {
//    p1 |= 0x01;
//  }
//
//  //
//  // Check to see if LED D2 is illuminated
//  //
//  if( P1OUT & 0x40 )
//  {
//    p1 |= 0x40;
//  }

  //
  // Check current state
  //
  switch( (state & 0x3) )
  {
    case 0x00:  // Full-stepping, counter-clockwise
      p2 ^= 0x05;
      p2 |= 0x0A;
      P2OUT = p2;
      break;
    case 0x01:  // Full-stepping, clockwise
      p2 ^= 0x05;
      p2 &= ~0x0A;
      P2OUT = p2;
      break;
    case 0x02:  // Half-stepping, counter-clockwise
      break;
    case 0x03:  // Half-stepping, clockwise
      break;
    default:
      _never_executed();
      break;
  };

  // Disable Timer A0 interrupt if in single-step state
  if( state & MOTION_MASK )
  {
    TACCTL0 &= ~CCIE;
  }
}


//
// UART0 Receive ISR
//
//__interrupt void usart0_Rx_Isr(void)
//{
//  // USART 0 TX buffer ready?
//  while( !(IFG1 & UTXIFG0) );
//
//  switch(U0RXBUF)
//  {
//  case 'D': // Direction (clockwise or counter-clockwise)
//  case 'd':
//    toggle_direction();
//    break;
//  case 'C': // Motion (continuous or step)
//  case 'c':
//    toggle_motion();
//    break;
//  case 'M': // Stepping mode (full or half)
//  case 'm':
//    toggle_stepping_mode();
//    break;
//  case 'F': // Increase stepping rate
//  case 'f':
//    increase_stepping_rate();
//    break;
//  case 'S': // Decrease stepping rate
//  case 's':
//    decrease_stepping_rate();
//    break;
//  default:
//    break;
//  };
//
//  // Copy RXBUF0 to TXBUF0
//  U0TXBUF = U0RXBUF;
//}


//
// WDT Interrupt Service Routine
//
#pragma vector=WDT_VECTOR
__interrupt void wdt_ISR(void)
{
  unsigned char sw_state;
  static unsigned char one_sec_flag = 0;

  if(ramping_flag) {
	  if(ramp_steps-- == 0) {
		  ramping_flag = 0;
	  } else {
		  start_freq += freq_step;
		  rate = 0xFFFFFFFFL / start_freq;
		  change_rate_flag = 1;
	  }
  }
  // Get current state of input pins P1.3,4,5,7
  sw_state = ((~P1IN & 0x38) >> 3) | ((~P1IN & 0x80) >> 4);

  // Check to see if no switches are active
  if( sw_state == 0x00 )
  {
    // Disable WDT Interrupt
	if(ramping_flag == 0)
		IE1 &= ~WDTIE;

    // Check to see if S2 was active for less than 1 second
    if( !one_sec_flag && (SW[1] >= DEBOUNCE_CNT) )
    {
      toggle_motion();
    }

    // Reset switch state counters
    SW[0] = 0;
    SW[1] = 0;
    SW[2] = 0;
    SW[3] = 0;

    // Reset flag
    one_sec_flag = 0;

    // Re-enable P1.3,4,5,7 interrupts
    P1IFG = 0x00;
    P1IE = 0xB8;
  }
  else
  {
    // Check to see if S1 is active
    if( sw_state & 0x01 )
    {
      if( SW[0] < ONE_SEC_CNT )
      {
        // Increment switch state counter
        ++SW[0];
      }

      if( SW[0] == DEBOUNCE_CNT )
      {
        toggle_direction();
      }
    }
    else  // Switch S1 is not active
    {
      SW[0] = 0;
    }

    // Check to see if S2 is active
    if( sw_state & 0x02 )
    {
      if( SW[1] < ONE_SEC_CNT )
      {
        // Increment switch state counter
        ++SW[1];
      }

      if( SW[1] == ONE_SEC_CNT )
      {
        toggle_stepping_mode();
        one_sec_flag = 1;
        SW[1] = 0;
      }
    }
    else  // Switch S2 is not active
    {
      // Check to see if S2 was active for less than 1 second
      if( !one_sec_flag && (SW[1] >= DEBOUNCE_CNT) )
      {
        toggle_motion();
      }

      one_sec_flag = 0;
      SW[1] = 0;
    }

    // Check to see if S3 is active
    if( sw_state & 0x04 )
    {
      // Check to see if in continuous stepping mode
      if( (state & MOTION_MASK) == 0 )
      {
        if( SW[2] < ONE_SEC_CNT )
        {
          // Increment switch state counter
          ++SW[2];
        }

        if( SW[2] == DEBOUNCE_CNT )
        {
          increase_stepping_rate();
        }
      }
      else // Single-step mode
      {
        // Increment switch state counter
        ++SW[2];

        if( (SW[2] % DEBOUNCE_CNT) == 0 )
        {
          increase_stepping_rate();
        }
      }
    }
    else  // Switch S3 is not active
    {
      SW[2] = 0;
    }

    // Check to see if S4 is active
    if( sw_state & 0x08 )
    {
      if( SW[3] < ONE_SEC_CNT )
      {
        // Increment switch state counter
        ++SW[3];
      }

      if( SW[3] == DEBOUNCE_CNT )
      {
        decrease_stepping_rate();
      }
    }
    else  // Switch S4 is not active
    {
      SW[3] = 0;
    }
  }
}


//
// PORT 1 Interrupt Service Routine
//
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR(void)
{
  // Disable P1 interrupts
  P1IE = 0x00;

  // Clear P1 interrupt flag
  P1IFG = 0x00;

  // Enable WDT Interrupt
  IE1 |= WDTIE;
}


//
// Increase stepping rate
//
void increase_stepping_rate(void)
{
  unsigned int new_rate;
  unsigned long end_freq;

  // Check to see if in continuous mode
  if( (state & MOTION_MASK) == 0 && ramping_flag == 0)
  {

    new_rate = rate >> 1;
    if( new_rate >= max_rate ) {
		start_freq = 0xFFFFFFFFL / rate;
		end_freq = 0xFFFFFFFFL / new_rate;
		ramp_steps += FREQ_STEPS;
		freq_step = (end_freq - start_freq) / ramp_steps;
		ramping_flag = 1;
		IE1 |= WDTIE;
    }
//
//    if( new_rate >= max_rate )
//    {
//      rate = new_rate;
//      change_rate_flag = 1;
//    }
  }

  // Enable Timer A0 interrupts
  TACCTL0 |= CCIE;
}


//
// Decrease stepping rate
//
void decrease_stepping_rate(void)
{
  // Check to see if in continuous mode
  if( (state & MOTION_MASK) == 0 && ramping_flag == 0)
  {
    if( rate <= (min_rate >> 1) )
    {
      rate <<= 1;
//      rate += RATE_STEP;
      change_rate_flag = 1;
    }
  }

  // Enable Timer A0 interrupts
  TACCTL0 |= CCIE;
}



//
// Toggle between full-stepping and half-stepping modes
//
void toggle_stepping_mode(void)
{
  // Toggle stepping mode state
  state ^= STEP_MASK;

  // Check to see if in half-stepping mode
  if( state & STEP_MASK )
  {
    // Switching from full-stepping to half-stepping mode
    // Timer A0 ISR frequency needs to be doubled
    rate = (rate >> 1);
    change_rate_flag = 1;
    max_rate = (MAX_RATE >> 1);
    min_rate = (MIN_RATE >> 1);

    // Turn on LED D2
    P1OUT |= 0x40;
  }
  else // Full-stepping mode
  {
    // Switching from half-stepping to full-stepping mode
    // Timer A0 ISR frequency needs to be halved
    rate = (rate << 1);
    change_rate_flag = 1;
    max_rate = MAX_RATE;
    min_rate = MIN_RATE;

    // Turn off LED D2
    P1OUT &= ~0x40;
  }
}


//
// Toggle between continuous mode and single-step mode
//
void toggle_motion(void)
{
  state ^= MOTION_MASK;

  // Check to see if in continuous stepping mode state
  if( (state & MOTION_MASK) == 0 )
  {
    // Enable Timer A0 interrupts
    TACCTL0 |= CCIE;

    // Turn off LED D1
    P1OUT &= ~0x01;
  }
  else
  {
    // Turn on LED D1
    P1OUT |= 0x01;
  }
}


//
// Toggle direction
//
void toggle_direction(void)
{
  state ^= DIR_MASK;
}



