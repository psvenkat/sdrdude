/*
 * Code to initialise the DMA functionality
 *  *
 * STM32-SDR: A software defined HAM radio embedded system.
 * Copyright (C) 2013, STM32-SDR Group
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <Arduino.h>

//#include	"stm32f4xx.h"
//#include	"stm32f4xx_rcc.h"
//#include 	"stm32f4xx_dma.h"
//#include 	"stm32f4xx_spi.h"
#include 	"Init_DMA.h"
//#include 	"Init_Codec.h"
//#include 	"Init_I2C.h" // reference for Delay();
//#include	"misc.h"

#ifdef __cplusplus
extern "C" {
#endif

int16_t Rx0BufferDMA[BUFFERSIZE];
int16_t Tx0BufferDMA[BUFFERSIZE];
int16_t Rx1BufferDMA[BUFFERSIZE];
int16_t Tx1BufferDMA[BUFFERSIZE];

extern volatile int16_t DSP_Flag;
extern volatile int16_t AGC_Flag;

uint8_t DMA_RX_Memory;
uint8_t DMA_TX_Memory;
 
volatile bool l0, l1;

void adc_setup ();

void dac_setup ();

void setup_ADC_Timer();

void setup_DAC_Timer();
  
void setup_pio_TIOA0 ();

void Audio_DMA_Init(void)
{
  adc_setup () ;         // setup ADC (causes Touch event?)

  dac_setup () ;        // setup up DAC auto-triggered at 48kHz

  setup_ADC_Timer();

  setup_DAC_Timer();
  
  setup_pio_TIOA0 () ;  // drive Arduino pin 2 at 48kHz to bring clock out

}

void setup_ADC_Timer()
{
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the TC0 channel 0

  TcChannel * t = &(TC0->TC_CHANNEL)[0] ;    // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
  t->TC_SR ;                   // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                  // waveform mode
              TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
 
  t->TC_RC =  875/2 ;     // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA =  875/4 ;     // roughly square wave
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
 
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.
}

void setup_DAC_Timer()
{
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+1) ;  // clock the TC0 channel 1

  TcChannel * t = &(TC0->TC_CHANNEL)[1] ;    // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
  t->TC_SR ;                   // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                  // waveform mode
              TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
 
  t->TC_RC =  875/4 ;     // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA =  875/8 ;     // roughly square wave
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
 
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.
}

void setup_pio_TIOA0 ()  // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
{
  //PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  //PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;  // disable PIO interrupts -- This will disable LCD too
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
}


void dac_setup ()
{
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  NVIC_EnableIRQ(DACC_IRQn);
  DACC->DACC_CR = DACC_CR_SWRST ;  // reset DAC

  DACC->DACC_MR =
    DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (2) |  // trigger 1 = TIO output of TC0
    //(1 << DACC_MR_USER_SEL_Pos) |  // select channel 1
    DACC_MR_MAXS |
    DACC_MR_REFRESH (0x0F) |       // bit of a guess... I'm assuming refresh not needed at 48kHz
    DACC_MR_TAG_EN  |             // Use Tag Ch#
    (2 << DACC_MR_STARTUP_Pos) ;  // 24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz

  DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH1
    | DACC_CHER_CH0; // enable chan 0 and 1

  //Enable PDC and setup buffers
  DACC->DACC_IER |= DACC_IER_ENDTX;     //INTR endtx
  DACC->DACC_TPR=(uint32_t)Tx0BufferDMA;      // DMA buffer
  DACC->DACC_TCR=BUFFERSIZE;
  DACC->DACC_TNPR=(uint32_t)Tx0BufferDMA;     // next DMA buffer
  DMA_TX_Memory = 0;
  DACC->DACC_TNCR=BUFFERSIZE;
  DACC->DACC_PTCR |= DACC_PTCR_TXTEN;   //Enable Transfer Control

  Serial.print("DACC_MR= "); Serial.println(DACC->DACC_MR, HEX);
}

void DACC_Handler() {   //DACC_ISR_HANDLER
  if (DMA_TX_Memory == 0){
    DACC->DACC_TNPR = (uint32_t)Tx0BufferDMA;
    DMA_TX_Memory = 1;
  }
  else {
    DACC->DACC_TNPR = (uint32_t)Tx1BufferDMA;
    DMA_TX_Memory = 0;
  }

  DACC->DACC_TNCR = BUFFERSIZE;

  digitalWrite(31, l1=!l1);       //no loss of speed??
}

void dac_write (int val)
{
  DACC->DACC_CDR = val & 0xFFF ;
  //DACC->DACC_CDR = sineWave[ix];
  //analogWrite(DAC1, sineWave[ix]>>4);
  //ix = (ix+1) % nPoints;
}

void adc_setup ()
{
  NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
  ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
  //ADC->ADC_IER = 0x1001 ;         // enable EOC intr on AD7, A12
  //ADC->ADC_IER = 0x0001 ;         // enable EOC intr on AD7, A06
  ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
  return;			// LCD ok, Touch ok

  ADC->ADC_CHER = 0x0003 ;    // enable just A0, A1, this will mess up Touch
  ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
  ADC->ADC_COR = 0x00000000 ;   // All offsets off
  //return;			// LCD ok, but spurious Touch

  // 1 = trig source TIO from TC0, 2 = TC1, 3=TC2 
  ADC->ADC_MR = 0x12110000 | (1 << 1) | ADC_MR_TRGEN;
  ADC->ADC_EMR = ADC_EMR_TAG;
  //Enable PDC and setup buffers
  ADC->ADC_IER |=1<<27;           //INTR endrx
  ADC->ADC_RPR=(uint32_t)Rx0BufferDMA;  // DMA buffer
  ADC->ADC_RCR=BUFFERSIZE;
  ADC->ADC_RNPR=(uint32_t)Rx1BufferDMA; // next DMA buffer
  ADC->ADC_RNCR=BUFFERSIZE;
  DMA_RX_Memory = 0;
  //return;			// Touch dead, no spurious
  ADC->ADC_PTCR=1;                //Enable Transfer Control
  
  Serial.print("ADC_MR= "); Serial.println(ADC->ADC_MR, HEX);
  //ADC->ADC_CR = 3;          //DONT DO THIS!
}

volatile int isr_count = 0 ;   // this was for debugging
void switchRxDMABuffer(){
    if (DMA_RX_Memory == 0){
      ADC->ADC_RNPR = (uint32_t)Rx0BufferDMA;
      DMA_RX_Memory = 1;
    }
    else {
      ADC->ADC_RNPR = (uint32_t)Rx1BufferDMA;
      DMA_RX_Memory = 0;
    }
}

void ADC_Handler ()
{
  if (ADC->ADC_ISR & ADC_ISR_EOC0)   // ensure there was an EOC and we read the ISR reg
  {
    uint32_t val;
    val = *(ADC->ADC_CDR+0) ;    // get conversion result
    //dac_write (0xFFF & ~val) ;   // copy inverted to DAC output FIFO
  }
 
  if (ADC->ADC_ISR &(1<<27)){     //Its ENDRX
    switchRxDMABuffer();
    DSP_Flag = 1;
    AGC_Flag = 1;
    ADC->ADC_RNCR=BUFFERSIZE;
    digitalWrite(30, l0 = !l0);
  } 
  
  isr_count ++ ;
}

void Audio_DMA_Start(void)
{
	Audio_DMA_Init();
}
#ifdef __cplusplus
}
#endif
