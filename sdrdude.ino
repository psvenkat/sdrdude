/*
 * The main code of the program
 *
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
#include <avr/pgmspace.h>
#include <malloc.h>
#include <stdlib.h>
#include <stdint.h> 

typedef bool _Bool;

// Includes ------------------------------------------------------------------

#include	"ChangeOver.h"
#include	"CW_Mod.h"
#include	"Encoders.h"
//**#include	"Init_Codec.h"
#include	"Init_DMA.h"
#include 	"Init_I2C.h"
//**#include	"Init_I2S.h"
#include	"PSKDet.h"
#include	"ModeSelect.h"
#include	"uart.h"
//**#include	"uart2.h"
//**#include 	"usb_conf.h"
//**#include	"usbh_hid_core.h"
//**#include	"usbh_usr.h"
//**#include 	"LcdDriver_ILI9320.h"

#include	"LcdHal.h"
#include	"TSHal.h"
#include	"screen_all.h"
#include	"options.h"
#include	"FrequencyManager.h"

#include	"sdr_image.h"

#include	"ScrollingTextBox.h"
#include	"xprintf.h"
#include	"yprintf.h"
#include	"Text_Enter.h"
#include	"AGC_Processing.h"
#include	"Band_Filter.h"
#include	"DSP_Processing.h"
#include  "DMA_IRQ_Handler.h"
#include	"widgets.h"
//**#include	"STM32-SDR-Subroutines.h"
//**#include	"Oscillator.h"
#include  "main.h"
//**#include	"ps2Keyboard_Input.h"
#include 	"Si570.h"
#include "SDR_Subroutines.h"

#define VERSION_STRING "2.001"

extern int NCO_Point;
extern int NCOTuneCount;
extern int NCO_Flag;

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart=(char *)0x20070000;
char *ramend=(char *)0x20088000;

const uint32_t CODEC_FREQUENCY = 8000;

const PROGMEM 
uint32_t fill[8000] = {0,0,0};

/**PSV
// USB structures (Must be 4-byte aligned if DMA active)
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
**/

/*
 * PROTOTYPES
 */
static void initializeHardware(void);
static void displaySplashScreen(void);
static void displaySerialPortWelcome(void);
static void main_delay(uint32_t numLoops);
static void displayLoadStationData(void);
static void showMemUsage();
char testMode = 'p';

/*
 * FUNCTIONS
 */

void xfunc_out(unsigned char ch) {
  Serial.write(ch);
}

//xdev_out(xf_out);

void setup()
{
  Serial.begin(115200);
  Serial.println("SDRDUE Version 1.1");

  //xdev_out(Serial.write());
  
  Serial.println("Mem 1");
    //String(pgm_read_word_near(fill+1))); 
    //String(fill[0]));
  showMemUsage();

	initializeHardware();

  //uint16_t x = pgm_read_word(fill[1]);
	/*
	 * Startup the GUI
	 */
  Serial.println("Mem 2");
  showMemUsage();
	Screen_CreateAllScreens();
  Serial.println("Mem 3");
  showMemUsage();
  
	if (!TS_IsCalibrated()) {
		Screen_ShowScreen(&g_screenCalibrate);
	} else {
		Screen_ShowMainScreen();
	}
   
 	InitTextDisplay();
  
	//send initialization to serial control port
//**	uart2_initSerial();

    showMemUsage();

}

void loop() {
		// Check if encoder-knobs have changed:
     
     Encoders_CalculateAndProcessChanges();

		if (NCO_Flag){
			if (--NCOTuneCount <= 0){
				NCO_Flag = 0;
				NCO_Frequency = (double)NCO_Point*15.625 +125.0;
				Acquire();
			}
		}

		// Check encoder 2 push button
		process_button();

		// Process any pending PS2 events
		//ps2Read();
//
//		// Process any pending USB events
//		USBH_Process(&USB_OTG_Core_dev, &USB_Host);
//
	  // Touch Events
    ProcessInputData();
//
		// Polling tasks:
		RxTx_CheckAndHandlePTT();
//
		// Redraw the screen (as needed)
		if (DSP_Flag == 1) {
      //GPIO_WriteBit(Test_GPIO, Test_1, Bit_SET);
//PSV			Process_All_DSP();
			UpdateScreenWithChanges();

      //GPIO_WriteBit(Test_GPIO, Test_1, Bit_RESET);
			DSP_Flag = 0;
		}

		if (AGC_Flag == 1) {
			Proc_AGC();
		}
		Screen_Update(); //see if button is pressed
    UpdateScreenWithChanges();  //Iterate thru all 
    //testChangeMode();
 
}

/**PSV
void testChangeMode(){
  char ch;
  if (Serial.available() > 0) {
    ch = Serial.read();
    if (ch != testMode){
      testMode = ch;
      switch (testMode) {
        case 'p':
          Screen_ShowScreen();
          break;
        case 's':
          Screen_ShowScreen();
          break;
        case 'c':
          Screen_ShowScreen();
          break;
      }
    }
  } 
}
**/
/**PSV
void Strobe_USB(void) {
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
}
**/
static void showMemUsage(){
  char *heapend=sbrk(0);
  register char * stack_ptr asm ("sp");
  struct mallinfo mi=mallinfo();

  Serial.print(F("Dynamic ram used: "));
  Serial.println(mi.uordblks);
  Serial.print(F("Program static ram used "));
  Serial.println(&_end - ramstart);
  Serial.print(F("Stack ram used ")); 
  Serial.println(ramend - stack_ptr);
  Serial.print(F("My guess at free mem: "));
  Serial.println(stack_ptr - heapend + mi.fordblks);
}

static void initializeHardware(void)
{
	debug(INIT, (const char*)F("initializeHardware:"));
	const int SETUP_DELAY = 100;
/**PSV  
	debug(INIT, "initializeHardware:uart_init\n");
	uart_init();
	debug(INIT, "initializeHardware:uart2_init\n");
	uart2_init();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:uart_init\n");
	displaySerialPortWelcome();
**/ 
	debug(INIT, (const char*)F("initializeHardware:LCD_Init\n"));
	GL_LCD_Init();
	main_delay(SETUP_DELAY);

	debug(INIT, (const char*)F("initializeHardware:I2C_GPIO_Init\n"));
	I2C_GPIO_Init();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:I2C_Cntrl_Init\n");
	I2C_Cntrl_Init();
	main_delay(SETUP_DELAY);
/**PSV  
	debug(INIT, "initializeHardware:Codec_AudioInterface_Init\n");
	Codec_AudioInterface_Init(CODEC_FREQUENCY);
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:No_Filt\n");
	No_Filt ();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:Audio_DMA_Init\n");
	Audio_DMA_Init();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:ResetModem\n");
	ResetModem(BPSK_MODE);
	main_delay(SETUP_DELAY);
**/  
	debug(INIT, "initializeHardware:displaySplashScreen\n");
	//displaySplashScreen();
	main_delay(SETUP_DELAY);
/**PSV
	debug(INIT, "initializeHardware:Si570_isEnabled\n");
	if (!Si570_isEnabled()) {
		main_delay(SETUP_DELAY);
		debug(INIT, "initializeHardware:Si570 Not found - Reinitializing\n");
			uart2_init();
			main_delay(SETUP_DELAY);
			debug(INIT, "initializeHardware:I2C_GPIO_Init\n");
			I2C_GPIO_Init();
			main_delay(SETUP_DELAY);
			debug(INIT, "initializeHardware:I2C_Cntrl_Init\n");
			I2C_Cntrl_Init();
			main_delay(SETUP_DELAY);
	}
**/
  
  if (Si570_isEnabled()) {
    debug(INIT, "initializeHardware:Si570_Init\n");
    Si570_Init();
    main_delay(SETUP_DELAY);
  }
  
	debug(INIT, "initializeHardware:SetRXFrequency\n");
	SetRXFrequency(1000);
	NCO_Point = (int)((1000./15.625)+.5);

	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:SetAFCLimit\n");
	SetAFCLimit(1000);
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:Options_Initialize\n");
	Options_Initialize();
	main_delay(SETUP_DELAY);

	debug(INIT, "initializeHardware:FrequencyManager_Initialize\n");
	FrequencyManager_Initialize();
	main_delay(SETUP_DELAY);

  showMemUsage();
  
	debug(INIT, "initializeHardware:Encoders_Init\n");
	Encoders_Init();
	main_delay(SETUP_DELAY);
/**PSV
	debug(INIT, "initializeHardware:TS_Initialize\n");
	TS_Initialize();
	main_delay(SETUP_DELAY);
  **/
	debug(INIT, "initializeHardware:Text_Initialize\n");
	Text_Initialize();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:displayLoadStationData\n");
	//Load stored macro data
	displayLoadStationData();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:Init_AGC\n");
	Init_AGC ();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:RxTx_Init\n");
	RxTx_Init();
	main_delay(SETUP_DELAY);
  debug(INIT, "initializeHardware:Mode_Init\n");
  Mode_Init();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:CW_Init\n");
	CW_Init();
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:Init_Waterfall\n");
	Init_Waterfall();
	main_delay(SETUP_DELAY);
/**PSV	
	debug(INIT, "initializeHardware:configureGPIOps2\n");
	configureGPIOps2();
	main_delay(SETUP_DELAY);

	debug(INIT, "initializeHardware:USBH_Init\n");

	 // Sampling rate interrupt timer
	 INTTIM_Config();

	 // PWM Configuration 
	 PWM_Config(256);
	 PWM_SetDC(128);

	// Init USB Host Library
	USBH_Init(
			&USB_OTG_Core_dev, // Core handle
		#ifdef USE_USB_OTG_FS
			USB_OTG_FS_CORE_ID,
		#else
			USB_OTG_HS_CORE_ID, // core ID
		#endif
			&USB_Host,         // defined in /main.c (above)
			&HID_cb,           // In /drv/src/usbh_hid.core.c (class callback)
			&USR_Callbacks     // In /usbh_usr.c (user callback)
	);
  **/
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:init_DSP\n");
	init_DSP();
  
	main_delay(SETUP_DELAY);
	debug(INIT, "initializeHardware:Audio_DMA_Start\n");
  //Get everything up and running before starting DMA Interrupt
  // Need 8K mem for this
	Audio_DMA_Start();			
	main_delay(SETUP_DELAY);
  showMemUsage();
  
	debug(INIT, "initializeHardware:GPIO_BandFilterInit\n");
	GPIO_BandFilterInit();

	int newFreq = FrequencyManager_GetCurrentFrequency();
	FrequencyManager_SetCurrentFrequency(newFreq);
	FrequencyManager_Check_FilterBand(newFreq);
	main_delay(SETUP_DELAY);
}

/** PSV not used
static void displaySplashScreen(void)
{
	GL_DrawBMP16Bit(0,0, gimp_image.height, gimp_image.width, (uint16_t*) gimp_image.pixel_data, 0);
	GL_SetTextColor(GL_BLACK);
	GL_SetBackColor(GL_WHITE);
	GL_PrintString(0, 0, "STM32 SDR V" VERSION_STRING, 0);
	GL_PrintString(230, 0, __DATE__, 0);
	GL_PrintString(230, 220, __TIME__, 0);
	GL_PrintString(0, 220, "Press both encoder to reset touchscreen.", 0);
	main_delay(10000000);
}

static void displaySerialPortWelcome(void)
{
	xprintf("...%c[2J%c[;H",27,27); // clear screen
	xprintf("STM32-SDR Version %s.\n", VERSION_STRING);
	xprintf("Compiled on %s at %s.\n", __DATE__, __TIME__);
	xprintf("Serial port at 115,200 baud, 8 bit, no parity, 1 stop-bit.\n");
	xprintf("\n");
}
*/

static void displayLoadStationData(void)
{
	GL_SetTextColor(GL_RED);
	GL_SetBackColor(GL_WHITE);
	GL_PrintString(0, 200, "Loading...", 0);
}

static void main_delay(uint32_t numLoops)
{
	volatile uint32_t i, j;

	for (i = 0; i < numLoops; i++) {
		j++;
	}
}

