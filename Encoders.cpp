/*
 * Code to handle the 2 encoder knobs
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

//**PSV**	#include	"stm32f4xx_rcc.h"
//**PSV**	#include	"stm32f4xx_exti.h"
//**PSV**	#include	"stm32f4xx_syscfg.h"
//**PSV**	#include 	"stm32f4xx_gpio.h"
//**PSV**	#include	"stm32f4xx_tim.h"
//#include	"arm_math.h"
//#include	"Si570.h"
//#include	"Init_I2C.h"
//#include	"Init_Codec.h"
#include 	<Wire.h>
#include	"Encoders.h"
#include	"FrequencyManager.h"
#include	"options.h"
//#include	"LcdHal.h"
//#include	"ChangeOver.h"
#include	"xprintf.h"
#include	"screen_all.h"
//#include	"main.h"

#define PIN1A	25
#define PIN1B	26
#define PIN1BUT	24
#define READ1_BOTH	0x03
#define READ1_A	0x01
#define READ1_B	0x02

#define PIN2A	27
#define PIN2B	28
#define PIN2BUT	23
#define READ2_BOTH	0x0C
#define READ2_A	0x04
#define READ2_B	0x08

#define deBounceMills	20

extern int 	NCOTUNE;

typedef struct
{
	uint16_t old;
	int8_t direction;
	int8_t change;
	uint8_t pinA, pinB, pinButton;
	byte regData;
	uint8_t aFlag, bFlag;
	int encoderPos;
	_Bool buttonPressed;
} EncoderStruct_t;

volatile EncoderStruct_t s_encoderOption = {0, 0, 0, PIN1A, PIN1B, 
	PIN1BUT, 0, 0, 0, 0, false}; //PD0, PD1, PA15
volatile EncoderStruct_t s_encoderFreq = {0, 0, 0, PIN2A, PIN2B, 
	PIN2BUT, 0, 0, 0, 0, false}; //PD2, PD3, PA14
unsigned long optionPressTime, freqPressTime;

// Encoder used for options =0 or filter code selection = 1

// Prototypes
static void init_encoderOption(void);
static void init_encoderFreq(void);
static void applyEncoderChangeOption(int8_t changeDirection);
static void applyEncoderChangeFreq(int8_t changeDirection);

/* ----------------------
 * Initialization
 * ---------------------- */
void Encoders_Init(void)
{
	init_encoderOption();
	init_encoderFreq();  //this may be required to introduce a delay for start up
	FrequencyManager_StepFrequencyDown();
	FrequencyManager_StepFrequencyUp();
}

void ISROptionA()
{
	// read all pin values then strip away all but pinA and pinB's values 
	s_encoderOption.regData = REG_PIOD_PDSR & READ1_BOTH; 
	//check that we have both pins at detent (HIGH) and that we are 
	// expecting detent on this pin's rising edge
	if(s_encoderOption.regData == READ1_BOTH && s_encoderOption.aFlag) { 
		s_encoderOption.encoderPos--; //decrement the encoder's position count
		s_encoderOption.bFlag = 0; //reset flags for the next turn
		s_encoderOption.aFlag = 0; //reset flags for the next turn
		s_encoderOption.direction = -1;
		s_encoderOption.change--;
	}
	else if (s_encoderOption.regData == READ1_A) 
		s_encoderOption.bFlag = 1; //signal that we're expecting pinB to 
				// signal the transition to detent from free rotation

	//debug(ENCODER, "ISROptionA %d %d\n", s_encoderOption.regData, s_encoderOption.encoderPos);
/**PSV
	if (count%100 == 0 && !Si570_isEnabled()) // every 0.1s if using serial control
				FrequencyManager_ControlCurrentFrequency();
**/
}

void ISROptionB(){
	// read all pin values then strip away all but pinA and pinB's values 
	s_encoderOption.regData = REG_PIOD_PDSR & READ1_BOTH; 
	//check that we have both pins at detent (HIGH) and that we are 
	// expecting detent on this pin's rising edge
	if(s_encoderOption.regData == READ1_BOTH && s_encoderOption.bFlag) { 
		s_encoderOption.encoderPos++; //decrement the encoder's position count
		s_encoderOption.bFlag = 0; //reset flags for the next turn
		s_encoderOption.aFlag = 0; //reset flags for the next turn
		s_encoderOption.direction = 1;
		s_encoderOption.change++;
	}
	else if (s_encoderOption.regData == READ1_B) 
		s_encoderOption.aFlag = 1; //signal that we're expecting pinB to 
				// signal the transition to detent from free rotation

}

void ISROptionButt(){
  unsigned long t = millis();
  if (t - optionPressTime > 20){
    s_encoderOption.buttonPressed = true;
  }
  optionPressTime = t;
}

void ISRFreqA(){
	// read all pin values then strip away all but pinA and pinB's values 
	s_encoderFreq.regData = REG_PIOD_PDSR & READ2_BOTH; 
	//check that we have both pins at detent (HIGH) and that we are 
	// expecting detent on this pin's rising edge
	if(s_encoderFreq.regData == READ2_BOTH && s_encoderFreq.aFlag) { 
		s_encoderFreq.encoderPos--; //decrement the encoder's position count
		s_encoderFreq.bFlag = 0; //reset flags for the next turn
		s_encoderFreq.aFlag = 0; //reset flags for the next turn
		s_encoderFreq.direction = -1;
		s_encoderFreq.change--;
	}
	else if (s_encoderFreq.regData == READ2_A) 
		s_encoderFreq.bFlag = 1; //signal that we're expecting pinB to 
				// signal the transition to detent from free rotation
}

void ISRFreqB(){
	// read all pin values then strip away all but pinA and pinB's values 
	s_encoderFreq.regData = REG_PIOD_PDSR & READ2_BOTH; 
	//check that we have both pins at detent (HIGH) and that we are 
	// expecting detent on this pin's rising edge
	if(s_encoderFreq.regData == READ2_BOTH && s_encoderFreq.bFlag) { 
		s_encoderFreq.encoderPos++; //decrement the encoder's position count
		s_encoderFreq.bFlag = 0; //reset flags for the next turn
		s_encoderFreq.aFlag = 0; //reset flags for the next turn
		s_encoderFreq.direction = 1;
		s_encoderFreq.change++;
	}
	else if (s_encoderFreq.regData == READ2_B) 
		s_encoderFreq.aFlag = 1; //signal that we're expecting pinB to 
				// signal the transition to detent from free rotation
}

void ISRFreqButt() {
  unsigned long t = millis();
  if (t - freqPressTime > 20){
    s_encoderFreq.buttonPressed = true;
  }
  freqPressTime = t;
}

void init_encoderOption()
{
  pinMode(PIN1A, INPUT_PULLUP); 
  pinMode(PIN1B, INPUT_PULLUP); 
  // set an interrupt on PinA, looking for a rising edge signal and executing the 	"ISROptionA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(PIN1A),ISROptionA,RISING); 
  // set an interrupt on PinB, looking for a rising edge signal and executing the 	"ISROptionB" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(PIN1B),ISROptionB,RISING);   
  // Rotary encoder button section of setup
  pinMode (PIN1BUT, INPUT_PULLUP); // setup the button pin
  optionPressTime = 0;
  attachInterrupt(digitalPinToInterrupt(PIN1BUT),
      ISROptionButt,RISING);
}

void init_encoderFreq()
{
  pinMode(PIN2A, INPUT_PULLUP); 
  pinMode(PIN2B, INPUT_PULLUP); 
  // set an interrupt on PinA, looking for a rising edge signal and executing the 	"ISROptionA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(PIN2A),ISRFreqA,RISING); 
  // set an interrupt on PinB, looking for a rising edge signal and executing the 	"ISROptionB" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(PIN2B),ISRFreqB,RISING);   
  // Rotary encoder button section of setup
  pinMode (PIN2BUT, INPUT_PULLUP); // setup the button pin
  freqPressTime = 0;
  attachInterrupt(digitalPinToInterrupt(PIN2BUT),
      ISRFreqButt,RISING);
  // TODO:- Is there a need to do this here?
  //RxTx_SetReceive();
}


// Return if one (or both) encoders pressed.
_Bool Encoders_IsOptionsEncoderPressed(void)
{
  _Bool t = s_encoderOption.buttonPressed;
  s_encoderOption.buttonPressed = false;
  return t;
}
_Bool Encoders_IsFrequencyEncoderPressed(void)
{
  _Bool t = s_encoderFreq.buttonPressed;
  s_encoderFreq.buttonPressed = false;
  return t;
}
_Bool Encoders_AreBothEncodersPressed(void)
{

//	return Encoders_IsOptionsEncoderPressed()
//			&& Encoders_IsFrequencyEncoderPressed();
  return false;
}

/*
 * Process Encoder Changes
 */
void Encoders_CalculateAndProcessChanges(void){
	//debug(GUI, "ProcessChange %d %d %d\n", 
	//s_encoderOption.change, s_encoderOption.direction,
	//s_encoderOption.encoderPos);
	if (s_encoderOption.change > 0){
		applyEncoderChangeOption(1);
		s_encoderOption.change--;
	}
	if (s_encoderOption.change < 0){
		applyEncoderChangeOption(-1);
		s_encoderOption.change++;
	}
	if (s_encoderFreq.change > 0){
		applyEncoderChangeFreq(1);
		s_encoderFreq.change--;
	}
	if (s_encoderFreq.change < 0){
		applyEncoderChangeFreq(-1);
		s_encoderFreq.change++;
	}
}

static void applyEncoderChangeFreq(int8_t changeDirection)
{
	// Check for no change
	if (changeDirection == 0) {
		return;
	}

	debug(INIT, "EncoderChangeFreq %d \n", changeDirection);
	// Are we pressed in?
	if (Encoders_IsFrequencyEncoderPressed()) {
		debug(INIT, "EncoderChangeFreq stepSize \n");
		if (changeDirection > 0) {
			FrequencyManager_DecreaseFreqStepSize();
		} else {
			FrequencyManager_IncreaseFreqStepSize();
		}
	}
	else {

		if (NCOTUNE){
			if (changeDirection > 0) {
				Tune_NCO_Up( );
			} else {
				Tune_NCO_Down();
			}
		}
		else {
			if (changeDirection > 0) {
				FrequencyManager_StepFrequencyUp();
			} else {
				FrequencyManager_StepFrequencyDown();
			}
		}
	}

	// Filter screen requires an update band frequency
	if (Screen_GetScreenMode() == FILTER){
		int band = Screen_GetFilterFreqID();
		uint32_t currentFrequency = FrequencyManager_GetCurrentFrequency();
		FrequencyManager_SetBandFreqFilter (band, currentFrequency);
	}

}
static void applyEncoderChangeOption(int8_t changeDirection)
{
	int curOptIdx;
	int16_t currentValue;
	int16_t newValue=0;
	int16_t minValue;
	int16_t maxValue;
	static int loopCount = 0;

	// Check for no change
	if (changeDirection == 0) {
		return;
	}

	debug(INIT, "EncoderChangeOption %d \n", changeDirection);
	// Are we setting general options or the band filter code values
	// Options uses an array of min/max values
	// Filter sets a fixed 3-bit binary code
	if (Screen_GetScreenMode() != FILTER){
		/*
		 * Check the limits
		 */
		curOptIdx = Options_GetSelectedOption();
		currentValue = Options_GetValue(curOptIdx);
		int changeRate = Options_GetChangeRate(curOptIdx);
		if (changeRate == 0) {
			loopCount++;
			if (loopCount%4 == 0){
				newValue = currentValue + changeDirection;
			} else {
				newValue = currentValue;
			}
		} else
			newValue = currentValue + Options_GetChangeRate(curOptIdx) * changeDirection;
		minValue = Options_GetMinimum(curOptIdx);
		maxValue = Options_GetMaximum(curOptIdx);
		if (newValue < minValue) {
			newValue = minValue;
		}
		if (newValue > maxValue) {
			newValue = maxValue;
		}
		// Set the value & Display it

		Options_SetValue(curOptIdx, newValue);
	} else {
		int band = Screen_GetFilterCodeID();
		currentValue = FrequencyManager_GetFilterCode (band);
		int changeRate = BAND_FILTER_CHANGE_RATE;
		newValue = currentValue + changeRate * changeDirection;
		minValue = BAND_FILTER_MINIMUM;
		maxValue = BAND_FILTER_MAXIMUM;
		if (newValue < minValue) {
			newValue = minValue;
		}
		if (newValue > maxValue) {
			newValue = maxValue;
		}

		// Set the value & Display it
		FrequencyManager_SetBandCodeFilter (band, newValue);
	}
}


