/*
 * GcFunctionsV1.h
 *
 *  Created on: 26 Jan 2021
 *  Last edited 15JUL2024
 *      Author: Geoff
 */

#ifndef INC_GCFUNCTIONSV1_H_
#define INC_GCFUNCTIONSV1_H_

#include "stm32l4xx_hal.h"  //this needs to be included for the compiler to understand what a uint8_t data type is...




uint32_t ascciichartohexnib(uint8_t charval);

uint8_t GetProcess(void);
void SetProcess(uint8_t val);

uint16_t GetProcessPhase(void); //1AUG2023: updated to return 16bit value rather than 8 bit
void SetProcessPhase(uint16_t phase); //1AUG2023: updated to be set with a 16bit value rather than 8bit

//function prototypes

//uint32_t ReadDelayVal(void); //getter function
uint32_t GetDelayValue(void);
//void SetDelayCount(uint32_t time); //setter function
void SetDelayValue(uint32_t time); //set timer value and continue

void Delay(uint32_t time); //set and wait specified time value
void DecrementDelayTime(void); //executed by TIM2 ISR


void SetIhStartAddress (uint16_t address);
void SetIhQty (uint16_t qty);
uint32_t IntelHexOutput(uint16_t Addr, uint16_t Qty);
void prepareIntelHexLine(uint16_t address, uint8_t bytecount, uint8_t* ptr, uint8_t ptroffset);

void TestFunction (uint8_t value);


void DispErrSlowLedMessage(void);
void hex16bit2fivedigitdec(uint8_t* pdecString, uint16_t input);
void hex2dec(uint8_t* pdecString, uint8_t input);
uint8_t Max7221DigitDecoder(uint8_t DigitValue);
void SpiOutput16bits(void);

uint8_t gcstrlen(uint8_t* str);
uint8_t gcstrcmp(uint8_t* str1, uint8_t* str2);
uint8_t loadconststring(uint8_t* deststring, uint8_t* str);
uint8_t gcstrcmplength(uint8_t* str1, uint8_t* str2, uint8_t length);
uint8_t gccopycompletestring(uint8_t qty, uint8_t* cmd_string, uint8_t* rx_buffer2);
//uint32_t ExtractValueFromString(uint8_t* cmd_String, uint8_t charOffset, uint8_t charqty);
uint32_t ExtractValueFromString(char* cmd_String, uint8_t charOffset, uint8_t charqty);
uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
uint8_t IsNumericChar(uint8_t character);

#endif /* INC_GCFUNCTIONSV1_H_ */
