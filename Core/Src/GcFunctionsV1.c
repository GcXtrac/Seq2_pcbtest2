/*
 * GcFunctions.c
 *
 *  Created on: 26 Jan 2021
 *  Last edited 1AUG2023
 *      Author: Geoff
 */

//#define SpiEnabled

#include <main.h> //needed to allow references to I/O pins
#include <GcFunctionsV1.h>
//#include "SerialComms_V1.h"
#include <stdio.h>	//used for sprintf() function
#include <string.h>  //used for strlen(), strcat() function
#include "global-settingsV1.h" //used for I2C memory settings
#include "GcI2cV1.h"

#define GcFunctions

//extern SPI_HandleTypeDef hspi2; //pulled from main.c
extern uint8_t* pSpiTxData;		//pulled from main.c
extern uint8_t* pSpiRxData;		//pulled from main.c



static uint8_t phase = 0;;
static uint16_t IhStartAddress = 0;
static uint16_t IhAddressPtr = 0;
static uint16_t Ihremainingbytes = 0;

static uint32_t DelayVar = 0;
static uint16_t ProcessPhase = 0; //updated from 8bit to 16bit value
static uint8_t Process = 0;


uint32_t ascciichartohexnib(uint8_t charval)
{
	//Created 13DEC2022
	//Returns a 32bit value
	//bits 31-24: error value:
	//	0: No conversion issues
	//	1: 0x39 < character value < 0x41
	//	2: character value > 0x46
	//	4: character value < 0x30
	//bits 0-3: output nibble

	uint8_t temp = 0;
	uint32_t ErrVal = 0;

	if (charval >= 0x30)
	{
		if (charval <= 0x39)
		{
			temp = charval - 0x30;
		}
		else if (charval >= 0x41)
		{
			if (charval <= 0x46)
			{
				temp = charval - 0x41 + 0x0A;
			}
			else
			{
				//above 0x46
				ErrVal = ErrVal & 0x02;
			}
		}
		else
		{
			//between 0x39 & 0x41
			ErrVal = ErrVal & 0x01;
		}
	}
	else
	{
		//less than 0x30
		ErrVal = ErrVal & 0x04;
	}
	return (ErrVal << 24) | temp;
}


uint8_t GetProcess(void)
{
	//Created 14MAR2023
	return Process;
}


void SetProcess(uint8_t val)
{
	//Created 14MAR2023
	Process = val;
	SetProcessPhase(0);
}


uint16_t GetProcessPhase(void)
{
	//Created 5DEC2022
	//Last edited 1AUG2023
	return ProcessPhase;
}


void SetProcessPhase(uint16_t phase)
{
	//Created 5DEC2022
	ProcessPhase = phase;
}



//uint32_t ReadDelayVal(void)
uint32_t GetDelayValue(void)
{
	//'Getter' function
	//Created 15DEC2022
	return DelayVar;
}

//void SetDelayCount(uint32_t time)
void SetDelayValue(uint32_t time)
{
	//Created 16DEC2022
	//Last edited 1AUG2023
	//value is decremented by TIM2 ISR
	DelayVar = time;
}

void Delay(uint32_t time)
{
	//Created 13DEC2022
	DelayVar = time;
	while (DelayVar != 0)
	{
	}
}

void DecrementDelayTime(void)
{
	//Function called from TIM2 ISR
	//Created 13DEC2022
	if (DelayVar != 0)
	{
		DelayVar--;
	}
}


void TestFunction (uint8_t value)
{
	//Created 26JAN2021
	value = value + 8;
}




void SetIhStartAddress (uint16_t address)
{
	//Created 5DEC2022
	//Used to set start address for Intel Hex output
	IhStartAddress = address;
}

void SetIhQty (uint16_t qty)
{
	//Function is designed to hold quantity of Intel hex data to be output
	//Created 5DEC2022
	Ihremainingbytes = qty; //this value will decrement with each block of data read from I2C memory
}


uint32_t IntelHexOutput(uint16_t Addr, uint16_t Qty)
{
	//reads attached I2C memory device and outputs Intel Hex data
	//Used for data backup
	//Use serial command "IH", with support from commands: "Axxxx", "Qxxxx"
	//initially setup for 24LC32 device (4k x 8bit), 32byte page
	//this needs to broken into a a multi-step process called from main loop to prevent watchdog timer reset!
	//Process 0x01
	//Created 5DEC2022
	//Last edited 28JUN2023
	//Input parameters:
	//	addr: define start address
	//	Qty: define total data size to be read
	//Output:
	//	0: No erros
	//	1: There was a problem reading from the I2C memory
	//	2: Intel hex read complete

	//uint8_t ReadSmallI2CDatablock(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty)
	//const uint8_t I2CREADBLOCKSIZE = 0x20;
	//const uint8_t MAXOUTPUTDATALENGTH = 16; //specified maximum data length per output line
	uint32_t ErrVal = 0;
	uint8_t ReturnVal = 0;

	char msg[100];

	phase = GetProcessPhase();


	if (phase == 0)
	{
		//start of process
		SetIhStartAddress(Addr);
		SetIhQty(Qty);

		sprintf(msg, "Intel Hex output, (start:0x%04X, qty:0x%04X)\r\n", IhStartAddress, Ihremainingbytes);
		SendSerial(msg);
		ReturnVal = 0;
		phase++; //increment process phase

		SetProcessPhase(phase);

		IhAddressPtr = 0;
	}

	else if (phase == 1)
	{

		//Processphase (128 x 0x20h blocks = 4096 bytes)
		uint8_t device = SEQUENCERMEMORY;
		//uint8_t BlockQty = 0x20;
		uint8_t ReadI2cData[I2CREADBLOCKSIZE];
		uint8_t* pData;
		pData = &ReadI2cData[0];
		//uint16_t address = (phase - 1) * 0x20;
		uint8_t bytecount = 0;
		//uint8_t recordtype = 0;
		//uint8_t checksum = 0;
		uint8_t linedata = 0;
		uint8_t opDataptr = 0;


		if (Ihremainingbytes > I2CREADBLOCKSIZE)
		{
			Ihremainingbytes = Ihremainingbytes - I2CREADBLOCKSIZE;
			bytecount = I2CREADBLOCKSIZE;
		}
		else
		{
			bytecount = Ihremainingbytes;
			Ihremainingbytes = 0;
		}



		if (bytecount == 0)
		{
			//All specified data has been read
			//terminate Intel hex block
			//recordtype = 1;
			//prepareIntelHexLine(recordtype, 0, MAXOUTPUTDATALENGTH, 0, opDataptr);
			prepareIntelHexLine(0, 0, pData, 0);

			ReturnVal = 2; //indicate to main loop that Intel hex processing was completed
		}
		else
		{
			//process data
			//recordtype = 0;

			ErrVal = ReadSmallI2CDatablock(device, pData, IhStartAddress + IhAddressPtr, bytecount, 0); //Read block of data from I2C device
			if (ErrVal == 0)
			{
				linedata = bytecount;
				opDataptr = 0;
				while (opDataptr < bytecount)
				{
					if (linedata > MAXOUTPUTDATALENGTH)
					{
						//current data quantity exceeds output line data quantity setting
						prepareIntelHexLine(IhStartAddress + IhAddressPtr + opDataptr, MAXOUTPUTDATALENGTH, pData, opDataptr);
						linedata = linedata - MAXOUTPUTDATALENGTH; //calculate remaining bytes to output
						opDataptr = opDataptr + MAXOUTPUTDATALENGTH;
					}
					else
					{
						//current data falls below maximum output data qty setting
						prepareIntelHexLine(IhStartAddress + IhAddressPtr + opDataptr, linedata, pData, opDataptr);
						opDataptr = opDataptr + linedata;

					}
				}
				ReturnVal = 0;

				IhAddressPtr = IhAddressPtr + bytecount;
			}
			else
			{
				sprintf(msg, "\r\nThere was a problem processing IH data!\r\n");
				SendSerial(msg);
				ReturnVal = 1; //indicate to main loop that a problem was encountered
			}
		}

	}

	return ReturnVal;
}


void prepareIntelHexLine(uint16_t address, uint8_t bytecount, uint8_t* ptr, uint8_t ptroffset)
{
	//Created 5DEC2022
	//Last edited 15DEC2022

	uint8_t checksum = 0;
	uint8_t data = 0;

	char msg[100];
	char msgtemp[100];
	uint8_t recordtype = 0;

	checksum = 0;
	//block of data successfully read from i2C device
	sprintf(msgtemp, ":"); //number of bytes per record
	strcpy(msg, msgtemp);
	sprintf(msgtemp, "%02X", bytecount); //address
	checksum = checksum + bytecount;
	strcat(msg, msgtemp);
	sprintf(msgtemp, "%04X", address); //address
	strcat(msg, msgtemp);
	checksum = checksum + (address >> 8) + address;
	recordtype = 0;
	if (bytecount == 0)
	{
		recordtype = 1;
	}
	sprintf(msgtemp, "%02X", recordtype); //record type
	checksum = checksum + recordtype;
	strcat(msg, msgtemp);

	if (bytecount != 0)
	{
		for(uint8_t i=0; i<bytecount; i++)
		{
			data = *(ptr + ptroffset + i);
			checksum = checksum + data;
			sprintf(msgtemp, "%02X", data); //record type
			strcat(msg, msgtemp); //add byte value characters to existing line
		}
	}
	//checksum = 256 - checksum;
	checksum = checksum ^ 0Xff; //invert bits
	checksum++;
	sprintf(msgtemp, "%02X\r\n", checksum); //record type
	strcat(msg, msgtemp);
	SendSerial(msg);
}


void DispErrSlowLedMessage(void)
{
	//Function designed to output a message to the LED display
	//Message: Err.Slow
	//Used to indicate that incoming frequency signal is too slow to measure
	//Created 4MAR2021


	//LED display error "Err.Slo"
	*(pSpiTxData) = 8;
	*(pSpiTxData+1) = 0x4F;
	SpiOutput16bits();

	*(pSpiTxData) = 7;
	*(pSpiTxData+1) = 0x05;
	SpiOutput16bits();

	*(pSpiTxData) = 6;
	*(pSpiTxData+1) = 0x05;
	SpiOutput16bits();

	*(pSpiTxData) = 5;
	*(pSpiTxData+1) = 0x80;
	SpiOutput16bits();

	*(pSpiTxData) = 4;
	*(pSpiTxData+1) = 0x5B;
	SpiOutput16bits();

	*(pSpiTxData) = 3;
	*(pSpiTxData+1) = 0x0E;
	SpiOutput16bits();

	*(pSpiTxData) = 2;
	*(pSpiTxData+1) = 0x1D;
	SpiOutput16bits();
}
#ifdef SpiEnabled
	void SpiOutput16bits(void)
	{
		//Function outputs 16 SPi bits to peripheral.
		//data word is already held in SpiTxData buffer
		//Created 3FEB2021

		//HAL_GPIO_WritePin(Spi2_Cs_GPIO_Port, Spi2_Cs_Pin, GPIO_PIN_RESET); //pull CS low

		//HAL_Status_TypeDef HAL_SPI_TransmitReceive(SPI_Handle_TypeDef* hspi,
		//												uint8_t* pTxData,
		//												uint8_t* pRxdata,
		//												uint16_t size,
		//												uint32_t Timeout);
		//HAL_SPI_TransmitReceive(&hspi2, pSpiTxData, pSpiRxData, 1, 100);
		HAL_SPI_TransmitReceive(&hspi2, pSpiTxData, pSpiRxData, 2, 100);
		//(*pSpiTxData)++;

		//HAL_GPIO_WritePin(Spi2_Cs_GPIO_Port, Spi2_Cs_Pin, GPIO_PIN_SET); //pull CS high
	}
#endif

uint8_t Max7221DigitDecoder(uint8_t DigitValue)
{
	//Created 3FEB2021
	//Last edited 14MAY2021
	//Function takes a hex nibble and converts into 7segment pattern
	//data -> seven segment:
	//D0-g
	//D1-f
	//D2-e
	//D3-d
	//D4-c
	//D5-b
	//D6-a
	//D7-DP

	const uint8_t DigitPatterns[16] = {0x7e, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47};
	const uint8_t* pDigitPattern;
	pDigitPattern = DigitPatterns; //don't need the ampersand as pointer assumes an address

	//debugging
	//int temp = DigitValue & 0x0F;
	//temp = *(pDigitPattern + (DigitValue & 0x0F));
	return *(pDigitPattern + (DigitValue & 0x0F));
}

//test with 1377d or 0x561
void hex16bit2fivedigitdec(uint8_t* pdecString, uint16_t input)
{
	//function converts a binary value into an array of decimal digits
	//Created 3FEB2021
	//Last edited 8FEB2022
	//The first elemement of the array pointed to be pDecString holds the least significant digit
	uint8_t bitcounter = 16;	//set maximum number of bits to process
	uint8_t digitcount = 5;
	uint8_t Carry = 0;
	uint16_t mask = 1;
	mask = mask << (bitcounter -1);

	//clear decimal digits array
	for (int i=0; i<digitcount; i++)
	{
		*(pdecString + i) = 0; //clear individual digit values
	}


	//shift bits left
	while(bitcounter > 0)
	{
		if ((input & mask) != 0)
		{
			Carry = 1;
		}
		input = input << 1;


		for (int i = 0; i<digitcount; i++)
		{
			*(pdecString + i)= *(pdecString + i) << 1;


			if (Carry != 0)
			{
				*(pdecString + i) = *(pdecString + i) | 1;
				Carry = 0;
			}

			if (*(pdecString + i) > 9)
			{
				*(pdecString + i) = *(pdecString + i) - 10;
				Carry = 1;
			}

		}
		bitcounter--;
	}
}


void hex2dec(uint8_t* pdecString, uint8_t input)
{
	//function converts a binary value into an array of decimal digits
	//Created 3FEB2021
	uint8_t bitcounter = 8;	//set maximum number of bits to process
	uint8_t digitcount = 3;
	uint8_t Carry = 0;

	//clear decimal digits array
	for (int i=0; i<digitcount; i++)
	{
		*(pdecString+i) = 0;
	}


	//shift bits left
	for (bitcounter=0; bitcounter<8; bitcounter++)
	{
		if ((input & 0x80) != 0)
		{
			Carry = 1;
		}
		input = input << 1;


		for (int i = 0; i<digitcount; i++)
		{
			*(pdecString+i)= *(pdecString+i)<< 1;


			if (Carry != 0)
			{
				*(pdecString+i) = *(pdecString+i) | 1;
				Carry = 0;
			}

			if (*(pdecString+i) > 9)
			{
				*(pdecString+i) = *(pdecString+i) - 10;
				Carry = 1;
			}

		}
	}
}


uint8_t gcstrlen(uint8_t* str)
{
	//Function used to measure length of a test string
	//Created 26JAN2021
	uint8_t charcount = 0;
	uint8_t* charptr;
	charptr = str;

	for (charcount = 0; charcount < 100; charcount++)
	{
		if (*charptr++ == 0)
		{
			break;
		}
	}
	return charcount;
}


uint8_t gcstrcmp(uint8_t* str1, uint8_t* str2)
{
	//Function used to compare two text string both terminated with a null character
	//Created 26JAN2021
	//1) check that strings are the same length
	uint8_t stringlength1 = 0;
	uint8_t stringlength2 = 0;
	uint8_t returnval = 0;
	uint8_t charcount = 0;
	uint8_t* charptr1 = str1;
	uint8_t* charptr2 = str2;

	stringlength1 = gcstrlen(str1);
	stringlength2 = gcstrlen(str2);
	if (stringlength1 == stringlength2)
	{
		for (charcount = 0; charcount < stringlength1; charcount++)
		{
			if (*charptr1++ != *charptr2++)
			{
				returnval = -2;
				break;
			}
		}
	}
	else
	{
		returnval = -1;
	}
	return returnval;

}

uint8_t loadconststring(uint8_t* deststring, uint8_t* str)
{
	//Function loads a constant text string into a character array (or string) variable
	//Created 26JAN2021
	uint8_t* sourcecharptr = str;
	uint8_t* destcharptr = deststring;
	uint8_t charcount = 0;

	for (charcount=0; charcount<100; charcount++)
	{
		if (*sourcecharptr != 0)
		{
			*destcharptr++ = *sourcecharptr++;
		}
		else
		{
			break;
		}
	}
	return charcount;
}


uint8_t gcstrcmplength(uint8_t* str1, uint8_t* str2, uint8_t length)
{
	//Function used to compare two text string both terminated with a null character
	//The length defines the number of characters to compare in each string
	//Created 26JAN2021
	//Last edited 7OCT2021
	//returns:
	//	0: strings were the same within specified length
	//	-1: string 1 length is less than specified test length
	//  -2: string 2 length is less then specified test length
	//	-4: strings differ within specified length


	uint8_t stringlength1 = 0;
	uint8_t stringlength2 = 0;
	uint8_t returnval = 0;
	uint8_t charcount = 0;
	uint8_t* charptr1 = str1;
	uint8_t* charptr2 = str2;

	//1) check that strings are at least as long as the specified length
	stringlength1 = gcstrlen(str1);
	stringlength2 = gcstrlen(str2);
	if (stringlength1 < length)
	{
		returnval = -1;
	}

	if (returnval == 0)
	{
		if (stringlength2 < length)
		{
			returnval = -2;
		}
	}


	if (returnval == 0)
	{
		for (charcount = 0; charcount < length; charcount++)
		{
			if (*charptr1++ != *charptr2++)
			{
				returnval = -4;
				break;
			}
		}
	}

	return returnval;

}

uint8_t gccopycompletestring(uint8_t qty, uint8_t* cmd_string, uint8_t* rx_buffer2)
{
	//function copies next complete string acroess to destination buffer
	//Created 26JAN2021

	return 1;
}

uint32_t ExtractValueFromString(char* cmd_String, uint8_t charOffset, uint8_t charqty)
//uint32_t ExtractValueFromString(uint8_t* cmd_String, uint8_t charOffset, uint8_t charqty)
{
	//This function is used to extract hex/numeric value from an ascii string.
	//Use this function to process up to 7 characters!
	//Created 7OCT2021
	//Last edited 10OCT2021
	//returns a single 32bit value:
	//	bit31: indicates a error was encountered
	//	bit30: indicates numeric values detected
	//	bit29: indicates hex values detected
	//	bit28: indicates too many characters were specified
	//	bit 0-27: 28bit extracted value returned
	uint8_t i = 0;
	uint8_t testchar = 0;
	uint8_t nibble = 0;
	uint32_t OutputVal = 0;
	uint32_t charstatus = 0; //this value should change to non zero for each character tested

	if (charqty < 8)
	{
		for (i=0; i<charqty; i++)
		{
			charstatus = charstatus & 0xFFFFFFF0; //clear low bits
			nibble = 0;
			testchar = *(cmd_String + charOffset + i);
			if (testchar > 0x2F)
			{
				if(testchar < 0x3A)
				{
					//numeric character detected
					//convert to nibble
					nibble = testchar - 0x30;
					charstatus = charstatus | 0x40000000;
					charstatus = charstatus | 1;
				}
			}
			if (testchar > 0x40)
			{
				if (testchar < 0x47)
				{
					//hex character detected
					nibble = testchar - 0x41 + 0x0A;
					charstatus = charstatus | 0x20000000;
					charstatus = charstatus | 2;
				}
			}
			if ((charstatus & 0x0F) == 0)
			{
				//non numeric/hex character detected
				charstatus = charstatus | 0x80000000;
			}
			OutputVal = OutputVal << 4;
			OutputVal = OutputVal & 0xFFFFFFF0;	//clear lowest nibble value
			OutputVal = OutputVal | nibble;
		}
	}

	else
	{
		//too many character specified
		charstatus = charstatus | 0x90000000;
	}

	charstatus = charstatus & 0xF0000000; //clear low bits
	OutputVal = OutputVal & 0x0FFFFFFF;
	OutputVal = OutputVal | charstatus;
	return OutputVal;
}


uint32_t Bcd2Hex(uint32_t InputVal)
{
	//Function designed to convert up to 8 BCD nibbles into a single HEX value
	//Created 7OCT2021
	//Last edited 15FEB2023
	uint32_t multiplier = 1;
	uint8_t nibble = 0;
	uint32_t OutputVal = 0;
	for (int i=0; i<7; i++)
	{
		nibble = InputVal & 0x0F;
		OutputVal = OutputVal + (nibble * multiplier);
		multiplier = multiplier * 10;
		InputVal = InputVal >> 4;
	}
	return OutputVal;
}


uint8_t IsNumericChar(uint8_t character)
{
	//Function used to determine if the supplied character is number 0x30 <= x <= 0x39
	//Created 10FEB2023
	uint8_t ErrVal = 1; //assume non numeric character
	if (character >= 0x30)
	{
		if (character <= 0x39)
		{
			ErrVal = 0;
		}
	}
	return ErrVal;
}
