/*
 * DataIntegrityV1.c
 *
 *  Created on: 8FEB2023
 *      Author: Geoff
 */
#include "stm32l4xx_hal.h"  //this needs to be included for the compiler to understand what a uint8_t data type is...
#include "DataIntegrityV1.h"

static uint16_t Crc16Value = 0; //reset CRC value


uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty)
{
	//Created 8FEB2023
	uint8_t Val = 0;
	SetCrc16Value(0);
	for (uint8_t i=0; i < qty; i++) //only include step time & data bytes in CRC calculation (ignore step index)
	{
		//CalculareCrc16(*(pPara + 2 + i));
		Val = *(pInt + i); //line used for debugging
		CalculareCrc16(*(pInt + i));
	}
	return GetCrc16Val();
}


void SetCrc16Value(uint16_t value)
{
	//Created 15DEC2022
	Crc16Value = value;
}

uint16_t GetCrc16Val(void)
{
	//Created 15DEC2022
	return Crc16Value;
}

uint16_t CalculareCrc16(uint8_t data)
{
	//function to calculate a new 16bit CRC value from an input 8 bit value
	//Created 20JAN2022
	//Last edited 15DEC2022
	//Input:
	//	Crc16Value holds current CRC16 value
	//	data is the new data value used to modify the CRC value
	Crc16Value = Crc16Value ^ ((uint16_t)data << 8);
	for (uint8_t i=0; i<8; i++)
	{
		if (Crc16Value & 0x8000)
		{
			Crc16Value = Crc16Value << 1;
			Crc16Value = Crc16Value ^ 0x1021; //xor with polynomial
		}
		else
		{
			Crc16Value = Crc16Value << 1;
		}
	}
	return Crc16Value;
}
