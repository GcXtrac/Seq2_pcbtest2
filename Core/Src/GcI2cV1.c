/*
 * GcI2cV1.c
 *
 *  Created on: 13 Dec 2022
 *  Last edited 10JUL2024
 *      Author: Geoff
 */
#include <global-settingsV1.h>
#include "stm32l4xx_hal.h"  //this needs to be included for the compiler to understand what a uint8_t data type is...
#include "GcI2cV1.h"
#include <stdio.h>	//used for sprintf() function
#include <string.h>  //used for strlen(), strcat() function
//#include "SerialComms_V1.h"

#include <GcFunctionsV1.h>
#include "DataIntegrityV1.h"

extern I2C_HandleTypeDef hi2c2;
//extern uint32_t I2cTiming; //if non zero, this value will be decremented by TIM2 ISR
extern uint32_t I2cStatus; //this value will be set by the I2C callback (ISR)
extern uint16_t diagnosticsval;

//uint8_t I2cDataBuffer[0x20] = {0};
//extern uint8_t I2cDataBuffer[0x40];
static uint8_t I2cDataBuffer[0x40];

static uint32_t I2cTiming2 = 0;
static uint8_t I2cStatus2 = 0;
static uint8_t I2cDeviceAddress = 0;
static uint16_t I2cInternalAddress = 0;
static uint8_t I2cInternalAddressWidth = 0;
static uint16_t I2cQuantity = 0;
static uint16_t I2cSrcAddress = 0;
static uint16_t I2cDestAddress = 0;

const uint8_t I2cpagesize = 0x20;


/*
 * moved to *.h file
struct I2cConfig{
	uint16_t I2cInternalAddress;
	uint16_t I2cQuantity;
	uint8_t I2cInternalAddressWidth;
	uint8_t I2cDeviceAddress;
}structI2cConfig;  //The 'structI2cConfig' creates an instance of the structure
*/

struct I2cConfig tempstruct; //create instance of structure

struct I2cConfig4 *I2cConfig4Ptr;

struct I2cConfig I2cConfigData; // create an instance of the I2cConfig data structure and call it I2cConfigData


const uint32_t I2cTrials = 10;
const uint32_t I2cTimeout = 100;

struct I2cConfig GetI2cConfig(void) //fuction returns a structure called I2cConfig
{
	return I2cConfigData;
}


uint32_t WriteSeqData(uint8_t* bufferptr, uint8_t quantity, uint16_t destaddress)
{
	//Created 16JUL2024
	uint32_t response = 0;

	response = I2cWriteBlock(SEQUENCERMEMORY, destaddress, 2, bufferptr, quantity);
	return response;
}


uint32_t ReadSeqMat(uint8_t index)
{
	//Read 8 byte MAT entry from I2C memory
	//Created 16JUL2024
	uint16_t address = 0;
	uint16_t qty = 0;
	uint8_t response = 0;
	uint8_t MatBuffer[8] = {0};
	uint8_t* MatBufferPtr = 0;

	address = MATTABLEADDR + (index * SEQMATENTRYSIZE);

	MatBufferPtr = &MatBuffer[0];


	response = ReadSmallI2CDatablock5(SEQUENCERMEMORY, MatBufferPtr, address, SEQMATENTRYSIZE); //read a copy of the specified MAT entry
	if (response == 0)
	{
		//Test MAT entry CRC
		SetCrc16Value(0);
		uint16_t tempval = 0;
		tempval = CalculateBlockCrc(MatBufferPtr, (SEQMATENTRYSIZE-2));
		if ((uint8_t)(tempval >>8) != *MatBufferPtr+6)
		{
			response = 4;
		}
		if ((uint8_t)(tempval) != *MatBufferPtr+7)
		{
			response = 5;
		}

		if (response == 0)
		{
			//obtain block start address and associated quantity to calculate new block start address
			address = (uint16_t)((*MatBufferPtr) << 8) | (uint16_t) *(MatBufferPtr+1);
			qty = (uint16_t)((*(MatBufferPtr+2)) << 8) | (uint16_t) *(MatBufferPtr+3);
			address = address + qty;
		}


	}
	return ((uint32_t)(response << 24) | (uint32_t)(address));
}


uint32_t ReadSeqHeader(uint8_t* bufferptr)
{
	//Read 8 byte Sequencer header data from I2C memory & check CRC value
	//Created 16JUL2024
	uint8_t response = 0;
	//uint8_t ReadSmallI2CDatablock5(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty)
	response = ReadSmallI2CDatablock5(SEQUENCERMEMORY, bufferptr, SEQHEADERADDR, SEQHEADERSIZE); //read a copy of the specified MAT entry

	if (response == 0) //check for issues whilst reading header data from I2C device
	{
		//Calculate CRC value
		uint16_t tempval = 0;
		uint32_t response = 0;
		SetCrc16Value(0);
		//uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
		tempval = CalculateBlockCrc(bufferptr, 14); //ignore initial element (status byte)

		if (*(bufferptr + 14) == (uint8_t)(tempval >> 8))
		{
			response = 4;
		}
		if (*(bufferptr + 15) != (uint8_t)tempval)
		{
			response = 5;
		}


	}

	return (uint32_t)response;

}


uint32_t UpdateHeaderBlock(uint8_t* bufferptr)
{
	//Created 16JUL2024
	//calculate CRC for Header data currently held in RAM
	//write copy of data held in RAM comp;lete with updated CRC to I2C destination

	uint32_t response = 0;

	//Test MAT entry CRC
	SetCrc16Value(0);
	uint16_t tempval = 0;
	tempval = CalculateBlockCrc(bufferptr, (SEQHEADERSIZE-2));
	*(bufferptr+(SEQHEADERSIZE-2)) = (uint8_t)(tempval >>8);
	*(bufferptr+(SEQHEADERSIZE-1)) = (uint8_t)tempval;



	response = I2cWriteBlock(SEQUENCERMEMORY, SEQHEADERADDR, 2, bufferptr, SEQHEADERSIZE);
	return response;

}


uint32_t CheckHeaderBlock(void)
{
	//Created 15JUl2024
	HAL_StatusTypeDef halret;
	uint32_t error = 0;
	uint8_t bytebuffer[16] = {0}; //first element holds a status byte, while 16off remaining elements hold data
	uint8_t* byteptr = 0;
	byteptr = &bytebuffer[0];
	//returns:
	//	0: no issues
	//	1: I2C device was busy
	//	2: failed to communicate with I2C device
	//	3: comms timeout expired
	//	4: Header CRC low byte failed
	//	5: Header CRC high byte failed


	//read block of 16bytes from I2C memory
	halret = HAL_I2C_IsDeviceReady(&hi2c2, SEQUENCERMEMORY, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//halret = HAL_I2C_Mem_Read_IT(&hi2c2, device, intaddress, 2, pI2cData, BlockQty);
		halret = HAL_I2C_Mem_Read_IT(&hi2c2, SEQUENCERMEMORY, SEQHEADERADDR, 2, byteptr, 16);
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100; //TIM2 will decrement this value
			while ((I2cStatus2 & 0x08) == 0)
			{
				if (I2cTiming2 == 0)
				{
					//timeout period expired
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C 'receive complete' callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag
				error = 0; //indicate no errors

			}
			else
			{
				//timout expired
				error = 3;
			}


		}
		else
		{
			//failed to communicate with I2C memory device
			error = 2;
		}
		//header data read

		if (error == 0) //check for issues whilst reading header data from I2C device
		{
			//Calculate CRC value
			uint16_t tempval = 0;
			uint32_t response = 0;
			SetCrc16Value(0);
			//uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
			tempval = CalculateBlockCrc(byteptr+1, 14); //ignore initial element (status byte)

			if (*(byteptr + 15) == (uint8_t)(tempval >> 8))
			{
				error = 4;
			}
			if (*(byteptr + 16) != (uint8_t)tempval)
			{
				error = 5;
			}


		}

	}
	else
	{
		error = 1;
	}
	return error;
}


uint32_t UpdateSeqHeaderCrc(void)
{
	//Created 15JUL2024
	HAL_StatusTypeDef halret;
	uint32_t error = 0;
	uint8_t bytebuffer[17] = {0};
	uint8_t* byteptr = 0;
	byteptr = &bytebuffer[0];


	//read block of 14bytes from I2C memory
	halret = HAL_I2C_IsDeviceReady(&hi2c2, SEQUENCERMEMORY, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//halret = HAL_I2C_Mem_Read_IT(&hi2c2, device, intaddress, 2, pI2cData, BlockQty);
		halret = HAL_I2C_Mem_Read_IT(&hi2c2, SEQUENCERMEMORY, SEQHEADERADDR, 2, byteptr + 1, 14);
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100; //TIM2 will decrement this value
			while ((I2cStatus2 & 0x08) == 0)
			{
				if (I2cTiming2 == 0)
				{
					//timeout period expired
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C 'receive complete' callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag
				*byteptr = 0; //indicate no errors

			}
			else
			{
				//timout expired
				*byteptr = 2;
			}


		}
		else
		{
			//failed to communicate with I2C memory device
			*byteptr = 4;
		}
		//header data read
	}

	if (*byteptr == 0)
	{
		//now calculate CRC for read block
		uint16_t tempval = 0;
		uint32_t response = 0;
		SetCrc16Value(0);
		//uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
		tempval = CalculateBlockCrc(byteptr+1, 14);
		*(byteptr + 15) = (uint8_t)tempval >> 8;
		*(byteptr + 16) = (uint8_t)tempval;

		//now write complete block back to I2C memory
		//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
		response = I2cWriteBlock(SEQUENCERMEMORY, SEQHEADERADDR, 2, byteptr + 1, 16);
		if (response != 0)
		{
			error = 2;
		}

	}
	return error;
}


uint8_t sizeofI2cDataBuffer(void)
{
	//Created 20MAR2023
	uint8_t temp;
	temp = sizeof(I2cDataBuffer);
	return temp;
}


uint8_t* GetI2cData(void)
{
	return &I2cDataBuffer[0];
}


void SetI2cDestPtr(uint16_t address)
{
	//Created 10JAN2023
	I2cDestAddress = address;
}


void SetI2cSourcePtr(uint16_t address)
{
	//Created 10JAN2023
	I2cSrcAddress = address;
}



void DecrementI2cTiming(void)
{
	//function called by TIM2 ISR
	//Created 13DEC2022
	if (I2cTiming2 != 0)
	{
		I2cTiming2--;
	}
}

void ClearI2cStatus(void)
{
	//Created 5DEC2022
	I2cStatus = 0;
}

uint32_t GetI2cStatus(void)
{
	//Created 5DEC2022
	return I2cStatus;
}

void ClearI2cStatusBit0(void)
{
	//Created 5DEC2022
	I2cStatus = I2cStatus & 0xFFFE; //reset control flag
}

void SetI2cStatusBit(uint8_t bitpattern)
{
	//Setter function called from I2C ISR callback(s)
	//Created 13DEC2022
	I2cStatus2 = I2cStatus2 | bitpattern;
}


void SetI2cDeviceAddress(uint8_t address)
{
	//Setter function
	//Created 13DEC2022
	//Last edited 19APR2024
	I2cDeviceAddress = address;
	I2cConfigData.I2cDeviceAddress = address;
}


void SetI2cInternalAddress(uint16_t address)
{
	//Setter function
	//Created 13DEC2022
	I2cInternalAddress = address;
	I2cConfigData.I2cInternalAddress = address;
}


void SetI2cBlockSize(uint16_t Qty)
{
	//Setter function
	//Created 13DEC2022
	//Last edited 17MAY2024
	I2cQuantity = Qty;
	I2cConfigData.I2cQuantity = Qty;
}


void SetInternalAddressWidth(uint8_t width)
{
	//Setter function
	//Created 13DEC2022
	//Last edited 19APR2024
	I2cInternalAddressWidth = width;
	I2cConfigData.I2cInternalAddressWidth = width;
}


uint8_t CopyI2cBlock(void)
{
	//Function designed to copy a block of data from one area of I2C memory to another
	//Called from serial command "I2CCPY"
	//Created 10JAN2023
	//Last edited 28JUN2023
	//function requires use of serial commands
	//I2CSxxxx: set source address
	//I2CDxxxx: set destination address
	//I2CQxxxx: set block size

	char msg[100] = {};
	uint16_t qty = I2cQuantity;
	uint16_t src = I2cSrcAddress;
	uint16_t dst = I2cDestAddress;
	uint8_t block = 0;
	uint8_t ErrVal = 0;
	uint8_t* pI2cData = &I2cDataBuffer[0]; //point to 2ndary buffer

	sprintf(msg, "\r\nSource address: 0x%04X", I2cSrcAddress);
	SendSerial(msg);
	sprintf(msg, "\r\nDestination address: 0x%04X", I2cDestAddress);
	SendSerial(msg);
	sprintf(msg, "\r\nBlock size: 0x%04X", I2cQuantity);
	SendSerial(msg);

	while(1)
	{
		//break total specified data block into a series of smaller blocks

		if (qty > 16)
		{
			//set maximum data block size to be read
			block = 16;
		}
		else
		{
			block = qty;
		}


		//check for destination address crossing page boundary...

		sprintf(msg, "\r\nSource address: 0x%04X, Dest address: 0x%04X, current block size:%d, remaining bytes:%d", src, dst, block, qty);
		SendSerial(msg);

		if (block > 1)
		{
			for (uint8_t i=0; i<block-1; i++)
			{
				if (((dst + 1 + i) & 0x0F) == 0) //check that next byte isn't going to cross page boundary
				{
					block = i + 1; //override block value to ensure no crossing of page boundary!

					sprintf(msg, "\r\nCurrent block will cross page boundary, block size reduced (%dbytes)", block);
					SendSerial(msg);

					break;
				}
			}
		}


		//uint8_t ReadSmallI2CDatablock3(uint16_t intaddress, uint8_t BlockQty);
		ErrVal = ReadSmallI2CDatablock3(src, block, 2);
		sprintf(msg, "\r\nSource data block read to SRAM buffer");
		SendSerial(msg);
		if (ErrVal == 0)
		{
			//uint32_t I2cWriteBlock2(uint8_t* srcdata, uint8_t qty);
			I2cInternalAddress = dst;
			ErrVal = I2cWriteBlock2(pI2cData, block, 1);
			if (ErrVal != 0)
			{
				sprintf(msg, "\r\nProblem writing copied data to device!");
				SendSerial(msg);

				ErrVal = 6;
				break;
			}
			else
			{
				sprintf(msg, "\r\nSource data block written to destination address");
				SendSerial(msg);
			}
		}


		qty = qty - block; //calculate remaining bytes to copy
		src = src + block; //pointer to start of next source data block
		dst = dst + block; //point to start of next destination block

		if (qty == 0)
		{
			sprintf(msg, "\r\nBlock copy complete");
			SendSerial(msg);
			break;
		}

	}

	return ErrVal;
}


uint32_t I2cMemoryFill(uint8_t data)
{
	//Created 14DEC2022
	//Last edited 9FEB2023
	//determine block size
	//command I2CQ sets block size

	uint16_t destaddress = 0;
	uint8_t* pI2cData = &I2cDataBuffer[0]; //point to 2ndary buffer
	uint16_t q = I2cQuantity;
	uint16_t block = 0;
	uint16_t blockindex = 0;
	uint8_t ErrVal = 0;
	char msg[100] = {};



	sprintf(msg, "start address: 0x%04X, q: 0x%02x\r\n", I2cInternalAddress, q);
	SendSerial(msg);


	destaddress = I2cInternalAddress;
	while (q != 0)
	{
		if (q > I2cpagesize)
		{
			block = I2cpagesize;
		}
		else
		{
			block = q;
		}

		//check for crossing of page boundary
		for (uint16_t j=1; j<block; j++)
		{
			if (((destaddress + j) & 0x1F) == 0)
			{
				//crossing of page boundary detected within current block size
				block = j;
				sprintf(msg, "block size adjusted to page boundary: %d\r\n", block);
				SendSerial(msg);
			}
		}

		for (uint16_t i=0; i<block; i++)
		{
			*(pI2cData+i) = data; //transfer block data to output buffer
		}

		for (uint8_t j=0; j<3; j++) //3 attempts to write block
		{
			sprintf(msg, "blockindex:%d, dest address: 0x%04x, block: %d, write attempt:%d\r\n", blockindex, destaddress, block, j);
			SendSerial(msg);


			//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty)
			ErrVal =  I2cWriteBlock(0xA0, destaddress, 2, pI2cData, block);
			if (ErrVal == 0)
			{
				break;
			}
			//DelayTime = 100;
			//while (DelayTime != 0)
			//{}
			Delay(100);
		}
		blockindex++;
		destaddress = destaddress + block;
		q = q - block;


	}

	if (ErrVal != 0)
	{
		sprintf(msg, "Filling memory FAILED!");
		SendSerial(msg);
	}

	return (uint32_t)ErrVal;
}


//static struct I2cConfig{
//	uint16_t I2cInternalAddress;
//	uint8_t I2cInternalAddressWidth;
//	uint8_t I2cDeviceAddress;
//};

//struct I2cConfiguration* ReadI2cConfig(void)
//void ReadI2cConfig(struct I2cConfig* I2cConfig2)
//void ReadI2cConfig(struct I2cConfig4* I2cConfig2) //I2cConfig4 declared in global-settingsV1.h
void ReadI2cConfig(struct I2cConfig4* I2cConfig4Ptr) //I2cConfig4 declared in global-settingsV1.h
{
	//Function to obtain I2C settings
	//Created 13DEC2022
	//Last edited 14DEC2022

	//struct I2cConfig I2cConfig2;
	I2cConfig4Ptr->I2cDeviceAddress = I2cDeviceAddress;
	I2cConfig4Ptr->I2cInternalAddress = I2cInternalAddress;
	I2cConfig4Ptr->I2cInternalAddressWidth = I2cInternalAddressWidth;
	I2cConfig4Ptr->I2cQuantity = I2cQuantity;
	//return &I2cConfig2;
}


void I2cTest(void)
{
	//Function called from serial command "I2CT"
	//Created 13DEC2022
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	uint8_t* pI2cData = 0;
	uint8_t I2cByteCount = 0;

	//I2cDeviceAddress = 0x78;
	//I2cDeviceAddress = 0xA0; //24LC32 memory device (this works)
	//I2cDeviceAddress = 0x50; //24LC32 memory device
	pI2cData = &I2cDataBuffer[0];
	*pI2cData = 0x12;
	*(pI2cData+1) = 0x34;
	*(pI2cData+2) = 0x56;
	I2cByteCount = 3;
	//I2cInternalAddress = 0x0002;
	I2cInternalAddressWidth = 2; //bytes

	//How do we set the device internal I2C address pointer???
	//looks like write to device with address pointer values
	//then read from device a number of bytes
	//how do we get across the repeated start???? Use the HAL_I2C_mem_Read/Write functions

	//HAL_StatusTypeDef HAL_I2C_IsDeviceReady(&hi2c1, I2cDeviceAddress, I2cTrials, I2cTimeout); //return HAL status value
	//HAL_StatusTypeDef y;
	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//halret = HAL_I2C_Master_Transmit_IT(&hi2c1, I2cDeviceAddress, pI2cData, I2cByteCount);
		halret = HAL_I2C_Mem_Write_IT(&hi2c2, I2cDeviceAddress, I2cInternalAddress, I2cInternalAddressWidth, pI2cData, I2cByteCount);

		if (halret == HAL_OK)
		{

			I2cTiming2 = 100;
			while (I2cTiming2 != 0) //value decremented by TIM2 ISR
			{
			}

			if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MasterTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag

				sprintf(msg, "I2CT command completed!\r\n");
				SendSerial(msg);
			}


		}
		else
		{
			sprintf(msg, "I2C mem write Error!\r\n");
			SendSerial(msg);
		}

	}
	else
	{
		sprintf(msg, "I2C device busy!\r\n");
		SendSerial(msg);
	}
}


uint32_t I2cAbort(void)
{
	//Created 13DEC2022
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	uint8_t ErrVal;

	halret = HAL_I2C_Master_Abort_IT(&hi2c2, I2cDeviceAddress);
	if (halret == HAL_OK)
	{
		I2cTiming2 = 100; //TIM2 will decrement this value
		while ((I2cStatus2 & 0x40) == 0)
		{
			if (I2cTiming2 == 0)
			{
				//timeout period expired
				break;
			}
		}
		if ((I2cStatus2 & 0x40) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
		{
			//abort call back was executed
			sprintf(msg, "I2C Master comms Abort request SUCCESSFUL!\r\n");
			SendSerial(msg);
		}
		else
		{
			//timeout expired whilst waiting for abort call back
			sprintf(msg, "waiting for I2C Master Abort request failed! (I2cStatus2: 0x%02X)\r\n", I2cStatus2);
			SendSerial(msg);
			ErrVal = 2;
		}
	}
	else
	{
		sprintf(msg, "I2C Master Abort request failed!\r\n");
		SendSerial(msg);
		ErrVal = 1;
	}
	return (uint32_t)ErrVal;
}


uint8_t WriteI2cDisplaydata(uint8_t I2cDevice, uint8_t LcdDataQty, uint8_t* pI2cData)
{
	//function designed for feeding I2C interfaced display modules with drivers such as SSD1306
	//function relies on I2C interrupt call backs
	//Created 14JAN2022
	//Last edited 13DEC2022

	uint8_t LcdErr = 0;
	//uint32_t ProcessingTime = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};

	I2cStatus2 = 0;	//reset global flag

	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDevice, I2cTrials, I2cTimeout); //check device is present
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		//halret = HAL_I2C_Mem_Write_IT(&hi2c1, I2cDeviceAddress, addr, I2cInternalAddressWidth, pI2cData, 1);
		halret = HAL_I2C_Master_Transmit_IT(&hi2c2, I2cDevice, pI2cData, LcdDataQty);

		if (halret == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100;
			//while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack
			while((I2cStatus2 & 0x01) == 0) //flag set by HAL_I2C_MasterTxCpltCallBack
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					//ProcessingTime = 100 - I2cTiming;
					break;
				}
			}

			//if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			if ((I2cStatus2 & 0x01) != 0) //flag set by HAL_I2C_MasterTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFE; //reset control flag
				//sprintf(msg, "I2C LCD write complete! processing time: %ldms\r\n", ProcessingTime);
				//SendSerial(msg);

			}
			else
			{
				sprintf(msg, "timeout occurred whilst waiting for I2C LCD write!\r\n");
				SendSerial(msg);
				LcdErr = 3;
			}


		}
		else //halret != HAL_OK : data block writing failed
		{
			sprintf(msg, "I2C display write Error!\r\n");
			SendSerial(msg);
			LcdErr = 2;

		}

	}
	else //halret != HAL_OK : I2C device was busy before we start
	{
		sprintf(msg, "I2C display busy!\r\n");
		SendSerial(msg);
		LcdErr = 1;
	}
	return LcdErr;
}


uint32_t I2cWriteBlock2Blocking(uint8_t* srcdata, uint8_t qty, uint8_t debugdata)
{
	//Function designed to write a block of data to an I2C memory device
	//Used by serial command "XR" for testing
	//Created 7FEB2022
	//Last edited 8FEB2023
	//Input values:
	//	DeviceAddress holds the I2C device address
	//	InternalAddress holds the I2C internal memory address
	//	InternalAddressWidth defines number of bytes used to hold internal address (1/2)
	//	srcdata is a pointer to an array of byte data
	//	qty defines quantity of data to be written
	//  diagnosticsval

	HAL_StatusTypeDef halret;
	char msg[100] = {};

	uint32_t ErrorVal = 0;
	uint8_t attempts = 3;

	//uint16_t addr = UserVal >> 8;
	//sprintf(msg, "\taddress: 0x%04X\r\n", addr);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

	//pI2cData = &I2cDataBuffer[0];
	//*pI2cData = UserVal & 0xFF;
	//sprintf(msg, "\tdata: 0x%02X\r\n", *pI2cData);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
	//returns:
	//			0: function successfully completed
	//			1: I2c device was busy
	//			2: write failed
	//			3: timeout expired

	if (debugdata != 0)
	{
		sprintf(msg, "Function 'I2cWriteBlock2Blocking'\r\n");
		if (debugdata == 2)
		{
			SendDiagnostics(msg);
		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}
		sprintf(msg, "\taddress:0x%04X, qty:0x%02X\r\n",I2cInternalAddress, qty);
		if (debugdata == 2)
		{
			SendDiagnostics(msg);

		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}
	}

	for (uint8_t i=0; i < attempts; i++)
	{
		halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
		if (halret == HAL_OK)
		{

			//transmit data
			//HAL_StatusTypeDef y;
			//prepare to write a single byte to the I2C device
			halret = HAL_I2C_Mem_Write_IT(&hi2c2, I2cDeviceAddress, I2cInternalAddress, I2cInternalAddressWidth, srcdata, qty);
			if (halret == HAL_OK)
			{

				//delay: wait for write cycle to complete
				I2cTiming2 = 100;
				while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
				{
					if(I2cTiming2 == 0) //value decremented by TIM2 ISR
					{
						break;
					}
				}

				if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
				{
					//we get here once the I2C transmit complete callback
					I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag

					if (debugdata != 0)
					{
						sprintf(msg, "\tI2C mem write complete!\r\n");
						if (debugdata == 2)
						{
							SendDiagnostics(msg);
						}
						if (debugdata == 1)
						{
							SendSerial(msg);
						}
					}
					break; //exit for loop
				}
				else
				{
					ErrorVal = 3;
					if (debugdata != 0)
					{
						sprintf(msg, "\ttimeout occurred whilst waiting for I2C mem write!\r\n");
						if (debugdata == 2)
						{
							SendDiagnostics(msg);
						}
						if (debugdata == 1)
						{
							SendSerial(msg);
						}
					}
				}

			}
			else
			{
				ErrorVal = 2;
				if (debugdata != 0)
				{
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
				}

			}

		}
		else
		{
			//ErrorVal = 1;
			if (debugdata != 0)
			{
				sprintf(msg, "\tI2C Device currently busy!\r\n");
				if (debugdata == 2)
				{
					SendDiagnostics(msg);

				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}
			}

			Delay(25);
		}
	}


	return ErrorVal;
}



uint32_t I2cWriteBlock2(uint8_t* srcdata, uint8_t qty, uint8_t debugdata)
{
	//Function designed to write a block of data to an I2C memory device
	//Used by serial commnand "XR" for testing
	//Created 7FEB2022
	//Last edited 8FEB2023
	//Input values:
	//	DeviceAddress holds the I2C device address
	//	InternalAddress holds the I2C internal memory address
	//	InternalAddressWidth defines number of bytes used to hold internal address (1/2)
	//	srcdata is a pointer to an array of byte data
	//	qty defines quantity of data to be written
	//returns:
	//			0: function successfully completed
	//			1: I2c device was busy
	//			2: write failed
	//			3: timeout expired


	HAL_StatusTypeDef halret;
	char msg[100] = {};

	uint32_t ErrorVal = 0;

	//uint16_t addr = UserVal >> 8;
	//sprintf(msg, "\taddress: 0x%04X\r\n", addr);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

	//pI2cData = &I2cDataBuffer[0];
	//*pI2cData = UserVal & 0xFF;
	//sprintf(msg, "\tdata: 0x%02X\r\n", *pI2cData);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);


	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		halret = HAL_I2C_Mem_Write_IT(&hi2c2, I2cDeviceAddress, I2cInternalAddress, I2cInternalAddressWidth, srcdata, qty);

		if (halret == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100;
			while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					break;
				}
			}

			if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag
				sprintf(msg, "\r\nI2C mem write complete!\r\n");
				if (debugdata == 2)
				{
					SendDiagnostics(msg);
				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}
			}
			else
			{
				ErrorVal = 3;
				sprintf(msg, "\r\ntimeout occurred whilst waiting for I2C mem write!\r\n");
				if (debugdata == 2)
				{
					SendDiagnostics(msg);
				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}
			}

		}
		else
		{
			ErrorVal = 2;
			sprintf(msg, "\r\nI2C mem write Error!\r\n");
			if (debugdata == 2)
			{
				SendDiagnostics(msg);
			}
			if (debugdata == 1)
			{
				SendSerial(msg);
			}

		}

	}
	else
	{
		ErrorVal = 1;
		sprintf(msg, "\r\nI2C Device busy!\r\n");
		if (debugdata == 2)
		{
			SendDiagnostics(msg);
		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}
	}


	return ErrorVal;
}


uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty)
{
	//Function designed to write a block of data to an I2C memory device
	//Used by serial commnand "XR" for testing
	//Created 7FEB2022
	//Last edited 10JUL2024
	//Input values:
	//	DeviceAddress holds the I2C device address
	//	InternalAddress holds the I2C internal memory address
	//	InternalAddressWidth defines number of bytes used to hold internal address (1/2)
	//	srcdata is a pointer to an array of byte data
	//	qty defines quantity of data to be written
	HAL_StatusTypeDef halret;
	char msg[100] = {};

	uint32_t ErrorVal = 0;

	//uint16_t addr = UserVal >> 8;
	//sprintf(msg, "\taddress: 0x%04X\r\n", addr);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

	//pI2cData = &I2cDataBuffer[0];
	//*pI2cData = UserVal & 0xFF;
	//sprintf(msg, "\tdata: 0x%02X\r\n", *pI2cData);
	//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
	//returns:
	//			0: function successfully completed
	//			1: I2c device was busy
	//			2: write failed
	//			3: timeout expired

	halret = HAL_I2C_IsDeviceReady(&hi2c2, DeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		halret = HAL_I2C_Mem_Write_IT(&hi2c2, DeviceAddress, InternalAddress, (uint16_t)InternalAddressWidth, srcdata, (uint16_t)qty);

		if (halret == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100;
			while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					break;
				}
			}

			if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag
				sprintf(msg, "I2C mem write complete!\r\n");
				//SendSerial(msg);
			}
			else
			{
				ErrorVal = 3;
				sprintf(msg, "timeout occurred whilst waiting for I2C mem write!\r\n");
				//SendSerial(msg);
			}

		}
		else
		{
			ErrorVal = 2;
			sprintf(msg, "I2C mem write Error!\r\n");
			//SendSerial(msg);
		}

	}
	else
	{
		ErrorVal = 1;
		sprintf(msg, "I2C Device busy!\r\n");
		//SendSerial(msg);
	}


	return ErrorVal;
}


uint32_t Write2I2cDevice(uint16_t Setting, uint8_t I2cDeviceAddress2)
{
	//function writes single data byte to I2C internal address
	//based on serial command "I2Caaaadd"
	//Created 4NOV2021
	//Input variables:
	//		Setting: consists of 16bit value containing both I2c Internal address << 8 | 8bit data
	//		I2cDeviceAddress2: I2C device address

	HAL_StatusTypeDef halret;
	char msg[100] = {};
	uint16_t addr2 = (Setting >> 8) & 0xFFFF;
	uint8_t data = Setting & 0xFF;
	uint16_t I2cInternalAddressWidth2 = 1;
	uint32_t retval = 0;

	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress2, 10, 100);
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		halret = HAL_I2C_Mem_Write_IT(&hi2c2, I2cDeviceAddress, addr2, I2cInternalAddressWidth2, &data, 1);

		if (halret == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100; //value is decremented byt TIM2
			while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					break;
				}
			}

			if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag
				sprintf(msg, "I2C mem write complete!\r\n");
				SendSerial(msg);
			}
			else
			{
				sprintf(msg, "timeout occurred whilst waiting for I2C mem write!\r\n");
				SendSerial(msg);
			}

		}
		else
		{
			sprintf(msg, "I2C mem write Error!\r\n");
			SendSerial(msg);
		}

	}
	else
	{
		sprintf(msg, "I2C Device busy!\r\n");
		SendSerial(msg);
		retval = -1;
	}
	return retval;

}


uint32_t I2cWriteByteBlocking(uint16_t IntAddress, uint8_t data, uint8_t debugdata)
{
	//Function used to write a single byte to I2C device but will retry if device was busy...
	//called by serial command "I2CWaaaadd"
	//Created 19DEC2022
	//Last edited 25APR2024

	HAL_StatusTypeDef halret;

	char msg[100] = {};
	uint8_t ErrVal = 0;
	uint8_t* pInt;
	pInt = &data;
	uint8_t attempts = 3;

	/*
	sprintf(msg, "Function 'I2cWriteByteBlocking' address:0x%04X, data:0x%02X\r\n", IntAddress, data);
	if (debugdata == 2)
	{
		SendDiagnostics(msg);
	}
	if (debugdata == 1)
	{
		SendSerial(msg);
	}
	*/

	ErrVal = 1; //remains if number of write attempotsa wexpires.
	for (uint8_t i=0; i<attempts; i++)
	{
		halret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)I2cDeviceAddress, I2cTrials, I2cTimeout);
		if (halret == HAL_OK)
		{

			//transmit data
			//HAL_StatusTypeDef y;
			//prepare to write a single byte to the I2C device

			if (I2cInternalAddressWidth == 2)
			{
				halret = HAL_I2C_Mem_Write_IT(&hi2c2, (uint16_t)I2cDeviceAddress, IntAddress, I2C_MEMADD_SIZE_16BIT, pInt, 1);
			}
			else
			{
				halret = HAL_I2C_Mem_Write_IT(&hi2c2, (uint16_t)I2cDeviceAddress, IntAddress, I2C_MEMADD_SIZE_8BIT, pInt, 1);
			}
			if (halret == HAL_OK)
			{

				//delay: wait for write cycle to complete
				I2cTiming2 = 100;
				while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
				{
					if(I2cTiming2 == 0) //value decremented by TIM2 ISR
					{
						break;
					}
				}

				if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
				{
					//we get here once the I2C transmit complete callback
					I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag
					/*
					sprintf(msg, "I2C mem write complete!\r\n");
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
					*/
					ErrVal = 0;
					break; //exit for loop
				}
				else
				{

					/*
					sprintf(msg, "timeout occurred whilst waiting for I2C mem write!\r\n");
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
					*/
					ErrVal = 3;
				}


			}
			else
			{
				/*
				sprintf(msg, "I2C mem write Error!\r\n");
				if (debugdata == 2)
				{
					SendDiagnostics(msg);
				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}
				*/
				ErrVal = 2;

			}

		}
		else
		{
			/*
			sprintf(msg, "I2C Device currently busy!\r\n");
			if (debugdata == 2)
			{
				SendDiagnostics(msg);
			}
			if (debugdata == 1)
			{
				SendSerial(msg);
			}
			*/
			ErrVal = 4;
			Delay(25); //allow I2C device to complete previous task before retrying
		}
	}
	return ErrVal;
}


uint32_t I2cWriteByte(uint16_t IntAddress, uint8_t data, uint8_t DebugDisp)
{
	//Function used to write a single byte to I2C device
	//Created 13DEC2022
	//Last edited 19APR2024
	//output:
	//	ErrVal:
	//		0x00: Communications successful
	//		0x01: device was not ready
	//		0x02: problem whilst trying to write data to memory
	//		0x03: timeout period expired whilst waiting for comms to compolete

	HAL_StatusTypeDef halret;

	//char msg[100] = {};
	uint8_t ErrVal = 0;
	uint8_t* pInt;
	pInt = &data;


	halret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		halret = HAL_I2C_Mem_Write_IT(&hi2c2, (uint16_t)I2cDeviceAddress, IntAddress, (uint16_t)I2cInternalAddressWidth, pInt, 1);

		if (halret == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100;
			while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack)
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					break;
				}
			}

			if ((I2cStatus2 & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFB; //reset control flag
				if (DebugDisp != 0)
				{
					ErrVal = 0;
					//sprintf(msg, "I2C mem write complete!\r\n");
					//SendSerial(msg);
				}
			}
			else
			{
				if (DebugDisp != 0)
				{
					//sprintf(msg, "timeout occurred whilst waiting for I2C mem write!\r\n");
					//SendSerial(msg);
				}
				ErrVal = 3;
			}


		}
		else
		{
			if (DebugDisp != 0)
			{
				//sprintf(msg, "I2C mem write Error!\r\n");
				//SendSerial(msg);
			}
			ErrVal = 2;

		}

	}
	else
	{
		if (DebugDisp != 0)
		{
			//sprintf(msg, "I2C Device busy!\r\n");
			//SendSerial(msg);
		}
		ErrVal = 1;
	}
	return ErrVal;
}


uint8_t ReadSmallI2CDatablock5(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty)
{
	//function to read small block (256 bytes) of data from I2C memory
	//Created 16JUL2024
	//Last edited 16JUL2024
	//returns:
	//	0: No issues
	//	1: timeout expired
	//	2: block ad failed

	uint8_t ErrVal = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	char msgtemp[100] = {};
	I2cStatus2 = 0;

	halret = HAL_I2C_IsDeviceReady(&hi2c2, device, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//receive data

		//prepare to read memory header data

		halret = HAL_I2C_Mem_Read_IT(&hi2c2, device, intaddress, 2, pI2cData, BlockQty);
		//Now need to wait until memory read transaction has completed...
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100; //TIM2 will decrement this value
			while ((I2cStatus2 & 0x08) == 0)
			{
				if (I2cTiming2 == 0)
				{
					//timeout period expired
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C 'receive complete' callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

//				if (debugdata != 0)
//				{
//					uint8_t n = 0;
//					for (int i=0; i<BlockQty; i++)
//					{
//						if ((n & 0x0F) == 0)
//						{
//
//							if (i != 0)
//							{
//								sprintf(msgtemp, "\r\n");
//								strcat (msg, msgtemp);
//								if (debugdata == 2)
//								{
//									SendDiagnostics(msg);
//
//								}
//								if (debugdata == 1)
//								{
//									SendSerial(msg);
//								}
//							}
//							//start a new line
//							sprintf(msg, "\r\na:0x%04X, ", intaddress+i);
//							if (debugdata == 2)
//							{
//								SendDiagnostics(msg);
//
//							}
//							if (debugdata == 1)
//							{
//								SendSerial(msg);
//							}
//							strcpy(msg,""); //clear string
//							n = 0;
//						}
//						//sprintf(msgtemp, "0x%02X, ", *(pI2cData+i));
//						sprintf(msgtemp, "%02X ", *(pI2cData+i));
//						strcat (msg, msgtemp);
//						n++;
//					}


//					sprintf(msgtemp, "\r\n");
//					strcat (msg, msgtemp);
//					if (debugdata == 2)
//					{
//						SendDiagnostics(msg);
//					}
//					if (debugdata == 1)
//					{
//						SendSerial(msg);
//					}
//				}


			}

			else
			{
				//timout expired
//				sprintf(msg, "I2C read timeout expired! (I2cStatus: 0x%02lX)\r\n", I2cStatus);
//				if (debugdata == 2)
//				{
//					SendDiagnostics(msg);
//
//				}
//				if (debugdata == 1)
//				{
//					SendSerial(msg);
//				}

				ErrVal = 1;
			}


		}
		else
		{
//			sprintf(msg, "I2C block read failed!\r\n");
//			if (debugdata == 2)
//			{
//				SendDiagnostics(msg);
//			}
//			if (debugdata == 1)
//			{
//				SendSerial(msg);
//			}
			ErrVal = 2;
		}
		//header data read
	}

	else
	{
//		sprintf(msg, "I2C device not ready!\r\n");
//		if (debugdata == 2)
//		{
//			SendDiagnostics(msg);
//		}
//		if (debugdata == 1)
//		{
//			SendSerial(msg);
//		}
		ErrVal = 3;
	}

	return ErrVal;
}


uint8_t* ReadSmallI2CDatablock4(uint8_t control)
{
	//function to read small block (256 bytes) of data from I2C memory but doesn't display anything
	//Created 25APR2024
	//Last edited 15JUL2024
	//input:
	//	Control:
	//		0: reset read byte counter
	//		1: read another small block (16bytes) until specified bloxck size has been read
	//
	//	debugdata:
	//		0: no diagnostics serial output
	//		1: diagnostics data to serial port 1 (main port)
	//		2: diagnostics data to serial port 2

	uint8_t ErrVal = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	char msgtemp[100] = {};
	uint8_t* pI2cData = I2cDataBuffer; //0x40 bytes,
										//byte[0] is a status byte,
										//	1:specified data quantity read
										//	2:timeout expired
										//	4:failed to communicate with device - check device address
										//	8:Device not ready
										//	0x10: zero bytes specified
										//
										//byte[1,2] block start address
										//byte[3] block byte qty
										//byte[4...n] read data bytes
	uint8_t n = 0;
	uint8_t blocksize = 0x10;
	uint16_t blockstartaddress = 0;
	static uint16_t Qtyread = 0;
	uint16_t blockqty = 0;

	if (control == 0)
	{
		Qtyread = 0; //reset read byte counter
	}


	I2cStatus2 = 0;

	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cConfigData.I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//receive data

		//prepare to read memory header data
		blockstartaddress = I2cConfigData.I2cInternalAddress + Qtyread;
		blockqty = I2cConfigData.I2cQuantity - Qtyread;

		if (blockqty > blocksize)
		{
			blockqty = blocksize; //cap size of block
		}


		*(pI2cData + 1) = (uint8_t)(blockstartaddress >> 8);
		*(pI2cData + 2) = (uint8_t)(blockstartaddress);
		*(pI2cData + 3) = (uint8_t)(blockqty);

		if (blockqty != 0)
		{
			halret = HAL_I2C_Mem_Read_IT(&hi2c2, I2cConfigData.I2cDeviceAddress, blockstartaddress, I2C_MEMADD_SIZE_16BIT, pI2cData+4, blockqty);
			//Now need to wait until memory read transaction has completed...
			if (halret == HAL_OK)
			{

				I2cTiming2 = 100; //TIM2 will decrement this value
				while ((I2cStatus2 & 0x08) == 0)
				{
					if (I2cTiming2 == 0)
					{
						//timeout period expired
						break;
					}
				}

				if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
				{
					//we get here once the I2C 'receive complete' callback has been executed
					I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag
					*pI2cData = 0; //indicate no errors

					Qtyread = Qtyread + blockqty;
					if (Qtyread >= I2cConfigData.I2cQuantity)
					{
						*pI2cData = 1; //Indicate specified data block read has been completed
					}


				}
				else
				{
					//timout expired

	//					sprintf(msg, "I2C read timeout expired! (I2cStatus: 0x%02lX)\r\n", I2cStatus);
	//					if (debugdata == 2)
	//					{
	//						SendDiagnostics(msg);
	//					}
	//					if (debugdata == 1)
	//					{
	//						SendSerial(msg);
	//					}
	//					ErrVal = 1;
					*pI2cData = 2;
				}


			}
			else
			{
				//failed to commuinicate with I2C memory device
	//
	//				sprintf(msg, "I2C block read failed!\r\n");
	//				if (debugdata == 2)
	//				{
	//					SendDiagnostics(msg);
	//				}
	//				if (debugdata == 1)
	//				{
	//					SendSerial(msg);
	//				}
	//
	//				ErrVal = 2;
				*pI2cData = 4;
			}
			//header data read
		}
		else
		{
			//no data bytes to read
			*pI2cData = 0x10;
		}
	}

	else
	{
//			sprintf(msg, "I2C device not ready!\r\n");
//			if (debugdata == 2)
//			{
//				SendDiagnostics(msg);
//			}
//			if (debugdata == 1)
//			{
//				SendSerial(msg);
//			}
//			ErrVal = 3;
		*pI2cData = 8;
	}
//	}
//	else
//	{
////		sprintf(msg, "Specified block size too large!!\r\n");
////		if (debugdata == 2)
////		{
////			SendDiagnostics(msg);
////		}
////		if (debugdata == 1)
////		{
////			SendSerial(msg);
////		}
////		ErrVal = 4;
//		*pI2cData = 4;
//	}
	//return pI2cData; //could use this is pI2cData was declared as static
	return &I2cDataBuffer;
}



uint8_t ReadSmallI2CDatablock3(uint16_t intaddress, uint8_t BlockQty, uint8_t debugdata)
{
	//function to read small block (256 bytes) of data from I2C memory
	//Created 14JAN2022
	//Last edited 19APR2024
	//input:
	//	debugdata:
	//		0: no diagnostics serial output
	//		1: diagnostics data to serial port 1 (main port)
	//		2: diagnostics data to serial port 2

	uint8_t ErrVal = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	char msgtemp[100] = {};
	uint8_t* pI2cData = I2cDataBuffer;
	uint8_t n = 0;

	if (BlockQty < 0x40)
	{
		I2cStatus2 = 0;

		halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
		if (halret == HAL_OK)
		{
			//receive data

			//prepare to read memory header data

			halret = HAL_I2C_Mem_Read_IT(&hi2c2, I2cDeviceAddress, intaddress, 2, pI2cData, BlockQty);
			//Now need to wait until memory read transaction has completed...
			if (halret == HAL_OK)
			{

				I2cTiming2 = 100; //TIM2 will decrement this value
				while ((I2cStatus2 & 0x08) == 0)
				{
					if (I2cTiming2 == 0)
					{
						//timeout period expired
						break;
					}
				}

				if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
				{
					//we get here once the I2C 'receive complete' callback has been executed
					I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

					if (debugdata != 0)
					{

						for (int i=0; i<BlockQty; i++)
						{
							if ((n & 0x0F) == 0)
							{

								if (i != 0)
								{
									if (debugdata != 0)
									{
										sprintf(msgtemp, "\r\n");
										strcat (msg, msgtemp);
										if (debugdata == 2)
										{
											SendDiagnostics(msg);
										}
										if (debugdata == 1)
										{
											SendSerial(msg);
										}
									}
								}
								//start a new line
								if (debugdata != 0)
								{
									sprintf(msg, "\r\na:0x%04X, ", intaddress+i);
									if (debugdata == 2)
									{
										SendDiagnostics(msg);
									}
									if (debugdata == 1)
									{
										SendSerial(msg);
									}
									strcpy(msg,""); //clear string
								}
								n = 0;
							}
							if (debugdata != 0)
							{
								//sprintf(msgtemp, "0x%02X, ", *(pI2cData+i));
								sprintf(msgtemp, "%02X ", *(pI2cData+i));
								strcat (msg, msgtemp);
							}
							n++;
						}

						if (n != 0)
						{
							if (debugdata != 0)
							{
								//sprintf(msgtemp, "\r\n");
								//strcat (msg, msgtemp);
								if (debugdata == 2)
								{
									SendDiagnostics(msg);
								}
								if (debugdata == 1)
								{
									SendSerial(msg);
								}
							}
						}
					}


				}

				else
				{
					//timout expired

					sprintf(msg, "I2C read timeout expired! (I2cStatus2: 0x%02lX)\r\n", I2cStatus2);
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
					ErrVal = 1;
				}


			}
			else
			{

				sprintf(msg, "I2C block read failed!\r\n");
				if (debugdata == 2)
				{
					SendDiagnostics(msg);
				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}

				ErrVal = 2;
			}
			//header data read
		}

		else
		{
			sprintf(msg, "I2C device not ready!\r\n");
			if (debugdata == 2)
			{
				SendDiagnostics(msg);
			}
			if (debugdata == 1)
			{
				SendSerial(msg);
			}
			ErrVal = 3;
		}
	}
	else
	{
		sprintf(msg, "Specified block size too large!!\r\n");
		if (debugdata == 2)
		{
			SendDiagnostics(msg);
		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}
		ErrVal = 4;
	}
	return ErrVal;
}


uint8_t ReadSmallI2CDatablock2(uint8_t debugdata)
{
	//function to read small block (256 bytes) of data from I2C memory
	//Created 19DEC2022
	//Last edited 10FEB2023

	uint8_t ErrVal = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	char msgtemp[100] = {};
	I2cStatus2 = 0;
	//uint8_t* pInt = &I2cDataBuffer[0];
	uint8_t* pInt = I2cDataBuffer;

	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//receive data

		//prepare to read memory header data

		halret = HAL_I2C_Mem_Read_IT(&hi2c2, I2cDeviceAddress,  I2cInternalAddress, 2, pInt, I2cQuantity);
		//Now need to wait until memory read transaction has completed...
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100; //TIM2 will decrement this value
			while ((I2cStatus2 & 0x08) == 0)
			{
				if (I2cTiming2 == 0)
				{
					//timeout period expired
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C 'receive complete' callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

				if (debugdata != 0)
				{
					uint8_t n = 0;
					for (int i=0; i<I2cQuantity; i++)
					{
						if ((n & 0x0F) == 0)
						{

							if (i != 0)
							{
								sprintf(msgtemp, "\r\n");
								strcat (msg, msgtemp);
								if (debugdata == 2)
								{
									SendDiagnostics(msg);
								}
								if (debugdata == 1)
								{
									SendSerial(msg);
								}
							}
							//start a new line
							sprintf(msg, "\r\na:0x%04X, ", I2cInternalAddress + i);
							if (debugdata == 2)
							{
								SendDiagnostics(msg);
							}
							if (debugdata == 1)
							{
								SendSerial(msg);
							}
							strcpy(msg,""); //clear string
							n = 0;
						}
						//sprintf(msgtemp, "0x%02X, ", *(pI2cData+i));
						sprintf(msgtemp, "%02X ", *(pInt+i));
						strcat (msg, msgtemp);
						n++;
					}


					sprintf(msgtemp, "\r\n");
					strcat (msg, msgtemp);
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
				}


			}

			else
			{
				//timout expired

				sprintf(msg, "I2C read timeout expired! (I2cStatus2: 0x%02lX)\r\n", I2cStatus2);
				if (debugdata == 2)
				{
					SendDiagnostics(msg);
				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}

				ErrVal = 1;
			}


		}
		else
		{
			sprintf(msg, "I2C block read failed!\r\n");
			if (debugdata == 2)
			{
				SendDiagnostics(msg);
			}
			if (debugdata == 1)
			{
				SendSerial(msg);
			}

			ErrVal = 2;
		}
		//header data read
	}

	else
	{
		sprintf(msg, "I2C device not ready!\r\n");
		if (debugdata == 2)
		{
			SendDiagnostics(msg);
		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}

		ErrVal = 3;
	}

	return ErrVal;
}


uint8_t ReadSmallI2CDatablock(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty, uint8_t debugdata)
{
	//function to read small block (256 bytes) of data from I2C memory
	//Created 14JAN2022
	//Last edited 10FEB2023

	uint8_t ErrVal = 0;
	HAL_StatusTypeDef halret;
	char msg[100] = {};
	char msgtemp[100] = {};
	I2cStatus2 = 0;

	halret = HAL_I2C_IsDeviceReady(&hi2c2, device, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//receive data

		//prepare to read memory header data

		halret = HAL_I2C_Mem_Read_IT(&hi2c2, device, intaddress, 2, pI2cData, BlockQty);
		//Now need to wait until memory read transaction has completed...
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100; //TIM2 will decrement this value
			while ((I2cStatus2 & 0x08) == 0)
			{
				if (I2cTiming2 == 0)
				{
					//timeout period expired
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C 'receive complete' callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

				if (debugdata != 0)
				{
					uint8_t n = 0;
					for (int i=0; i<BlockQty; i++)
					{
						if ((n & 0x0F) == 0)
						{

							if (i != 0)
							{
								sprintf(msgtemp, "\r\n");
								strcat (msg, msgtemp);
								if (debugdata == 2)
								{
									SendDiagnostics(msg);

								}
								if (debugdata == 1)
								{
									SendSerial(msg);
								}
							}
							//start a new line
							sprintf(msg, "\r\na:0x%04X, ", intaddress+i);
							if (debugdata == 2)
							{
								SendDiagnostics(msg);

							}
							if (debugdata == 1)
							{
								SendSerial(msg);
							}
							strcpy(msg,""); //clear string
							n = 0;
						}
						//sprintf(msgtemp, "0x%02X, ", *(pI2cData+i));
						sprintf(msgtemp, "%02X ", *(pI2cData+i));
						strcat (msg, msgtemp);
						n++;
					}


					sprintf(msgtemp, "\r\n");
					strcat (msg, msgtemp);
					if (debugdata == 2)
					{
						SendDiagnostics(msg);
					}
					if (debugdata == 1)
					{
						SendSerial(msg);
					}
				}


			}

			else
			{
				//timout expired
				sprintf(msg, "I2C read timeout expired! (I2cStatus2: 0x%02lX)\r\n", I2cStatus2);
				if (debugdata == 2)
				{
					SendDiagnostics(msg);

				}
				if (debugdata == 1)
				{
					SendSerial(msg);
				}

				ErrVal = 1;
			}


		}
		else
		{
			sprintf(msg, "I2C block read failed!\r\n");
			if (debugdata == 2)
			{
				SendDiagnostics(msg);
			}
			if (debugdata == 1)
			{
				SendSerial(msg);
			}
			ErrVal = 2;
		}
		//header data read
	}

	else
	{
		sprintf(msg, "I2C device not ready!\r\n");
		if (debugdata == 2)
		{
			SendDiagnostics(msg);
		}
		if (debugdata == 1)
		{
			SendSerial(msg);
		}
		ErrVal = 3;
	}

	return ErrVal;
}


uint32_t I2cReadByte(uint16_t Intaddress)
{
	//Function to read a single I2C byte from I2C device
	//Created 13DEC2022
	//Last edited 25APR2024

	HAL_StatusTypeDef halret;
	uint8_t* pI2cData = 0;

	char msg[100] = {};
	char msgtemp[50] = {};
	uint8_t ErrVal = 0;

	//now read single byte from I2C address
	halret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)I2cDeviceAddress, I2cTrials, I2cTimeout); //this outputs device address and looks for the acknowledge bit
	if (halret == HAL_OK)
	{

		pI2cData = &I2cDataBuffer[0];

		if (I2cInternalAddressWidth == 2)
		{
			halret = HAL_I2C_Mem_Read_IT(&hi2c2, (uint16_t)I2cDeviceAddress, Intaddress, I2C_MEMADD_SIZE_16BIT, pI2cData, 1);
		}
		else
		{
			halret = HAL_I2C_Mem_Read_IT(&hi2c2, (uint16_t)I2cDeviceAddress, Intaddress, I2C_MEMADD_SIZE_8BIT, pI2cData, 1);
		}

		//Now need to wait until memory read transaction has completed...
		if (halret == HAL_OK)
		{

			I2cTiming2 = 100;
			while((I2cStatus2 & 0x08) == 0)
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					break;
				}
			}

			if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
			{
				//we get here once the I2C receive complete callback has been executed
				I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

				//sprintf(msg, "a:0x%04X, d:0x%02X, ", Intaddress, *pI2cData);
				//sprintf(msgtemp, "\r\n");
				//strcat (msg, msgtemp);

			}

			else
			{
				//timout expired
				//sprintf(msg, "timeout expired!\r\n");
				//SendSerial(msg);
				ErrVal = 3;
			}
		}

		else
		{
			//sprintf(msg, "I2C device was busy for another block read!\r\n");
			//SendSerial(msg);
			ErrVal = 2;

		}
	}

	else
	{
		//sprintf(msg, "I2C device is busy!\r\n");
		//SendSerial(msg);
		ErrVal = 1;
	}
	return (ErrVal << 24) | *pI2cData;

}


uint8_t I2cReadBlock(void)
{
	//Function used to read a block of data from an I2C device
	//Created 13DEC2022
	//Last edited 15DEC2022

	//following variables need to be set beforehand
	//I2cDeviceAddress

	HAL_StatusTypeDef halret;
	uint16_t I2cReadQty = 0; //reset read byte counter
	uint8_t I2cByteCount = 0;
	uint8_t* pI2cData = 0;
	uint16_t I2cInternalAddressptr = 0; //reset address offset pointer
	uint32_t ErrVal = 0;

	char msg[100] = {};
	char msgtemp[50] = {};

	halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret == HAL_OK)
	{
		//receive data
		//HAL_StatusTypeDef y;
		I2cReadQty = 0; //set read data quantity count
		I2cInternalAddressptr = 0;

		sprintf(msg, "Preparing to read block of data from I2C device!\r\n");
		SendSerial(msg);
		sprintf(msg, "Start address:0x%04X, Quantity:0x%04X\r\n", I2cInternalAddress, I2cQuantity);
		SendSerial(msg);
		//sprintf(msg, "Initial CRC16:0x%04X\r\n", GetCrc16Val());
		//SendSerial(msg);

		while (I2cReadQty < I2cQuantity)
		{
			if ((I2cQuantity - I2cReadQty) > 8)
			{
				//set maximum read block size
				//prepare to read a block of 8 bytes from I2C device
				I2cByteCount = 8;
			}
			else
			{
				//prepare to read remaining bytes from I2C device
				I2cByteCount = I2cQuantity - I2cReadQty;
			}

			halret = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
			if (halret == HAL_OK)
			{

				pI2cData = &I2cDataBuffer[0];

				halret = HAL_I2C_Mem_Read_IT(&hi2c2, I2cDeviceAddress, I2cInternalAddress + I2cInternalAddressptr, I2cInternalAddressWidth, pI2cData, I2cByteCount);
				//Now need to wait until memory read transaction has completed...
				if (halret == HAL_OK)
				{

					I2cTiming2 = 100; //TIM2 will decrement this value
					while ((I2cStatus2 & 0x08) == 0)
					{
						if (I2cTiming2 == 0)
						{
							//timeout period expired
							break;
						}
					}
					if ((I2cStatus2 & 0x08) != 0) //flag set by HAL_I2C_MasterRxCpltCallBack
					{
						//we get here once the I2C receive complete callback has been executed
						I2cStatus2 = I2cStatus2 & 0xFFF7; //reset control flag

						sprintf(msg, "a:0x%04X, q:%2d, ", I2cInternalAddress + I2cInternalAddressptr, I2cByteCount);
						for (int i=0; i<I2cByteCount; i++)
						{
							CalculareCrc16(*(pI2cData+i));
							sprintf(msgtemp, "%02X, ", *(pI2cData+i));
							strcat (msg, msgtemp);
						}
						sprintf(msgtemp, "\r\n");
						strcat (msg, msgtemp);
						SendSerial(msg);

						I2cReadQty = I2cReadQty + I2cByteCount; //update read byte counter
						I2cInternalAddressptr = I2cInternalAddressptr + I2cByteCount; //update address pointer offset

					}
					else
					{
						//timout expired
						sprintf(msg, "timeout expired! (I2cStatus2: 0x%02lX)\r\n", I2cStatus2);
						SendSerial(msg);
						ErrVal = 4;
						break;
					}

				}

				else
				{
					sprintf(msg, "I2C device was busy for another block read!\r\n");
					SendSerial(msg);
					ErrVal = 3;
					break;
				}

			}
			else
			{
				sprintf(msg, "I2C block read failed!\r\n");
				SendSerial(msg);
				ErrVal = 2;
			}

		}
		//indicate Final CRC value
		sprintf(msg, "CRC16:0x%04X\r\n", GetCrc16Val());
		SendSerial(msg);

	}

	else
	{
		sprintf(msg, "I2C device is busy!\r\n");
		SendSerial(msg);
		ErrVal = 1;
	}
	return ErrVal;
}


uint8_t	SendI2cData(uint16_t I2cDeviceAddress, uint8_t* pI2cData, uint8_t LcdDataQty)
{
	//Function designed to write a block of byte to an I2C device
	//The I2C is initially tested to make it isn't already busy (busy could be due to a write cycle being completed)
	//Created 13DEC2022
	uint8_t ErrVal = 0;
		//1: Target I2C device is busy
		//2: problem whilst preparing to write data
		//3: write timeout expired

	uint32_t ProcessingTime = 0;

	HAL_StatusTypeDef halret2;
	char msg[100] = {};

	//const uint32_t I2cTrials = 10;
	//const uint32_t I2cTimeout = 100;

	halret2 = HAL_I2C_IsDeviceReady(&hi2c2, I2cDeviceAddress, I2cTrials, I2cTimeout);
	if (halret2 == HAL_OK)
	{

		//transmit data
		//HAL_StatusTypeDef y;
		//prepare to write a single byte to the I2C device
		//halret = HAL_I2C_Mem_Write_IT(&hi2c1, I2cDeviceAddress, addr, I2cInternalAddressWidth, pI2cData, 1);
		halret2 = HAL_I2C_Master_Transmit_IT(&hi2c2, I2cDeviceAddress, pI2cData, LcdDataQty);

		if (halret2 == HAL_OK)
		{

			//delay: wait for write cycle to complete
			I2cTiming2 = 100;
			//while((I2cStatus2 & 0x04) == 0) //flag set by HAL_I2C_MemTxCpltCallBack
			while((I2cStatus2 & 0x01) == 0) //flag set by HAL_I2C_MasterTxCpltCallBack
			{
				if(I2cTiming2 == 0) //value decremented by TIM2 ISR
				{
					ProcessingTime = 100 - I2cTiming2;
					break;
				}
			}

			if ((diagnosticsval & 0x0004) != 0)
			{
				sprintf(msg, "Processing time: %ldms\r\n", ProcessingTime);
				//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
				SendSerial(msg);
			}

			//if ((I2cStatus & 0x04) != 0) //flag set by HAL_I2C_MemTxCpltCallBack
			if ((I2cStatus2 & 0x01) != 0) //flag set by HAL_I2C_MasterTxCpltCallBack
			{
				//we get here once the I2C transmit complete callback
				I2cStatus2 = I2cStatus2 & 0xFFFE; //reset control flag
			}
			else
			{
				ErrVal = 3; //writing data timeout expired
			}

		}
		else
		{
			ErrVal = 2; //there was a problem whilst preparing to write data to I2C
		}
	}
	else
	{
		ErrVal = 1; //I2C device is busy
	}
	return ErrVal;
}

