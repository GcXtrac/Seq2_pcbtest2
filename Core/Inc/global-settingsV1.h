/*
 * global-settingsV1.h
 *
 *  Created on: 14 Mar 2023
 *  Last edited 16JUL2024
 *      Author: Geoff
 */

#ifndef INC_GLOBAL_SETTINGSV1_H_ //guard
	#define INC_GLOBAL_SETTINGSV1_H_

    #include "stm32l4xx_hal.h"

	//added 14MAR2023
	#define TimIsrSeqStepUpdate //if defined, the timer ISR is responsible for loading the next step data and updating output(s)
		//used globally


	#define I2CALCDENABLED
	//#define Alcd1 //Midas I2C LCD alphanumeric display
	#define ALCD1ADDRESS 0x7C

	#define Alcd2 //ST7066U based alphanumeric LCD connected to MCP23017 port expander
	#define ALCD2ADDRESS 0x40


	#define OLEDADDRESS 0x78


	#define SEQUENCERMEMORY 0xA0

	#define SEQHEADERADDR 0x0020
	#define SEQHEADERSIZE 0x10

	#define MATTABLEADDR 0X100
	#define SEQMATENTRYSIZE 8

	#define BLOCKSIZE 0x10
	#define CMDMAXLENGTH 0x20

	#define FILLVALUE 0x55 //value used to fill spare memory locations (i.e sequencer MAT table)

	struct I2cConfig4{
		uint16_t I2cInternalAddress;
		uint16_t I2cQuantity;
		uint8_t I2cInternalAddressWidth;
		uint8_t I2cDeviceAddress;
	};



	//Intel hex output
	#define I2CREADBLOCKSIZE 0x20 //maximum data read block size
	#define MAXOUTPUTDATALENGTH 16 //specified maximum data length per output line

	//Sequencer settings
	#define SEQSTEPBUFFERELEMENTS 8 //this will affect the number of sequencer buffer underruns
									//30MAR2023 this was increased from 4 this may need to be increased depending on the number of successive fast sequence steps

#endif /* INC_GLOBAL_SETTINGSV1_H_ */
