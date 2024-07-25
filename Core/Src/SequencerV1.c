/*
 * SequencerV1.c
 *
 *  Created on: Jul 22, 2024
 *      Author: GCholmeley
 */

#include "global-settingsV1.h" //global #defines
#include "SequencerV1.h"
#include "GcI2cV1.h"
#include "DataIntegrityV1.h"

//enum Sequencerstate
//{
//	DISABLED = 0,
//	ENABLED = 1,
//	STEPTIMERUNNING = 2,
//	NEXTSTEPSETUP = 3,
//};

//there is an issue if the following structures are placed in the *.h file???
struct SeqStepData{
	uint16_t time;
	uint16_t StepIndex;
	uint8_t parameter[0x20];
	uint16_t paracrc;
	uint8_t paracount;
}SeqStepDataElement;

struct Sequencer_Data{
	struct SeqStepData SeqStepDataArray[SEQSTEPBUFFERELEMENTS];
	//struct SeqStepData SeqStepDataElement[SEQSTEPBUFFERELEMENTS];
	//struct SeqStepDataElement[SEQSTEPBUFFERELEMENTS];
	uint16_t readptr;
	uint16_t writeptr;
	uint16_t fillcount;
	uint16_t NextSequenceSteptoBuffer;
	uint16_t MaxSteps;
	uint16_t CurrentSequenceStep;
	//uint8_t State;
	//enum Sequencerstate state; //enumeration value within a data structure???
	enum Sequencerstate {DISABLED = 0, ENABLED = 1, STEPTIMERUNNING = 2, NEXTSTEPSETUP = 3} state;
}SequencerData;



void SetSequencerState (uint8_t state)
{
	//Created 22JUL2024
	SequencerData.state = state;
}

uint8_t GetSequencerState(void)
{
	//Created 22JUL2024
	return SequencerData.state;
}


void InitialiseSequencerDataBuffer(void)
{
	//Created 22JUL2024
	//Last edited 23JUL2024
	SequencerData.readptr = 0;
	SequencerData.writeptr = 0;
	SequencerData.fillcount = 0;
	SequencerData.NextSequenceSteptoBuffer = 0;
	SequencerData.state = DISABLED;
	SequencerData.MaxSteps = 0;
	SequencerData.CurrentSequenceStep = 0;
}


uint16_t GetSeqStepTime(void)
{
	//Created 23JUL2024
	uint16_t tempval = 0;
	tempval = SequencerData.SeqStepDataArray[SequencerData.readptr].time;
	return tempval;
}


uint16_t GetSeqBufferFillLevel(void)
{
	//Created 22JUL2024
	return SequencerData.fillcount;
}


void SetSequencerMaxSteps(uint16_t MaxSteps)
{
	//Created 22JUL2024
	SequencerData.MaxSteps = MaxSteps;
}


uint16_t SeqBufferReadEntry(void)
{
	//function used to read next sequencer step from SRAM buffer
	uint8_t* byteptr;
	byteptr = SequencerData.SeqStepDataArray[SequencerData.readptr].parameter;
	for (uint8_t i=0; i<SequencerData.SeqStepDataArray[SequencerData.readptr].paracount; i++)
	{
		//now decode sequencer data string
	}
	SequencerData.readptr++;
	if (SequencerData.readptr >= SEQSTEPBUFFERELEMENTS) //test for buffer wraparound
	{
		SequencerData.readptr = 0;
	}
	SequencerData.fillcount--;

	return 1234;
}


uint16_t SeqBufferLoadEntry(void)
{
	//Created 22JUL204
	//Loads Sequencer data from  I2C memory into SRAM data structure.
	uint32_t tempval = 0;
	uint8_t bytebuffer[0x20] = {0};
	uint8_t* byteptr = 0;
	byteptr = &bytebuffer[0];


	tempval = ReadSeqHeader(byteptr); //check header
	if (tempval == 0)
	{
		uint16_t MatAddress = (uint16_t)(*(byteptr+2) << 8) | (uint16_t)*(byteptr + 3);
		uint8_t Quantity = 8;
		//uint8_t ReadSmallI2CDatablock5(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty);
		uint8_t response = 0;
		response = ReadSmallI2CDatablock5(SEQUENCERMEMORY, byteptr, MatAddress + (SequencerData.NextSequenceSteptoBuffer), Quantity);
		if (response == 0)
		{
			//now check MAT entry CRC value
			//uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
			uint16_t CalculateCrc = 0;
			CalculateCrc = CalculateBlockCrc(byteptr, SEQMATENTRYSIZE - 2);
			if ((uint8_t)(CalculateCrc >> 8) != *(byteptr + 6))
			{
				if ((uint8_t)CalculateCrc != *(byteptr + 7))
				{
					//MAT entry CRC tested OK
					uint16_t DataAddress = (uint16_t)(*byteptr << 8) | (uint16_t)(*(byteptr + 1));
					uint16_t DataQuantity = (uint16_t)(*(byteptr + 2) << 8) | (uint16_t)(*(byteptr + 3));
					if (DataQuantity <= I2CREADBLOCKSIZE)
					{
						response = ReadSmallI2CDatablock5(SEQUENCERMEMORY, byteptr, DataAddress, DataQuantity);
						if (response == 0)
						{
							CalculateCrc = CalculateBlockCrc(byteptr, SEQMATENTRYSIZE - 2);
							if ((uint8_t)(CalculateCrc >> 8) != *(byteptr + 6))
							{
								if ((uint8_t)CalculateCrc != *(byteptr + 7))
								{
									//data block tested OK

									tempval = (uint32_t)(*byteptr << 8) | (uint32_t) (*(byteptr));
									SequencerData.SeqStepDataArray[SequencerData.writeptr].time = (uint16_t)tempval;

									tempval = (uint32_t)(*(byteptr + DataQuantity - 2) << 8) | (uint32_t) *(byteptr + DataQuantity - 1);
									SequencerData.SeqStepDataArray[SequencerData.writeptr].paracrc = (uint16_t)tempval;

									uint8_t* destbyteptr = SequencerData.SeqStepDataArray[SequencerData.writeptr].parameter;
									for(uint16_t i=0; i<DataQuantity - 4; i++)
									{
										*(destbyteptr + 2 + i) = *(byteptr + 2 + i);
									}

									SequencerData.writeptr++;
									if (SequencerData.writeptr >= SEQSTEPBUFFERELEMENTS) //test for buffer wraparound
									{
										SequencerData.writeptr = 0;
									}

									SequencerData.NextSequenceSteptoBuffer++;
									if(SequencerData.NextSequenceSteptoBuffer > SequencerData.MaxSteps)
									{
										SequencerData.NextSequenceSteptoBuffer = 0;
									}
									SequencerData.fillcount++;

								}
								else
								{
									tempval = 15;
								}
							}
							else
							{
								tempval = 14;
							}
						}

					}
					else
					{
						tempval = 13;
					}

				}
				else
				{
					tempval = 12;
				}
			}
			else
			{
				tempval = 11;
			}
		}
		else
		{
			tempval = 10;
		}

	}

	if (tempval != 0)
	{
		//could not read header data
		SequencerData.state = 0;
	}
	return tempval;
}


uint16_t SetDefaultOutputs(void)
{
	//Created 22JUL2024
	uint32_t tempval = 0;
	uint8_t bytebuffer[0x10] = {0};
	uint8_t* byteptr = 0;
	byteptr = &bytebuffer[0];

	tempval = ReadSeqHeader(byteptr); //check header
	if (tempval == 0)
	{

	}
	return 1234;
}
