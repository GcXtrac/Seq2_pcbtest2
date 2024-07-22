/*
 * SequencerV1.h
 *
 *  Created on: Jul 22, 2024
 *      Author: GChol
 */

#ifndef INC_SEQUENCERV1_H_
#define INC_SEQUENCERV1_H_





	void SetSequencerState (uint8_t state);
	uint8_t GetSequencerState(void);
	void InitialiseSequencerDataBuffer(void);
	uint16_t GetSeqBufferFillLevel(void);
	void SetSequencerMaxSteps(uint16_t MaxSteps);
	uint16_t SeqBufferReadEntry(void);
	uint16_t SeqBufferLoadEntry(void);



#endif /* INC_SEQUENCERV1_H_ */
