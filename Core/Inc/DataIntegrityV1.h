/*
 * DataIntegrityV1.h
 *
 *  Created on: 8FEB2023
 *      Author: Geoff
 */

#ifndef INC_DATAINTEGRITYV1_H_
	#define INC_DATAINTEGRITYV1_H_

	uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
	void SetCrc16Value(uint16_t);
	uint16_t GetCrc16Val(void);
	uint16_t CalculareCrc16(uint8_t data);

#endif /* INC_DATAINTEGRITYV1_H_ */
