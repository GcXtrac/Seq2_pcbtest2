/*
 * GcI2cV1.h
 *
 *  Created on: 13 Dec 2022
 *  Last edited 17MAY2024
 *      Author: Geoff
 */

#ifndef INC_GCI2CV1_H_
	#define INC_GCI2CV1_H_


//needed to placed the struct definition here so that both main.c & I2c.c understand
//if the struct definition was only in the I2c.C then ONLY i2c.c would understand
	/*
	struct I2cConfig{
		uint16_t I2cInternalAddress;
		uint16_t I2cQuantity;
		uint8_t I2cInternalAddressWidth;
		uint8_t I2cDeviceAddress;
	};
	*/

	uint8_t sizeofI2cDataBuffer(void);
	uint8_t* GetI2cData(void);


	struct I2cConfig GetI2cConfig(void);

	uint32_t CheckHeaderBlock(void);
	uint32_t UpdateSeqHeaderCrc(void);

	void SetI2cDestPtr(uint16_t address);
	void SetI2cSourcePtr(uint16_t address);

	void DecrementI2cTiming(void);
	void ClearI2cStatus(void);
	uint32_t GetI2cStatus(void);
	void ClearI2cStatusBit0(void);
	void SetI2cStatusBit(uint8_t bitpattern);

	void SetI2cDeviceAddress(uint8_t address);
	void SetI2cInternalAddress(uint16_t address);
	void SetI2cBlockSize(uint16_t Qty);
	void SetInternalAddressWidth(uint8_t width);

	uint8_t CopyI2cBlock(void);
	//void ReadI2cConfig(struct I2cConfig* I2cConfig2);
	//void ReadI2cConfig(struct I2cConfig4* I2cConfig2); //structure declared in global-settingsV1.h
	void ReadI2cConfig(struct I2cConfig4* I2cConfig4Ptr); //structure declared in global-settingsV1.h

	void I2cTest(void);

	uint32_t I2cAbort(void);
	uint32_t I2cMemoryFill(uint8_t data);
	uint8_t WriteI2cDisplaydata(uint8_t I2cDevice, uint8_t LcdDataQty, uint8_t* pI2cData);
	uint32_t I2cWriteBlock2Blocking(uint8_t* srcdata, uint8_t qty, uint8_t debugdata);
	uint32_t I2cWriteBlock2(uint8_t* srcdata, uint8_t qty, uint8_t debugdata);
	uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
	uint32_t Write2I2cDevice(uint16_t Setting, uint8_t I2cDeviceAddress2);
	uint32_t I2cWriteByteBlocking(uint16_t IntAddress, uint8_t data, uint8_t debugdata);

	//uint32_t I2cWriteByte(uint16_t IntAddress, uint8_t data);
	uint32_t I2cWriteByte(uint16_t IntAddress, uint8_t data, uint8_t DebugDisp);

	uint8_t* ReadSmallI2CDatablock4(uint8_t control); //returns a pointer to data block, first first element being a status byte,
	uint8_t ReadSmallI2CDatablock3(uint16_t intaddress, uint8_t BlockQty, uint8_t debugdata);
	uint8_t ReadSmallI2CDatablock2(uint8_t debugdata);
	uint8_t ReadSmallI2CDatablock(uint8_t device, uint8_t* pI2cData, uint16_t intaddress, uint8_t BlockQty, uint8_t debugdata);

	uint32_t I2cReadByte(uint16_t IntAddress);
	uint8_t I2cReadBlock(void);
	uint8_t	SendI2cData(uint16_t I2cDeviceAddress, uint8_t* pI2cData, uint8_t LcdDataQty);

	struct I2cConfig{
		uint16_t I2cInternalAddress;
		uint16_t I2cQuantity;
		uint8_t I2cInternalAddressWidth;
		uint8_t I2cDeviceAddress;
	}; //Placing this here allows this definition to be used within  main.c


#endif /* INC_GCI2CV1_H_ */
