# Seq2_pcbtest2

code development for the dedicated PCB for the MK2 sequencer (STM32L476VCT6 device)
Serial output used VT100 strings to format the display
Initial comms using RS422 interface to dedicated UART

separate I2C busses for display and memory
memory interface rehacked for 24LC1026 1mbit memory device (128byte page)

recognised serial commands:

I2CQyyyy:	Set I2C data quantity value
I2CDxx:		Set I2C device address
I2CAaaaa:	Set I2C internal address
I2CAy:		Set number of I2C address bytes
I2CWaadd	Write single byte to I2C device
I2CR:		Read block of data from I2C device
CH:		Check header memory area
XFy: 		Set X-modem received data format
		y=0: expect basic text
		y=1: expect Intel hex data
		y=2: expect Sequencer specific data
CSM: 		Clear sequencer memory
XR:		Enable X-modem receive function
