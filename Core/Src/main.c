/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sequencer Mk2 PCB test
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "global-settingsV1.h" //global #defines
#include "GcI2cV1.h"
#include "GcFunctionsV1.h"
#include "DataIntegrityV1.h"
#include "SequencerV1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLAG_SET  1
#define FLAG_CLEAR  0
#define TIMER1PERIOD  1000
#define TIMER1DACUPDATECOUNT 1
#define UARTUPDATEPERIOD 2000
#define RXBUFFERLENGTH 300
#define ESCAPEDISPLAYPERIOD 500
#define OPBUFFERSIZE 200
#define MAXLINELENGTH 16


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile static uint8_t timer1heartbeat = FLAG_CLEAR; //set in timer 1 ISR, read and reset in main loop
volatile static uint8_t timer1heartbeat2 = FLAG_CLEAR; //set in timer 1 ISR, read and reset in main loop
volatile static uint8_t CanDataReceived = FLAG_CLEAR; //set by CAN receive callback, read & reset by mainloop
volatile static uint8_t CanState = 0;
volatile static uint8_t UartOutputFlag = FLAG_CLEAR; //flag normally set by TIM ISR and cleared within the main loop
volatile static uint8_t UartMsgSent = FLAG_CLEAR; //flag normally set when UART data is transmitted and cleared when transmission is completed

static uint8_t CanRxData[8] = {};


volatile char RxBuffer1[2] = "";
volatile char RxBuffer2[RXBUFFERLENGTH] = "";
//volatile static uint8_t UartRxData = FLAG_SET;
volatile static uint8_t RxState = 0; //UART3 state bits:
										//0: character arrived
										//1: buffer overflow
										//2: new string
										//3: escape character detected
										//4: clear escape character message from VT100 screeen
										//5: set when received struing is to be processed

volatile static uint16_t RxWritePtr = 0;
volatile static uint16_t RxBufferCount = 0;

volatile static uint16_t EscapeClearCount = 0;

volatile static uint16_t FunctionDelay = 0; //value decremented by TIM ISR

volatile static uint8_t XmodemStatus = 0;

//trying here for the benefit of stm studio...
uint16_t DacVal = 0;
static uint16_t timer1count = TIMER1PERIOD;
uint8_t mainloopcount = 0;

//CAN Rx
CAN_RxHeaderTypeDef CanRxHeader = {};
CAN_RxHeaderTypeDef * pCanRxHeader = &CanRxHeader;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

enum SeqDecode {
				NONE = 0,
				STEP = 1,
				TIME = 2,
				DIGITALOUT = 3,
				ANALOGOUT = 4,
				SEQHEADER = 5,
				SEQCYCLECOUNT = 6,
				COMMENT = 7,
			};


int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint8_t mainloopcount = 0; //mobve for trhe benefit of stmm studio...
	//uint16_t DacVal = 0;
	uint16_t DacVal2 = 0xFFF;
	uint8_t index = 0;
	//uint8_t rxstringlength = 0;
	char RxString[RXBUFFERLENGTH] = "";

	uint8_t RxStringLen = 0;
	uint16_t RxReadPtr = 0;

	char tmpstr[200] = "";
	char tempstring[200] = "";


	uint16_t CanIdentifier = 0;
	uint8_t CanDlc = 0;
	uint8_t CanData[8] = {0};

	uint8_t CanTxMsgCount = 0; //generic CAN TX message counter
	uint8_t CanTxMailboxfullmsg = FLAG_CLEAR;  //flag set when CAN tx mailbox is full, this prevents repeated output
	uint8_t candatacount = 0;

	uint8_t ProcessRececivedCanData = FLAG_CLEAR;

	uint8_t I2cInitialisationFunction = 0;
	uint8_t I2cReadBlockFunction = 0; //Main loop function controlling variable

	uint8_t recognisedstring = FLAG_CLEAR;

	uint32_t UserVal = 0;
	uint8_t screenblock = FLAG_CLEAR;

	uint8_t processloopcount = 0;
	uint8_t Xmodempoacketcount = 0;
	uint8_t xmodempacketblockno = 0;

	uint8_t Tempdata[0x400] = {0};
	uint16_t opwriteptr = 0;
	uint16_t opreadptr = 0;
	uint16_t opbytecount = 0;
	uint8_t linestring[128] = {0}; //This needs to be large enough to hold a complete x-modem packet containing no CR characters
	uint8_t linecharcount = 0;
	uint8_t linecount = 0;
	uint8_t linearray[0x400] = {0};
	uint8_t dataformat = 1;	//Set X modem data format
							//0:text
							//1: intel hex data

	//used for Intel hex data decoding/buffering
	uint8_t bytearray[16] = {0x00};
	uint8_t* byteptr = &bytearray[0];

	uint32_t address = 0;
	uint32_t bytecount = 0;
	uint8_t writeattempt = 0;

	uint8_t FillI2cMemoryFunction = 0; //Main loop function controlling variable
	uint16_t blockcount = 0;

	uint16_t StepIndex = 0;
	uint16_t SeqCmdError = 0;
								//0x10 problem obtaining header data
								//0x11 received sequencer index specifies data that already exists
								//0x20 problem storing sequencer command string.


	 uint16_t SeqStepIndex = 0;
	 uint16_t MaxSequencerCycles = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);


  //CAN Tx
  CAN_TxHeaderTypeDef CanTxHeader = {};
  CAN_TxHeaderTypeDef * pCanTxHeader = &CanTxHeader;
  pCanTxHeader->DLC = 8;
  pCanTxHeader->IDE = CAN_ID_STD;
  pCanTxHeader->RTR = CAN_RTR_DATA;
  //pCanTxHeader->StdId = 0x234; //standard identifier
  pCanTxHeader->StdId = 0x345; //standard identifier used for loop back mode
  //pCanTxHeader->ExtId = 0x01; //force TXREQ bit!!!
  pCanTxHeader->ExtId = 0x00; //CAN Tx function should add the Tx request bit...
  pCanTxHeader->TransmitGlobalTime = 0;





  CAN_FilterTypeDef FilterConfig;
  CAN_FilterTypeDef* pFilterConfig = &FilterConfig;
  pFilterConfig->FilterIdLow = 0x123u<<5;
  //pFilterConfig->FilterIdHigh = 0x345u;
  //pFilterConfig->FilterIdHigh = 0x123u<<5;
  pFilterConfig->FilterIdHigh = 0x000u<<5; //received everything
  pFilterConfig->FilterActivation = CAN_FILTER_ENABLE;
  pFilterConfig->FilterBank = 0;
  pFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
  //pFilterConfig->FilterMaskIdHigh = 0x345u<<5;
  pFilterConfig->FilterMaskIdHigh = 0x000u<<5;
  pFilterConfig->FilterMaskIdLow = 0x345u<<5;
  pFilterConfig->FilterMode = CAN_FILTERMODE_IDLIST;
  pFilterConfig->FilterScale = CAN_FILTERSCALE_16BIT;
  //pFilterConfig->SlaveStartFilterBank = 0;
  //pFilterConfig->SlaveStartFilterBank = 14;


//  //Prepare for CAN reception
//  //Configure CAN FIFO 0, filter 0 & 1.
//  //CAN_FilterTypeDef CanFilterConfig
//  CanFilterConfig.FilterIdHigh = 0x102<<5; //set message identifier to look for, was 0x321<<5
//  CanFilterConfig.FilterIdLow = 0x102<<5;  //was 0x321<<5
//
//  //mask identifier - default type of filter is a list NOT mask
//  CanFilterConfig.FilterMaskIdHigh = CAN_MSG_ID_DIAG_REQUEST<<5; //set message identifier to look for, was 0x321<<5;
//  CanFilterConfig.FilterMaskIdLow = CAN_MSG_ID_DIAG_REQUEST<<5;  //was 0x321<<5;
//
//  CanFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //variable type: CAN_filter_FIFO (FIFO 0)
//  CanFilterConfig.FilterBank = 0;//specifies which filter bank will be initialised
//  CanFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; //variable type: CAN_filter_modeTjis controls bit FBMx=1
//  CanFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; //variable type: CAN_filter_scale (2off 16bit filters) This controls bit FSCx=0
//  CanFilterConfig.FilterActivation = CAN_FILTER_ENABLE; //variable of type: CAN_filter_activation
//  PtrCanFilterConfig = &CanFilterConfig; //set pointer to point to variable
//
//  HAL_CAN_ConfigFilter(&hcan1, PtrCanFilterConfig); //Configure the CAN reception filters (HAL CAN configuration function)


  uint8_t CanFilterErr = 0;
  if (HAL_CAN_ConfigFilter(&hcan1, pFilterConfig) != HAL_OK)
  {
	  //problem with CAN filter setup!
	  CanFilterErr = 0xff;
  }

  //HAL_StatusTypeDef HAL_CAN_ActivateNotification (CAN_HandleTypeDef * hcan, uint32_t ActiveITs)
  uint8_t CanError = 0;
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR) != HAL_OK)
  {
	  CanError = 0xff;
  }
  //what is the difference between the HAL_CAN_ENABLE_IT() macro and the HAL_CANM_ActivateNotification() function????



  HAL_CAN_ActivateNotification (&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  CanFilterErr = 0xfe;
  }


  HAL_TIM_Base_Start_IT(&htim1);

//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE); //Enable UART Transmit Data register empty interrupt
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR); //enable UART error interrupt
//

  //HAL_UART_Receive_IT(&huart1, (uint8_t *) RxBuffer1, 2); //this enables UART reception interrupt
  HAL_UART_Receive_IT(&huart3, (uint8_t *) RxBuffer1, 1); //this enables UART reception interrupt


  //__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_FULL | CAN_IT_ERROR);

  //clear entire screen
  strcpy(tempstring, "\e[2J");
  uint16_t stringlength = strlen(tempstring);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
  UartMsgSent = FLAG_SET;


  /*
  struct I2cConfig{
  	uint16_t I2cInternalAddress;
  	uint16_t I2cQuantity;
  	uint8_t I2cInternalAddressWidth;
  	uint8_t I2cDeviceAddress;
  }structI2cConfig;  //The 'tructI2cConfig' creates an instance of the structure
*/



  struct I2cConfig tempstruct; //create instance of structure defined in GcI2cV1.h
  //struct structI2cConfig tempstruct;
  //structI2cConfig tempstruct;

  tempstruct = GetI2cConfig();

  //structptr->I2cDeviceAddress = 0x98; //this line doesn't work!
  uint8_t x = 0;
  //x = GetI2cConfig().I2cDeviceAddress;
  x = tempstruct.I2cDeviceAddress;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (GetSequencerState() == 1)
	  {
		  if (GetSeqBufferFillLevel() != SEQSTEPBUFFERELEMENTS)
		  {
			  //Load sequencer buffer
			  //Sequencer process won't run until the buffer contains data
			  SeqBufferLoadEntry();

		  }



		  if (GetSeqBufferFillLevel() != 0)
		  {
			  //at least one entry exists within the sequencer buffer memory
			  //sequencer process start...
			  //decode the first entry in the buffer memory, set ioutputs and start step timer
		  }
	  }


	  if (XmodemStatus != 0)
	  {
		if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		{
			  switch (XmodemStatus & 0x0F)
			  {
				  case(1):
					FunctionDelay = 1000; //value decremented by TIM ISR
					XmodemStatus = 2;
					break;

				  case(2):
					if (FunctionDelay == 0)
					{
						sprintf(tmpstr, "\e[3;1H\e[KPress button to initiate xmodem comms");
						strcpy(tempstring, tmpstr);

						uint16_t stringlength = strlen(tempstring);

						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;

						//clear output buffer
						for (uint16_t i=0; i<0x400; i++)
						{
							Tempdata[i] = 0;
						}

						XmodemStatus = 3;
						Xmodempoacketcount = 0;
						xmodempacketblockno = 1; //first xmodem block is labelled 1!
						opwriteptr = 0;	//reset output data pointer
						opreadptr = 0;
						opbytecount = 0;
						linecharcount = 0;
						linecount = 0;
					}
					break;

				  case(3):
					//waiting for user to press button to initiate x-modem comms
					break;

				  case(4):

						if (FunctionDelay == 0) //test for timeout expiry
						{
							uint8_t tempval = 0;
							tempval = RxBufferCount; //obtain number of characters held in the 2ndary receive buffer
							//tempval = opwriteptr;

							sprintf(tmpstr, "\e[3;1H\e[KX-modem timeout expired");
							strcpy(tempstring, tmpstr);

							uint16_t stringlength = strlen(tempstring);

							//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
							HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
							UartMsgSent = FLAG_SET;
							XmodemStatus = 5;
							FunctionDelay = 1000; //value decremented by TIM ISR

						}
//						if((XmodemStatus & 0x10) != 0)
//						{
//							//x-modem packet received, do something with it
//							//also reset xmodem timeout period
//							FunctionDelay = 5000; //value decremented by TIM ISR
//						}

						//tempval = RxStringLen; //obtain number of characters held in the 2ndary receive buffer
						break;

				  case(5):
					if (FunctionDelay == 0)
					{
						XmodemStatus = 7; //terminate x-modem function
					}
					break;

				  case(6):
					//we get here if the last xmodem packet termination character has been received
					tempstring[0] = 0x06; //ACK
					tempstring[1] = 0; //string terminator


					__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //re-enable receive interrupts


					//send initiation character for start of X-modem transfer
					//strcpy(tempstring, 0x06); //ACK
					//strcpy(tempstring, 0x05); //NAK

					//tempstring[0] = 0x06; //ACK
					//tempstring[1] = 0; //string terminator

					stringlength = strlen(tempstring);
					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;

					FunctionDelay = 5000; //value decremented by TIM ISR
					XmodemStatus = 7; //terminate x-modem function
					break;

				  case(7):
					if (FunctionDelay == 0)
					{
						sprintf(tmpstr, "\e[3;1H\e[KX-modem download finished"); //clear line
						strcpy(tempstring, tmpstr);

						uint16_t stringlength = strlen(tempstring);

						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;
						XmodemStatus = 0;
					}
					break;

				  case(8):
					//We get here if a complete X-modem packet has been received
					//time to decode the packet held in variable 'RxString'

					//now test packet data integrity before issuing ACK or NAK response
					Xmodempoacketcount++;
				  	SetCrc16Value(0);
					if (RxString[0] == 0x01) //test for SOH (start of header) character
					{
						if (RxString[1] == xmodempacketblockno)
						{
							if (RxString[2] == (xmodempacketblockno ^ 0xFF))
							{
								xmodempacketblockno++;

								uint8_t data = 0;
								//for (uint8_t i=0; i<RxStringLen; i++)
								for (uint8_t i=0; i<128; i++)
								{
								  data = RxString[i+3];
								  CalculareCrc16(data);
								  Tempdata[opwriteptr] = data;
								  opwriteptr++;
								  if (opwriteptr >= OPBUFFERSIZE)
								  {
									  opwriteptr = 0;
								  }
								  opbytecount++;
								}

								//test calculated CRC with received value
								uint16_t tempval = GetCrc16Val();
								if ((uint8_t)(tempval >> 8) == RxString[131])
								{
									if ((uint8_t)(tempval) == RxString[132])
									{
										//CRC value calculated/received correctly
										tempval = 1;

									}
									else
									{
										tempval = 0;
									}
								}
								else
								{
									tempval = 0;
								}

								if (tempval == 0)
								{
									//Packet CRC test failed, prepare to send NAK to get a new copy of the current packet
									RxBufferCount = 0; //reset buffer count ready for next packet


									__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //re-enable receive interrupts

									tempstring[0] = 0x15; //NAK
									tempstring[1] = 0; //string terminator


									stringlength = strlen(tempstring);
									//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
									HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
									UartMsgSent = FLAG_SET;

									//reset timeout period
									FunctionDelay = 5000; //value decremented by TIM ISR
									XmodemStatus = 4; //wait for further data to arrive
								}
								else
								{
									//X-modem packet CRC tested OK
									XmodemStatus = 9;

									//tempstring[0] = 0x06; //ACK
									//tempstring[1] = 0; //string terminator
								}
							}
						}
					}




					//send initiation character for start of X-modem transfer
					//strcpy(tempstring, 0x06); //ACK
					//strcpy(tempstring, 0x05); //NAK

					//tempstring[0] = 0x06; //ACK
					//tempstring[1] = 0; //string terminator

					//RxBufferCount = 0; //reset buffer count ready for next packet

					//stringlength = strlen(tempstring);
					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					//HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					//UartMsgSent = FLAG_SET;



					//reset timeout period
					//FunctionDelay = 5000; //value decremented by TIM ISR
					//XmodemStatus = 4; //wait for further data to arrive
					break;


				  case(9):
					//X-modem packet has just been received
					//read out CR terminated strings
					uint8_t data = 0;
					while (opbytecount > 0)
					{
						data = Tempdata[opreadptr];

						//eliminatge LF character
						uint8_t processlinechar = 1;
						if (linecharcount == 0)
						{
							if (data == 10) //test for Line feed (LF) character
							{
								processlinechar = 0;
							}
						}

						if (processlinechar == 1) //ignore first character if it is a LF character
						{
							linestring[linecharcount] = data; //reconstruct line data
							linecharcount++;
						}

						opreadptr++;
						if (opreadptr >= OPBUFFERSIZE) //test for wrap around
						{
							opreadptr = 0;
						}
						opbytecount--; //decrement packet byte count

						if (data == 0x0d)
						{

							if (linecount < 64)
							{
								for (uint8_t i=0; i<linecharcount; i++)
								{
									if (i == (MAXLINELENGTH - 1))
									{
										break;
									}
									linearray[(linecount*MAXLINELENGTH) + i] = linestring[i];
								}
							}
							linecount++; //advance line counter


							break;
						}

					}

					XmodemStatus = 10;
					break;


				  case(10):
					//complete data line stripped from x-modem packet / remaining packet bytes are buffered as part of a new line
					//line characters are held in buffer 'linestring[]'
					//number of characters on line is specified by 'linecharcount'
					uint8_t lineprocessing = 0;
					if (linestring[linecharcount-1] == 0x0d) //test for complete line
					{
						//determine data format
						if (dataformat == 0)
						{

						}

						else if (dataformat == 1)
						{
							//process line as intel hex data
							if (linestring[0] == ':') //test for intel hex start of line character
							{

								//ExtractValueFromString(char* cmd_String, uint8_t charOffset, uint8_t charqty);
								uint8_t checksum = 0;
								bytecount = ExtractValueFromString((char*)linestring, 1, 2); //extract byte count from linestring
								checksum = checksum + (uint8_t)bytecount;

								address = ExtractValueFromString((char*)linestring, 3, 4); //extract destination address
								checksum = checksum + (uint8_t)(address >> 8);
								checksum = checksum + (uint8_t)address;

								uint32_t record = ExtractValueFromString((char*)linestring, 7, 2); //extract destination address
								checksum = checksum + (uint8_t)record;

								uint32_t bytevalue = 0;
								uint32_t recdchecksum = ExtractValueFromString((char*)linestring, 9 + (2*(uint8_t)bytecount), 2);
								for (uint8_t i=0; i<(uint8_t)bytecount; i++)
								{
									bytevalue = ExtractValueFromString((char*)linestring, 9 + (2*i), 2);
									checksum = checksum + (uint8_t)bytevalue;
									bytearray[i] = (uint8_t)bytevalue;

								}
								checksum = checksum ^ 0xFF;
								checksum = checksum + 1;
								if (checksum == (uint8_t)recdchecksum)
								{
									writeattempt = 3;
									XmodemStatus = 12;
									lineprocessing = 1;



								}
								else
								{
									//there is a problem with the Intel hex data - terminate transfer at this point...
								}
							}

						}
						else if (dataformat == 2)
						{
							//Sequencer configuration data

							enum SeqDecode SeqConfigDecode;
							uint8_t valcharcount = 0;
							//uint8_t addtovaluestring = 0;
							uint8_t cmdcharcount = 0;

							uint8_t commanddata[CMDMAXLENGTH + 2] = {0};
							uint8_t cmddatacounter = 0;


							for (uint8_t i=0; i<linecharcount; i++)
							{
								if (linestring[i] == ';') //test for line comment
								{
									SeqConfigDecode = COMMENT;


									//store command string to I2C memory

									//obtain next available memory location
									uint32_t response = 0;
									uint16_t address = 0;
									uint8_t error = 0;
									response = ReadSeqMat(StepIndex - 1);
									if ((response >> 24) == 0)
									{
										address = (uint16_t)response;

										//generate data CRC value
										SetCrc16Value(0);
										uint16_t tempval = 0;
										tempval = CalculateBlockCrc(&commanddata[0], cmddatacounter);
										commanddata[cmddatacounter + 1] = (uint8_t)(tempval >> 8);
										commanddata[cmddatacounter + 2] = (uint8_t)tempval;

										response = WriteSeqData(&commanddata, cmddatacounter + 2, address);

									}
									if (response == 0)
									{
										//update MAT entry
										*byteptr = (uint8_t)(address >> 8);
										*(byteptr+1) = (uint8_t)address;
										*(byteptr+2) = 0;
										*(byteptr+3) = cmddatacounter + 2;
										*(byteptr+4) = FILLVALUE;
										*(byteptr+5) = FILLVALUE;

										SetCrc16Value(0);
										uint16_t tempval = 0;
										tempval = CalculateBlockCrc(*byteptr, SEQMATENTRYSIZE-2);
										*(byteptr+6) = (uint8_t)(tempval >> 8);
										*(byteptr+7) = (uint8_t)tempval;

										response = I2cWriteBlock(SEQUENCERMEMORY, MATTABLEADDR + (StepIndex * SEQMATENTRYSIZE), 2, byteptr, SEQMATENTRYSIZE);
									}
									else
									{
										error = 1;
									}

									if (response == 0)
									{
										//MAT entry was updated successfully
										StepIndex++;

										//now update sequencer header data
										response = ReadSeqHeader(byteptr);
										if (response == 0)
										{


											*byteptr = (uint8_t)(StepIndex >> 8);
											*(byteptr+1) = (uint8_t)StepIndex;

											response = UpdateHeaderBlock(byteptr);


										}
										else
										{
											error = 2;
										}
									}
									if (error != 0)
									{
										SeqCmdError = 20;	//stop further decoding of sequencer command strings
									}

								}




								if (SeqConfigDecode != COMMENT)
								{
									if (cmddatacounter < CMDMAXLENGTH)
									{
										commanddata[cmddatacounter] = linestring[i];
										cmddatacounter++;
									}

									if (SeqConfigDecode == NONE)
									{
										if (cmdcharcount == 0)
										{
											if (linestring[i] == 'A')
											{
												cmdcharcount++;
												valcharcount = 0;
											}

											if (linestring[i] == 'C')
											{
												cmdcharcount++;
												valcharcount = 0;
											}

											if (linestring[i] == 'D')
											{
												cmdcharcount++;
												valcharcount = 0;
											}

											if (linestring[i] == 'H')
											{
												cmdcharcount++;
												valcharcount = 0;
											}

											if (linestring[i] == 'S')
											{
												SeqConfigDecode = STEP;
												cmdcharcount++;
												valcharcount = 0;
											}

											if (linestring[i] == 'T')
											{
												SeqConfigDecode = TIME;
												cmdcharcount++;
												valcharcount = 0;
											}
										}

										else if (cmdcharcount == 1)
										{
											if (linestring[i-1] == 'A')
											{
												if (linestring[i] == 'd')
												{
													SeqConfigDecode = ANALOGOUT;
													cmdcharcount++;
												}
											}

											if (linestring[i-1] == 'C')
											{
												if (linestring[i] == 'Y')
												{
													cmdcharcount++;
												}
											}

											if (linestring[i-1] == 'D')
											{
												if (linestring[i] == 'D')
												{
													SeqConfigDecode = DIGITALOUT;
													cmdcharcount++;
												}
											}

											if (linestring[i-1] == 'H')
											{
												if (linestring[i] == 'E')
												{
													cmdcharcount++;
												}
											}

										}
										else if (cmdcharcount == 2)
										{
											if (linestring[i-1] == 'Y')
											{
												if (linestring[i] == 'C')
												{
													SeqConfigDecode = SEQCYCLECOUNT;
													cmdcharcount++;
												}
											}

											if (linestring[i-1] == 'E')
											{
												if (linestring[i] == 'A')
												{
													cmdcharcount++;
												}
											}
										}


										else if (cmdcharcount == 3)
										{

											if (linestring[i-1] == 'A')
											{
												if (linestring[i] == 'D')
												{
													SeqConfigDecode = SEQHEADER;
													cmdcharcount++;
												}
											}
										}
									}
								}


								else
								{
									//command string already detected
									if (linestring[i] != 0x20)
									{
										valcharcount++;
									}
									else
									{
										//end of value detected
										uint32_t Response = 0;


										//ExtractValueFromString((char*) linestring, i - valcharcount, valcharcount);
										UserVal = ExtractValueFromString(RxString, cmdcharcount, valcharcount);
										if ((UserVal & 0x80000000) == 0)
										{
										//now do something with value
										//write to memory block
										//update MAT

											if (SeqConfigDecode == STEP)
											{
												//check that specified step doesn't already exist
												//check that step number falls outside steps specified in sequencer header
												Response = ReadSeqHeader(byteptr);
												if (Response == 0)
												{
													//read sequencer header data
													uint16_t tempval = 0;
													tempval = (uint16_t)(*(byteptr) << 8) | (uint16_t)*(byteptr + 1);
													if ((UserVal & 0xFFFF) >= tempval)
													{
														SeqCmdError = 0x11;
													}


												}
												else
												{
													//STEP error
													SeqCmdError = 0x10;
												}

											}


											if (SeqConfigDecode == SEQCYCLECOUNT)
											{
												*byteptr = (uint8_t)UserVal>>8;
												*(byteptr + 1) = (uint8_t)UserVal;
												//uint8_t bytearray[16] = {0x00};
												//uint8_t* byteptr = &bytearray;


												//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
												Response = I2cWriteBlock(SEQUENCERMEMORY, 0x0026, 2, byteptr, 2);
												if (Response != 0)
												{
													Response = 3;
												}

											}

											Response = UpdateSeqHeaderCrc(); //update header data block held in I2C memory
											if (Response != 0)
											{
												Response = 4;
											}

										}
										else
										{
											Response = 1;
										}

									}

								}

							}
						}

						linecharcount = 0;
					}




					if (lineprocessing == 0) //check for line data processing completed.
					{
						if (SeqCmdError != 0) //determine if a problem has been found with most recent received data
						{
							XmodemStatus = 5; //terminate further X-modem comms
						}

						if (opbytecount != 0)
						{
							//further xmodem packet bytes need to be fed through the linestring buffer
							XmodemStatus = 9;
						}
						else
						{
							//all packet data has been fed into the linestring buffer
							//linstring buffer my hold an incomplete line which will be completed with the next x-modem packet
							XmodemStatus = 11;
						}
					}
					break;


				  case(11):
					//x-modem packet has just been processed
					RxBufferCount = 0; //reset buffer count ready for next packet

					tempstring[0] = 0x06; //ACK
					tempstring[1] = 0; //string terminator

					__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //re-enable receive interrupts

					stringlength = strlen(tempstring);
					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;



					//reset timeout period
					FunctionDelay = 5000; //value decremented by TIM ISR
					XmodemStatus = 4; //wait for further data to arrive
					break;


				  case(12):
					//Write Intel hex data block to destination memory

					//byteptr = &bytearray;
					//intel hex string is valid: computed checksum matches received checksum
					//now write to target memory....
					//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty)
					uint32_t I2cWriteState = 0;
					I2cWriteState = I2cWriteBlock(0xA0, (uint16_t)address, 2, byteptr, (uint8_t)bytecount);

					//need to check at some point that device write has completed.....
					if (I2cWriteState == 1)
					{
						//I2C device was busy - unable to write block of data...
						//prepare to try another write attempt
						writeattempt--;
						if (writeattempt == 0)
						{
							//terminate X-modem transfer
							FunctionDelay = 5000;
							XmodemStatus = 7;
						}
					}
					else
					{

					  	if (opbytecount != 0)
					  	{
					  		//further xmodem packet bytes need to be fed through the linestring buffer
					  		XmodemStatus = 9;
					  	}
					  	else
					  	{
					  		//all packet data has been fed into the linestring buffer
					  		XmodemStatus = 11;
					  	}
					}
					break;

				  default:
					XmodemStatus = 0;
			  }
	  	  }

	  }


	  if (FillI2cMemoryFunction != 0)
	  {
			//main loop function initiated by serial command "FILLxx"
			switch(FillI2cMemoryFunction)
			{
				case(1):
					if (FunctionDelay == 0)
					{
						tempstruct = GetI2cConfig();
						if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
						{
							sprintf(tmpstr, "\e[4;1H\e[KDevice address:0x%02X", tempstruct.I2cDeviceAddress); //move cursor to 3rd line, clear text,
							strcpy(tempstring, tmpstr);
							sprintf(tmpstr, "\e[5;1H\e[KSource address:0x%04X", tempstruct.I2cInternalAddress);
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\e[6;1H\e[KQuantity:0x%04X", tempstruct.I2cQuantity);
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\e[7;1H\e[KData fill:0x%02X", *byteptr);
							strcat(tempstring, tmpstr);

							uint16_t stringlength = strlen(tempstring);
							//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
							HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
							UartMsgSent = FLAG_SET;


							FunctionDelay = 1000; //value decremented by TIM ISR
							FillI2cMemoryFunction = 2;
							address = 0;
						}
					}
					break;

				case(2):
					if (FunctionDelay == 0)
					{
						if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
						{
							if (tempstruct.I2cQuantity != 0)
							{
								uint8_t currentblock = 0;
								if(tempstruct.I2cQuantity > BLOCKSIZE)
								{
									currentblock = BLOCKSIZE;
								}
								else
								{
									currentblock = tempstruct.I2cQuantity;
								}
								blockcount++;

								for (uint8_t i = 0; i<BLOCKSIZE-1; i++)
									*(byteptr + 1 + i) = *byteptr;

								uint32_t response = 0;
								//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
								response = I2cWriteBlock(SEQUENCERMEMORY, address, 2, byteptr, currentblock);
								if (response == 0)
								{
									address = address + currentblock;
									if (address == tempstruct.I2cQuantity)
									{
										//block fill is complete
										FillI2cMemoryFunction = 3;
									}
									else
									{
										//continue around this loop until the specified block has been filled
										sprintf(tmpstr, "\e[7;1H\e[KBlock count:0x%02X", blockcount);
										strcpy(tempstring, tmpstr);

										uint16_t stringlength = strlen(tempstring);
										//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
										HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
										UartMsgSent = FLAG_SET;
									}
								}
								else
								{
									FillI2cMemoryFunction = 4;
								}
							}
							else
							{
								sprintf(tmpstr, "\e[6;1H\e[KQuantity:0x%04X", tempstruct.I2cQuantity);
								strcpy(tempstring, tmpstr);

								uint16_t stringlength = strlen(tempstring);
								//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
								HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
								UartMsgSent = FLAG_SET;
							}
						}
					}
					break;

				case(3):
					if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
					{
						sprintf(tmpstr, "\e[6;1H\e[KBlock fill completed");
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "\e[7;1H\e[K"); //clear line 7
						strcpy(tempstring, tmpstr);


						uint16_t stringlength = strlen(tempstring);
						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;

						FillI2cMemoryFunction = 0; //terminate the function
					}
					break;

				case(4):
					if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
					{
						sprintf(tmpstr, "\e[6;1H\e[KBlock fill FAILED");
						strcpy(tempstring, tmpstr);
						FillI2cMemoryFunction = 0; //terminate the function
						uint16_t stringlength = strlen(tempstring);
						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;
					}
					break;

				default:


//				if (UartMsgSent == FLAG_CLEAR)
//				{
//					uint16_t stringlength = strlen(tempstring);
//					if (stringlength != 0)
//					{
//						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
//						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
//						UartMsgSent = FLAG_SET;
//					}
//				}
			}
	  }

	  if (I2cReadBlockFunction != 0)
	  {
		  //result of "I2CR" serial command; outputs block of I2c data to the display
		  //use setup commands beforehand:
		  //I2CDxx
		  //I2CAxxxx
		  //I2cQxxx
		  //I2CAx
		  //uint8_t* byteptr; //used to point to data returned from I2C read function
		  switch(I2cReadBlockFunction)
		  {
		  	  case(1):
				//initialisation
				//display pointers
				if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				{
					screenblock = FLAG_SET;

					//clear message from display

					tempstruct = GetI2cConfig();
					sprintf(tmpstr, "\e[4;1H\e[KDevice address:0x%02X", tempstruct.I2cDeviceAddress); //move cursor to 3rd line, clear text,
					strcpy(tempstring, tmpstr);
					sprintf(tmpstr, "\e[5;1H\e[KSource address:0x%04X", tempstruct.I2cInternalAddress);
					strcat(tempstring, tmpstr);
					sprintf(tmpstr, "\e[6;1H\e[KQuantity:0x%04X", tempstruct.I2cQuantity);
					strcat(tempstring, tmpstr);

					uint16_t stringlength = strlen(tempstring);

					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;
					FunctionDelay = 1000; //value decremented by TIM ISR
					I2cReadBlockFunction = 2;
				}
				break;

			case(2):
				//wait for message to be displayed
				if (FunctionDelay == 0)
				{
				  I2cReadBlockFunction = 3;
				}
				break;

			case(3):
				if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				{
					byteptr = ReadSmallI2CDatablock4(0); //initiate block read (reset block pointer)
					if ((*byteptr & 0xFE) == 0)
					{
						sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "I2C data block read - now to display...");
						strcat(tempstring, tmpstr);

						sprintf(tmpstr, "\e[8;1H\eK");
						strcat(tempstring, tmpstr);
						sprintf(tmpstr, "block address:0x%04X, block size:0x%02X, state:0x%02X,", (*(byteptr+1)<<8)|(*(byteptr+2)), *(byteptr+3), *byteptr);
						strcat(tempstring, tmpstr);


						I2cReadBlockFunction = 4;

					}
					else
					{
						sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "I2C data block read reports error: 0x%02X", *byteptr);
						strcat(tempstring, tmpstr);
						I2cReadBlockFunction = 0; //Disable this main loop function
					}
					uint16_t stringlength = strlen(tempstring);

					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;
					//FunctionDelay = 1000; //value decremented by TIM ISR

				}
				break;

			case(4):
				if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				{
					sprintf(tmpstr, "\e[9;1H\eK");
					strcpy(tempstring, tmpstr);
					sprintf(tmpstr, "0x%04X: 0x%02X 0x%02X  ", (*(byteptr+1)<<8)|(*(byteptr+2)), *(byteptr+3), *byteptr);
					strcat(tempstring, tmpstr);

					uint8_t tempqty = *(byteptr+3);
					for (uint8_t i = 0; i<tempqty; i++) //format read I2C data bytes
					{
						sprintf(tmpstr, "%02X ", *(byteptr+4+i) );
						strcat(tempstring, tmpstr);
					}

					uint16_t stringlength = strlen(tempstring);

					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;


					if (*byteptr == 0x01)
					{
						//specified data byte quantity has been read
						I2cReadBlockFunction = 0;
					}
					else
					{
						I2cReadBlockFunction = 5;
					}
					processloopcount = 0;
				}
				break;

			case(5):
				if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				{
					uint8_t* byteptr;
					byteptr = ReadSmallI2CDatablock4(1); //read another small block from memory device
					if ((*byteptr & 0xFE) == 0)
					{
						if (*(byteptr+3) != 0) //check to see if a number of bytes have been read
						{
							//sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
							//strcpy(tempstring, tmpstr);
							//sprintf(tmpstr, "I2C data block read - now to display...");
							//strcat(tempstring, tmpstr);

							sprintf(tmpstr, "\e[%d;1H\eK", 10+processloopcount);
							strcpy(tempstring, tmpstr);
							sprintf(tmpstr, "0x%04X: 0x%02X 0x%02X  ", (*(byteptr+1)<<8)|(*(byteptr+2)), *(byteptr+3), *byteptr);
							strcat(tempstring, tmpstr);
							uint8_t tempqty = *(byteptr+3);
							for (uint8_t i = 0; i<tempqty; i++)
							{
								sprintf(tmpstr, "%02X ", *(byteptr+4+i) );
								strcat(tempstring, tmpstr);
							}

							if (*byteptr == 0x01)
							{
								I2cReadBlockFunction = 0;
							}
							else
							{
								I2cReadBlockFunction = 5; //prepare to read another small block
							}
							processloopcount++;
						}
						else
						{
							//no data to be displayed.
							I2cReadBlockFunction = 0;
						}

					}
					else
					{
						//sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
						sprintf(tmpstr, "\e[%d;1H\eK", 10+processloopcount);
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "I2C data block read reports error: 0x%02X", *byteptr);
						strcat(tempstring, tmpstr);
						I2cReadBlockFunction = 0;
					}
					uint16_t stringlength = strlen(tempstring);

					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;
					//FunctionDelay = 1000; //value decremented by TIM ISR

				}
				break;


			default:
				I2cReadBlockFunction = 0; //Disable this main loop function

		  }


	  }

	  if (I2cInitialisationFunction == 1)
	  {
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {

			  sprintf(tempstring, "\e[4;1HI2C initialising...");
			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
			  I2cInitialisationFunction = 2;
			  FunctionDelay = 1000; //value decremented by TIM ISR
		  }
	  }

	  if (I2cInitialisationFunction == 2)
	  {
		  //wait for message to be displayed
		  if (FunctionDelay == 0)
		  {
			  I2cInitialisationFunction = 3;
		  }
	  }

	  if (I2cInitialisationFunction == 3)
	  {
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {
			  //clear message from display
			  sprintf(tempstring, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
			  I2cInitialisationFunction = 0;
		  }
	  }


	  if ((CanState & 0x01) != 0) //test for CAN error callback activity
	  {
		  CanState = CanState & 0xFE; //clear control flag

	  }



	  if (ProcessRececivedCanData == FLAG_CLEAR) //don't grab more data until the previous data has been processsed and displayed.
	  {
		  if (CanDataReceived == FLAG_SET) //test for CAN receive complete callback activity
		  {
			  //grab a copy of the received data
			  CanIdentifier = pCanRxHeader->StdId;
			  CanDlc = pCanRxHeader->DLC;
			  for (uint8_t i=0; i<CanDlc; i++)
			  {
				  CanData[i] = CanRxData[i];
			  }

			  ProcessRececivedCanData = FLAG_SET;
			  CanDataReceived = FLAG_CLEAR;
			  //output received CAN data
		  }
	  }


	  if (ProcessRececivedCanData == FLAG_SET)
	  {
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {
			  char tempstring2[40] = "";
			  sprintf(tempstring, "\e[21;1HCAN Id:0x%3X, DLC:%d, data:", CanIdentifier, CanDlc);
			  for (uint8_t i=0; i<CanDlc; i++)
			  {
				  sprintf(tempstring2, "0x%02X, ", CanData[i]);
				  strcat(tempstring, tempstring2);
			  }

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
			  ProcessRececivedCanData = FLAG_CLEAR;
		  }


	  }





	  if (timer1heartbeat2 == FLAG_SET)
	  {
		  timer1heartbeat2 = FLAG_CLEAR;
		  //Set DAC output
		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)DacVal);
		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)DacVal2);

		  DacVal++;
		  if (DacVal > 0xFFF)
		  {
			  DacVal = 0;
		  }

		  DacVal2--;
		  if (DacVal2 > 0xFFF)
		  {
			  DacVal2 = 0xFFF;
		  }
	  }


	  if (timer1heartbeat == FLAG_SET)
	  {
		  //We get here after 100 timer interrupts have occurred
		  timer1heartbeat = FLAG_CLEAR;


		  //output a CAN message
		  uint32_t x = 0;
		  x = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		  if (x != 0)
		  {
			  uint8_t datapayload[8] = {};
			  for (uint8_t i = 0; i<8; i++)
			  {
				  datapayload[i] = 0x10 + candatacount;
				  candatacount++;
			  }
			  //uint32_t Txmailbox = CAN_TX_MAILBOX0;
			  uint32_t Txmailbox = 0xff;
			  //uint32_t* pTxmailbox = Txmailbox;
			  uint8_t CanTxError = 0;
			  if (HAL_CAN_AddTxMessage(&hcan1, pCanTxHeader, datapayload, &Txmailbox) != HAL_OK)
			  {
				  //there is a problem with sending a CAN message...
				  CanTxError = 1;
			  }
			  else
			  {
				  if (screenblock == FLAG_CLEAR)
				  {
					  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
					  {
						  sprintf(tempstring, "\e[20;1HCAN message %3d", CanTxMsgCount);
						  uint16_t stringlength = strlen(tempstring);
						  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						  UartMsgSent = FLAG_SET;
						  CanTxMsgCount++;
					  }
				  }
			  }
			  CanTxMailboxfullmsg = FLAG_CLEAR; //allow TX mailbox full message to be displayed if the mailbox becomes full again!
		  }
		  else
		  {
			  if (CanTxMailboxfullmsg == FLAG_CLEAR)
			  {
				  if (screenblock == FLAG_CLEAR)
				  {
					  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
					  {
						  sprintf(tempstring, "\e[20;1HCAN TX mailboxes FULL!");
						  uint16_t stringlength = strlen(tempstring);
						  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						  UartMsgSent = FLAG_SET;
						  CanTxMailboxfullmsg = FLAG_SET;
					  }
				  }
			  }
		  }



		  if ((mainloopcount & 0x01) != 0)
		  {
			  //exercise digital outputs
			  HAL_GPIO_WritePin(GPIOD, HSD_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, HSD_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, HSD_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, HSD_4_Pin, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(GPIOD, LSD_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, LSD_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, LSD_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, LSD_4_Pin, GPIO_PIN_RESET);
		  }

		  else
		  {
			  HAL_GPIO_WritePin(GPIOD, HSD_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, HSD_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, HSD_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, HSD_4_Pin, GPIO_PIN_SET);

			  HAL_GPIO_WritePin(GPIOD, LSD_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, LSD_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOD, LSD_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, LSD_4_Pin, GPIO_PIN_SET);
		  }


		  if (HAL_GPIO_ReadPin(MODE1_GPIO_Port, MODE1_Pin) != 0) //test key switch input
		  {
			  HAL_GPIO_WritePin(MODE1_LED_GPIO_Port, MODE1_LED_Pin, GPIO_PIN_SET);

			  if (XmodemStatus == 0x03)
			  {
				  XmodemStatus = 4;
				  //send initiation character for start of X-modem transfer
				  strcpy(tempstring, "C");
				  uint16_t stringlength = strlen(tempstring);
				  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
				  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
				  UartMsgSent = FLAG_SET;

				  RxStringLen = 0; //zero contents of receive buffer - this will get updated by mainloop as soon as x-modem data arrives

				  //start timeout period
				  FunctionDelay = 5000; //value decremented by TIM ISR
			  }
		  }
		  else
		  {
			  HAL_GPIO_WritePin(MODE1_LED_GPIO_Port, MODE1_LED_Pin, GPIO_PIN_RESET);
		  }


		  if ((mainloopcount & 0x02) != 0)
		  {
			  HAL_GPIO_WritePin(GPIOD, SpiReset_Pin, GPIO_PIN_SET);

		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOD, SpiReset_Pin, GPIO_PIN_RESET);
		  }

	  }

	  if (UartOutputFlag == FLAG_SET) //flag set periodically by TIM ISR
	  {
		  if (XmodemStatus == 0)
		  {
			  if (screenblock == FLAG_CLEAR)
			  {
				  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				  {

					  UartOutputFlag = FLAG_CLEAR;
					  //HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)

					  //char tempstring[20] = "ABC123";

					  sprintf(tempstring, "\e[%d;1HABC-%3d\r", index+8, mainloopcount);

					  uint16_t stringlength = strlen(tempstring);
					  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					  UartMsgSent = FLAG_SET;

					  index++;
					  if (index > 7)
					  {
						  index = 0;
					  }
				  }
			  }
		  }


		  mainloopcount++;
	  }

	  if ((RxState & 0x20) != 0) //test for processing of complete string
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {
			  RxState = RxState & 0xDF;	//clear flag

			  //now test received string
			  uint8_t commandlength = strlen(RxString);
			  uint8_t comp = 0;
			  //char tmpstr[50] = "";
			  recognisedstring = FLAG_CLEAR;


			  if (commandlength == 2)
			  {

				  comp = strcmp(RxString, "CH");
				  if (comp == 0)
				  {
					  //Check sequencer header data

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Checkling header block:");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;


					  uint32_t tempval = 0;
					  tempval = CheckHeaderBlock();
					  if (tempval == 0)
					  {
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "CRC OK"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
					  }
					  else
					  {
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "CRC FAILED!"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "(error:0x%02X)", tempval); //reset all attributes
						  strcat(tempstring, tmpstr);
					  }


				  }

				  comp = strcmp(RxString, "XR");
				  if (comp == 0)
				  {
					  //Enable X modem receive
					  XmodemStatus = 1;

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "X-modem receive function");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;

				  }
			  }

			  if (commandlength == 3)
			  {
				  comp = strcmp(RxString, "CSM");
				  if (comp == 0)
				  {
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Clearing sequencer memory");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;

					  //now clear sequencer memory block(s)

					  uint8_t tempbuffer[16] = {0};
					  uint8_t* intptr = 0;
					  intptr = &tempbuffer[0];

					  tempbuffer[0] = 0x00;
					  tempbuffer[1] = 0x00;
					  tempbuffer[2] = 0x00;	//MAT table address
					  tempbuffer[3] = 0x80;
					  tempbuffer[4] = 0x01;	//data start address
					  tempbuffer[5] = 0x00;
					  tempbuffer[6] = 0x00; //cyclecount
					  tempbuffer[7] = 0x04;
					  tempbuffer[8] = 0x00;	//Default output block address
					  tempbuffer[9] = 0x40;
					  tempbuffer[10] = 0x00;
					  tempbuffer[11] = 0x00;
					  tempbuffer[12] = 0x00;
					  tempbuffer[13] = 0x00;

					  //generate CRC for sequencer header memory block
					  SetCrc16Value(0);
					  //uint16_t CalculateBlockCrc(uint8_t* pInt, uint16_t qty);
					  uint16_t temp = 0;
					  temp = CalculateBlockCrc(intptr, 14);
					  tempbuffer[14] = (uint8_t)(temp>>8);
					  tempbuffer[15] = (uint8_t)temp;

					  StepIndex = 0;
					  SeqCmdError = 0;

					  //I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
					  uint32_t response = 0;
					  response = I2cWriteBlock(SEQUENCERMEMORY, SEQHEADERADDR, 2, intptr, 16);
					  if (response != 0)
					  {
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "FAILED to write header data"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "(error = 0x%02X)", response); //reset all attributes
						  strcat(tempstring, tmpstr);
					  }
					  else
					  {
//						  response = UpdateSeqHeaderCrc(); //update header data block held in I2C memory
//						  if (response == 0)
//						  {
							  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "OK"); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
//						  }
//						  else
//						  {
//							  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
//							  strcat(tempstring, tmpstr);
//							  sprintf(tmpstr, "Header update failed"); //reset all attributes
//							  strcat(tempstring, tmpstr);
//						  }
					  }

				  }

				  comp = strcmp(RxString, "XF");
				  if (comp == 0)
				  {
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Set X-modem input data format");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  dataformat = 99;
					  recognisedstring = FLAG_SET;
					  if (RxString[2] == '0')
					  {
						  //basic text input
						  dataformat = 0;

					  }
					  if (RxString[2] == '1')
					  {
						  //Intel hex input
						  dataformat = 1;

					  }
					  if (RxString[2] == '2')
					  {
						  //Intel hex input
						  dataformat = 2;

					  }

					  if (dataformat != 0)
					  {
						  sprintf(tmpstr, "\e[4;1H\e[KData format: 0x%02X", dataformat); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
					  }
					  else
					  {
						  sprintf(tmpstr, "\e[4;1H\e[KUnrecognised format!"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
					  }
				  }

			  }

			  if (commandlength == 4)
			  {
				  comp = strcmp(RxString, "I2C?");
				  if (comp == 0)
				  {
					  struct I2cConfig tempstruct; //structure defined in GcI2cV1.c
					  tempstruct = GetI2cConfig();

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "I2C config:");

					  sprintf(tmpstr, "\e[4;1H\e[KDevice address: 0x%02X", tempstruct.I2cDeviceAddress); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[5;1H\e[KAddress width: %d", tempstruct.I2cInternalAddressWidth); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[6;1H\e[KInternal address: 0x%04X", tempstruct.I2cInternalAddress); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[7;1H\e[KQuantity: 0x%04X", tempstruct.I2cQuantity); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;
				  }

				  comp = strcmp(RxString, "I2CI");
				  if (comp == 0)
				  {
					  //sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "I2C Initialisation");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;
					  I2cInitialisationFunction = 1;  //initialise function call from within main loop
				  }

				  comp = strcmp(RxString, "I2CR");
				  if (comp == 0)
				  {
					  //sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "I2C Read block");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

					  recognisedstring = FLAG_SET;
					  I2cReadBlockFunction = 1;
					  screenblock = FLAG_SET; //prevent other main loop processed overwriting the screen
				  }


				  comp = strcmp(RxString, "SEQ");
				  if (comp == 0)
				  {
					  if (RxString[3] == '0')
					  {
						  //Stop sequencer process
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Sequencer Stopped");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;

						  SetSequencerState(0);
					  }


					  if (RxString[3] == '1')
					  {
						  //Start sequencer process
						  //sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Sequencer Started");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;
						  //load sequencer buffer memory

						  //check header
						  recognisedstring = FLAG_SET;


						  uint32_t tempval = 0;
						  //tempval = CheckHeaderBlock();
						  tempval = ReadSeqHeader(byteptr);
						  if (tempval == 0)
						  {
							  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);
							  //sprintf(tmpstr, "CRC OK"); //reset all attributes
							  //strcat(tempstring, tmpstr);


							  //read max steps
							  uint16_t MaxSequencerSteps = (uint16_t)(*byteptr) << 8 | (uint16_t)(*(byteptr+1));
							  SetSequencerMaxSteps(MaxSequencerSteps);

							  sprintf(tmpstr, "Max steps:5d", MaxSequencerSteps); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);

							  MaxSequencerCycles = (uint16_t)(*(byteptr + 6)) << 8 | (uint16_t)(*(byteptr+7));
							  sprintf(tmpstr, "Max cycles:5d"); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);

							  SeqStepIndex = 0;


							  InitialiseSequencerDataBuffer(); //reset sequencer buffer

							  SetSequencerState(1);

						  }
						  else
						  {
							  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "CRC FAILED!"); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "(error:0x%02X)", tempval); //reset all attributes
							  strcat(tempstring, tmpstr);

							  SetSequencerState(0);
						  }







					  }


					  recognisedstring = FLAG_SET;
					  I2cReadBlockFunction = 1;
					  screenblock = FLAG_SET; //prevent other main loop processed overwriting the screen
				  }


				  comp = strcmp(RxString, "1234");
				  if (comp == 0)
				  {

					  sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Recognised string");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;

				  }
			  }


			  if (commandlength == 5)
			  {
					comp = strncmp(RxString, "I2CA", 4); //I2CAx
					if (comp == 0)
					{
						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "I2C device address width set ");
						uint8_t width = 0;
						if (RxString[4] == '2')
						{
							width = 2;
							strcat(tempstring, "2");
						}
						else
						{
							strcat(tempstring, "1");
						}
						SetInternalAddressWidth(width);
						recognisedstring = FLAG_SET;
					}
			  }


			  if (commandlength == 6)
			  {



				  comp = strncmp(RxString, "I2CD", 4); //I2CDxx
				  if (comp == 0)
				  {


						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "I2C Device set: ");
						sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);

						char valstring[10] = "";
						strncpy(valstring, &RxString[4], 2); //obtain value characters

						//set I2C device address
						UserVal = ExtractValueFromString(RxString, 4, 2);
						if ((UserVal & 0x80000000) == 0)
						{
							SetI2cDeviceAddress(UserVal & 0xFF);

							/*
							struct I2cConfig3{
								uint16_t I2cInternalAddress;
								uint16_t I2cQuantity;
								uint8_t I2cInternalAddressWidth;
								uint8_t I2cDeviceAddress;
							};

							struct I2cConfig3 tempstruct; //create an instance of a structure
							struct I2cConfig3* structptr; //structure type declared in 'GcI2cC1.c'
							*/
							//struct I2cConfig4 tempstruct; //I2cConfig2 declared in global-settingsV1.h
							//struct I2cConfig4* structptr;
							//structptr = &tempstruct;
							//ReadI2cConfig(structptr);
							//ReadI2cConfig(&tempstruct);
							struct I2cConfig tempstruct; //structure defined in GcI2cV1.c
							tempstruct = GetI2cConfig();

							sprintf(tmpstr, "I2C device address: 0x%02X", tempstruct.I2cDeviceAddress);
							strcat(tempstring, tmpstr);
						}
						else
						{
							sprintf(tmpstr, "I2C device address set Error!");
							strcat(tempstring, tmpstr);
						}

						sprintf(tmpstr, "\e[0m"); //reset all attributes
						strcat(tempstring, tmpstr);
						recognisedstring = FLAG_SET;

				  }
			  }

			  if (commandlength == 8)
			  {
				  comp = strncmp(RxString, "I2CA", 4); //I2CAxxxx
				  if (comp == 0)
				  {


						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "I2C Internal address set: ");


						//char valstring[10] = "";
						//strncpy(valstring, RxString[4], 2); //obtain value characters

						//set I2C device address
						UserVal = ExtractValueFromString(RxString, 4, 4);
						if ((UserVal & 0x80000000) == 0)
						{
							SetI2cInternalAddress(UserVal & 0xFFFF);

							struct I2cConfig tempstruct; //structure defined in GcI2cV1.c
							tempstruct = GetI2cConfig();

							sprintf(tmpstr, "I2C internal address: 0x%04X", tempstruct.I2cInternalAddress);
							strcat(tempstring, tmpstr);
						}
						else
						{
							sprintf(tmpstr, "I2C internal address set Error!");
							strcat(tempstring, tmpstr);
						}


						sprintf(tmpstr, "\e[0m"); //reset all attributes
						strcat(tempstring, tmpstr);
						recognisedstring = FLAG_SET;


				  }

				  comp = strncmp(RxString, "I2CF", 4); //I2CFxx
				  if (comp == 0)
				  {

					  	//Fill block of I2C memory
						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Fill I2C memory: ");
						sprintf(tmpstr, "\e[4;1H\e[K"); //
						strcpy(tempstring, tmpstr);

						char valstring[10] = "";
						strncpy(valstring, &RxString[4], 2); //obtain value characters

						//set I2C device address
						UserVal = ExtractValueFromString(RxString, 4, 2);
						if ((UserVal & 0x80000000) == 0)
						{
							FillI2cMemoryFunction = 1;
							*byteptr = (uint8_t)UserVal; //record fill data value
							FunctionDelay = 1000; //value decremented by TIM ISR
							recognisedstring = FLAG_SET;
						}
						else
						{
							sprintf(tmpstr, " ERROR!"); //
							strcat(tempstring, tmpstr);
						}
				  }

				  comp = strncmp(RxString, "I2CQ", 4); //I2CQxxxx
				  if (comp == 0)
				  {
					  	sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Set Quantity Value");
						UserVal = ExtractValueFromString(RxString, 4, 4);
						if ((UserVal & 0x80000000) == 0)
						{

							sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "Quantity: 0x%04X", (uint16_t)(UserVal & 0xFFFF));
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
							strcat(tempstring, tmpstr);

							SetI2cBlockSize(UserVal & 0xFFFF);
							recognisedstring = FLAG_SET;
						}
						else
						{

						}
						sprintf(tmpstr, "\e[0m"); //reset all attributes
						strcat(tempstring, tmpstr);


				  }

				  comp = strncmp(RxString, "I2CR", 4); //I2CRxxxx
				  if (comp == 0)
				  {

						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Read byte from I2C: ");


						//char valstring[10] = "";
						//strncpy(valstring, RxString[4], 2); //obtain value characters

						//set I2C device address
						UserVal = ExtractValueFromString(RxString, 4, 4);
						if ((UserVal & 0x80000000) == 0)
						{
							uint16_t addr = UserVal & 0xFFFF;

							sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "I2C internal address: 0x%04X", addr);
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
							strcat(tempstring, tmpstr);

							uint32_t data = 0;
							data = I2cReadByte(addr);
							if ( ((data >> 24) & 0xFF) == 0 )
							{
								//sprintf(tmpstr, "I2C read data: 0x%08lX", data);
								sprintf(tmpstr, "I2C read data: 0x%02X", (uint8_t)(data & 0xFF));
							}
							else
							{
								//sprintf(tmpstr, "There was a problem reading a byte from I2C memory!");
								strcpy(tmpstr, "There was a problem reading a byte from I2C memory!");
							}
							strcat(tempstring, tmpstr);



						}
						else
						{
							sprintf(tmpstr, "I2C read single byte Error! (0x%02lX)", (UserVal >> 24) & 0xFF);
							strcat(tempstring, tmpstr);
						}

						sprintf(tmpstr, "\e[0m"); //reset all attributes
						strcat(tempstring, tmpstr);
						recognisedstring = FLAG_SET;

				  }

			  }


			  if (commandlength == 10)
			  {
				  comp = strncmp(RxString, "I2CW", 4); //I2CWaaaadd
				  if (comp == 0)
				  {

						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Write single byte to I2C");
						sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
						strcat(tempstring, tmpstr);
						//process address and data values
						UserVal = ExtractValueFromString(RxString, 4, 6);
						if ((UserVal & 0x80000000) == 0)
						{
							uint16_t addr = UserVal >> 8;

							sprintf(tmpstr, "\taddress: 0x%04X", addr);
							strcat(tempstring,tmpstr);
							//SendSerial(msg);
							//SetI2cInternalAddress(addr);

							uint8_t* pI2cData = 0;
							//pI2cData = &I2cDataBuffer[0];
							pI2cData = GetI2cData();
							*pI2cData = UserVal & 0xFF;
							sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 3rd line, clear text,
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\tdata: 0x%02X", *pI2cData);
							//SendSerial(msg);
							strcat(tempstring,tmpstr);
							uint32_t result = 0;
							//result = I2cWriteByte(addr, UserVal & 0xFF);
							//result = I2cWriteByteBlocking(addr, UserVal & 0xFF, 1);
							result = I2cWriteByte(addr, (uint8_t)(UserVal & 0xFF), 1);
							sprintf(tmpstr, "\e[6;1H\e[K"); //move cursor to 3rd line, clear text,
							strcat(tempstring, tmpstr);
							sprintf(tmpstr, "\tresponse: 0x%02X", (uint8_t)result);
							//SendSerial(msg);
							strcat(tempstring,tmpstr);
						}
						else
						{
							sprintf(tmpstr, "I2C write single byte command syntax Error!\r\n");
							//SendSerial(msg);
							strcat(tempstring,tmpstr);
						}
						recognisedstring = FLAG_SET;

				  }
			  }

			  if (recognisedstring == FLAG_CLEAR)
			  {
				  sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;41m"); //move cursor to 3rd line, clear text, white text on red background
				  strcpy(tempstring, tmpstr);
				  strcat(tempstring, "Unrecognised string");
				  sprintf(tmpstr, "\e[0m"); //reset all attributes
				  strcat(tempstring, tmpstr);
			  }


			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
		  }
	  }





	  if ((RxState & 0x10) != 0) //test for clearing of ESCAPE message from VT100 screen
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {
			  //clear ECSAPE message from VT100 screen
			  //char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[2;1H\e[K"); //move cursor to 2nd line, white text on red background
			  strcpy(tempstring, tmpstr);

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;

			  RxState = RxState & 0xEF; //clear flag
		  }
	  }



	  if ((RxState & 0x08) != 0) //test for Escape character
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {

			  //char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[2;1H\e[1;37;41m"); //move cursor to 2nd line, white text on red background
			  strcpy(tempstring, tmpstr);
			  strcat(tempstring, "ESCAPE");
			  sprintf(tmpstr, "\e[0m"); //reset all attributes
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[1;1H\e[K"); //clear string construction line
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[6;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;


			  RxReadPtr = 0;
			  RxWritePtr = 0;
			  RxBufferCount = 0;

			  EscapeClearCount = ESCAPEDISPLAYPERIOD; //value to be decremented by timer ISR

			  RxState = RxState & 0xF7; //clear bit.

			  screenblock = FLAG_CLEAR; //allows other main loop functions to update the display


		  }
	  }


	  if ((RxState & 0x04) != 0) //test for carriage return terminated string
	  {

		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {


			  //char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[2;1H\e[K"); //move cursor to 2nd line, clear existing data
			  strcpy(tempstring, tmpstr);
			  strcat(tempstring, RxString);
			  sprintf(tmpstr, "\e[0m"); //reset all attributes
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[1;1H\e[K"); //clear string construction line
			  strcat(tempstring, tmpstr);


			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;

			  RxState = RxState & 0xFB; //finish displaying received string
			  RxState = RxState | 0x20;	//prepare to process received string



		  }
	  }


	  if ((RxState & 0x02) != 0) //check for sewrial receive buffer overflow
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {
			  //indicate buffer overflow
			  //char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[1;1H\e[K\e[1;37;41m"); //move cursor to 1st line, clear and text, then prepare white text on red background
			  strcpy(tempstring, tmpstr);
			  strcat(tempstring, "Buffer overflow!");
			  sprintf(tmpstr, "\e[0m"); //reset all attributes
			  strcat(tempstring, tmpstr);


			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;

			  //Clear receive buffer
			  RxReadPtr = 0;
			  RxWritePtr = 0;
			  RxBufferCount = 0;

			  RxState = RxState & 0xFD; //clear bit.
		  }
	  }


	  if ((RxState & 0x01) != 0) //check for reception of a new character
	  {

		  if ((XmodemStatus & 0x0F) == 0)
		  {

			  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
			  {

				  //process received serial data.
				  RxState = RxState & 0xFE; //clear bit

				  //char tmpstr[20]="";
				  uint8_t charindex = RxReadPtr;
				  RxStringLen = 0;
				  RxString[0] = 0; //terminate string

				  if (RxBufferCount != 0) //check fill level of primary receive buffer
				  {
					  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); //disable receive interrupts until current buffer has been tested
					  for (uint8_t i=0; i<RxBufferCount; i++)
					  {

						  //treat received data as text strings
						  if (RxBuffer2[charindex] == 0x0d)
						  {
							  if (RxStringLen != 0) //test for at least one character before the carriage return character
							  {
								  RxState = RxState | 0x04;
								  RxBufferCount = RxBufferCount - (RxStringLen + 1);
							  }
							  RxReadPtr = charindex + 1;
							  if (RxReadPtr >= RXBUFFERLENGTH) //Check for wraparound condition
							  {
								  RxReadPtr = 0;
							  }
							  break;
						  }

						  else
						  {
							  RxString[i] = RxBuffer2[charindex];
							  RxStringLen++;
							  RxString[i+1] = 0; //terminate string

							  charindex++;
							  if (charindex >= RXBUFFERLENGTH)
							  {
								  charindex = 0;
							  }
						  }
					  }


					  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //re-enable receive interrupts

					  sprintf(tmpstr, "\e[1;1H\e[K\e[1;33;40m"); //set cursor to line 1, clear existing data, set yellow background
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, RxString);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);

				  }
				  else
				  {
					  //we get here if Rx buffer has been emptied of all characters
					  sprintf(tmpstr, "\e[1;1H\e[K"); //set cursor to line 1, clear existing data
					  strcpy(tempstring, tmpstr);

				  }
				  uint16_t stringlength = strlen(tempstring);
				  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
				  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
				  UartMsgSent = FLAG_SET;
				  //end main loop received serial string processing
			  }

		  }
		  else
		  {
			  //treat received data as X-modem packets
			  //process received serial data.
			  RxState = RxState & 0xFE; //clear bit

			  if (RxBufferCount == 1)
			  {
				  if (RxBuffer2[RxReadPtr] == 0x04)
				  {
					  //xmodem End of transmission character detected
					  XmodemStatus = 6;
				  }
			  }

			  else if (RxBufferCount >= 133) //check fill level of primary receive buffer
			  {
				  RxStringLen = 0;
				  RxString[0] = 0; //terminate string
				  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); //disable receive interrupts until current buffer has been tested
				  for (uint8_t i=0; i<RxBufferCount; i++)
				  {
					  RxString[i] = RxBuffer2[RxReadPtr];
					  RxStringLen++;
					  RxString[i+1] = 0; //terminate string

					  RxReadPtr = RxReadPtr + 1;
					  if (RxReadPtr >= RXBUFFERLENGTH) //Check for wrap around condition
					  {
						  RxReadPtr = 0;
					  }

				  }
				  XmodemStatus = 8;





			  }
			  //end main loop x-modem packet processing
		  }


	  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909BEB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 39;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Backlight_Pin|SpiReset_Pin|HSD_1_Pin|HSD_2_Pin
                          |HSD_3_Pin|HSD_4_Pin|LSD_1_Pin|LSD_2_Pin
                          |LSD_3_Pin|LSD_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UP_LED_Pin|DWN_LED_Pin|MODE1_LED_Pin|MODE2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_Pin DOWN_Pin MODE1_Pin MODE2_Pin */
  GPIO_InitStruct.Pin = UP_Pin|DOWN_Pin|MODE1_Pin|MODE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Backlight_Pin SpiReset_Pin HSD_1_Pin HSD_2_Pin
                           HSD_3_Pin HSD_4_Pin LSD_1_Pin LSD_2_Pin
                           LSD_3_Pin LSD_4_Pin */
  GPIO_InitStruct.Pin = Backlight_Pin|SpiReset_Pin|HSD_1_Pin|HSD_2_Pin
                          |HSD_3_Pin|HSD_4_Pin|LSD_1_Pin|LSD_2_Pin
                          |LSD_3_Pin|LSD_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_LED_Pin DWN_LED_Pin MODE1_LED_Pin MODE2_LED_Pin */
  GPIO_InitStruct.Pin = UP_LED_Pin|DWN_LED_Pin|MODE1_LED_Pin|MODE2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		//static uint16_t timer1count = TIMER1PERIOD;
		static uint16_t DacUpdatecount = TIMER1DACUPDATECOUNT;
		static uint16_t UartUpdateCount = UARTUPDATEPERIOD;

		if (FunctionDelay != 0)
		{
			FunctionDelay--;
		}

		if (EscapeClearCount != 0) //value set during main loop
		{
			EscapeClearCount--;
			if (EscapeClearCount == 0)
			{
				RxState = RxState | 0x10; //indicate to main loop
			}
		}


		if (timer1count != 0)
		{
			timer1count--;
			if (timer1count == 0)
			{
				timer1heartbeat = FLAG_SET;
				timer1count = TIMER1PERIOD;
			}
		}

		if (DacUpdatecount != 0)
		{
			DacUpdatecount--;
			if (DacUpdatecount == 0)
			{
				HAL_GPIO_TogglePin(GPIOC, MODE2_LED_Pin);

				DacUpdatecount = TIMER1DACUPDATECOUNT;
				timer1heartbeat2 = FLAG_SET;
			}
		}

		if (UartUpdateCount != 0)
		{
			UartUpdateCount--;
			if (UartUpdateCount == 0)
			{
				UartUpdateCount = UARTUPDATEPERIOD;
				UartOutputFlag = FLAG_SET;
			}
		}

		DecrementI2cTiming(); //Used as timout period for I2C comms

	}
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{

	if (hcan->Instance == CAN1)
	{
		//CAN message transmitted
	}
}


void HAL_CAN_ErrorCallback (CAN_HandleTypeDef * hcan)
{
	if (hcan->Instance == CAN1)
	{
		CanState = CanState | 0x01; //indicate to main loop that CAN error callback was called

		//uint32_t HAL_CAN_GetError (const CAN_HandleTypeDef * hcan)
		uint32_t y = 0;
		y = HAL_CAN_GetError(hcan);


	}
}


//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//CAN message received
	if (hcan->Instance == CAN1)
	{
		uint32_t qty = 0;
		qty = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
		if (qty != 0)
		{



			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, pCanRxHeader, CanRxData);
			CanDataReceived = FLAG_SET;

		}
	}
}


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed once all I2C bytes have been written to memory
	//created 4NOV2021
	//I2cStatus = I2cStatus | 0x04; //flag to main loop
	SetI2cStatusBit(0x04);
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed once all I2C bytes have been read from memory
	//created 4NOV2021
	//I2cStatus = I2cStatus | 0x08; //flag to main loop
	SetI2cStatusBit(0x08);
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed once all I2C bytes have been received via HAL_I2C_Master_Receive_IT()
	//created 3NOV2021
	//I2cStatus = I2cStatus | 0x02; //flag to main loop
	SetI2cStatusBit(0x02);

}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed once all I2C bytes have been transmitted via HAL_I2C_Master_Transmit_IT()
	//created 3NOV2021
	//I2cStatus = I2cStatus | 0x01; //flag to main loop
	SetI2cStatusBit(0x01);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed on detected I2C error...
	//Created 5NOV2021
	//I2cStatus = I2cStatus | 0x80; //flag to main loop
	SetI2cStatusBit(0x80);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//call back executed on Master I2C abort execution
	//Created 5NOV2021
	//I2cStatus = I2cStatus | 0x40; //flag to main loop
	SetI2cStatusBit(0x40);
}




void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == USART1)
	if (huart->Instance == USART3)
	{

	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Last edited 6JUN2024
	if (huart->Instance == USART3)
	//if (huart->Instance == USART1)
	{
		if (XmodemStatus == 0)
		{
			//copy characters from primary to secondary buffers
			if (RxBuffer1[0] == 0x1B)
			{
				RxState = RxState | 0x08; //indicate to main loop that escape character has been detected
			}


	//		else if (RxBufferCount < RXBUFFERLENGTH)
	//		{

			else
			{
				if (RxBuffer1[0] == '\b')
				{

					//delete a character from the buffer
					if (RxBufferCount != 0)
					{
						RxBufferCount--;
						RxWritePtr--;
						RxBuffer2[RxWritePtr] = 0; //re-terminate string
					}
				}
				else
				{
					RxBuffer2[RxWritePtr] = RxBuffer1[0];
					RxWritePtr++;
					if (RxWritePtr >= RXBUFFERLENGTH) //test for wrap around
					{
						RxWritePtr = 0;
					}
					RxBuffer2[RxWritePtr] = 0; //terminate string

					RxBufferCount++;
					if (RxBufferCount > RXBUFFERLENGTH)
					{
						RxState = RxState | 0x02; //indicate rx buffer overflow
					}
				}
				RxState = RxState | 0x01; //indicate new character arrival
			}

	//		}
	//		else
	//		{
	//			RxState = RxState | 0x02; //indicate rx buffer overflow
	//		}
		}
		else
		{
			//receive an x-modem packet of data...
			RxBuffer2[RxWritePtr] = RxBuffer1[0];
			RxWritePtr++;
			if (RxWritePtr >= RXBUFFERLENGTH) //test for wrap around
			{
				RxWritePtr = 0;
			}
			RxBuffer2[RxWritePtr] = 0; //terminate string

			RxBufferCount++;
			if (RxBufferCount > RXBUFFERLENGTH)
			{
				RxState = RxState | 0x02; //indicate rx buffer overflow
			}

//			if (RxBufferCount > 10)
//			{
				RxState = RxState | 0x01; //indicate new character arrival
			}
//		}

		//UartRxData = FLAG_SET; //indicate to main loop that a new character has arrived
		HAL_UART_Receive_IT (&huart3, (uint8_t *) RxBuffer1, 1); //this re-enables UART reception interrupt
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == USART1)
	if (huart->Instance == USART3)
	{
		UartMsgSent = FLAG_CLEAR; // indicate to main loop that new data can be sent
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
