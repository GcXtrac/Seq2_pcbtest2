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
#define LFCHAR 10	//line feed character value
#define ACKCHAR 6	//Acknowledge character value
#define CRCHAR 13	//Carriage return character

#define PROJECTSTRING "Sequencer MkII V0.0.1"
#define DATESTRING "23OCT2025"




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

//static used here means that only this file cvan access these global variables
static uint8_t CanRxData[8] = {};
static uint8_t TempCanRxData[8] = {};


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
											//bit3-0 form a counter:
											//1: Initiate X modem transfer, initiate a timer (1)
											//2: wait for timer (1) period to expire, prompt user to press button
											//3: wait for button to be pressed
											//4: Button has been pressed, initialisation character 'C' has been sent to host, another timer (2) started,
											//		Waiting for x modem packet to arrive
											//		Single character received (EOT) jump to 6
											//		complete 133 byte packet received: jump to 8
											//5: packet receive timeout (2) expired, timer (3a) started, jump to step 7.
											//6: send acknowledgement character and initialise timer (3b)
											//7: wait for timer (3ab) period to expire, once expired issue X-modem comms finished message
											//8: test received data packet
											//

//trying here for the benefit of stm studio...
uint16_t DacVal = 0;
static uint16_t timer1count = TIMER1PERIOD;
uint8_t mainloopcount = 0;

volatile static uint8_t CanAnalogScanState = 0;
volatile static uint16_t ScanUpdatetimeRefreshValue = 200;
volatile static uint16_t ScanUpdateTime = 0;
volatile static uint16_t ScanValue = 0;

volatile static uint8_t UpdateScreen = 0;

volatile static uint16_t SeqStepTime = 0;

uint8_t Multishift = 0;	//set to non-zero value to initiate multiple shifting
uint8_t DedicatedShiftControl = 0x20;	//bit 7 enables dedicated shift demand sequencing. see serial command "SCx"
										//bit 6 enables shift demand sequencing via CAN. see serial command "SCCx"
										//bit 5 enables control of shift demand based on reported CAN position (enabled by default)

uint8_t ShiftDemand = 0;	//bit 7:set for CAN upshift
	 	 	 	 	 	 	//bit 6 set for CAN downshift
	 	 	 	 	 	 	//bit 5: set for logic level upshift
	 	 	 	 	 	 	//bit 4: set for logic level downshift
							//bit 3~0: state count





uint16_t PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
uint16_t PreloadPushActivationtime = 50;
uint16_t PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse


uint16_t ShiftDemandPulseCount = 0;
uint16_t ShiftDemandPulseTime = 100;


uint16_t PreloadPullActivationTime = 50;
uint16_t PreloadPullActivationCount = 0;
uint16_t PreloadPullDemandPulseTime = 100;
uint16_t PreloadPullDemandCount = 0;
uint16_t PreloadPushDelayTime = 100;
uint16_t PreloadPushActivationTime = 100;
uint16_t PreloadPushActivationCount = 0;
uint16_t PreloadPushDemandPulseTime = 100;
uint16_t PreloadPushDemandCount = 0;

uint16_t Shift2ShiftCount = 0;
uint16_t Shift2ShiftTime = 500;
uint16_t ShiftDemandCount = 0;

uint8_t recmsgindex = 0;
uint8_t reccount = 0;

uint8_t CanRxFifoFull = 0;

//CAN Rx
CAN_RxHeaderTypeDef CanRxHeader = {};
CAN_RxHeaderTypeDef TempCanRxHeader = {};
CAN_RxHeaderTypeDef * pCanRxHeader = &CanRxHeader;
CAN_RxHeaderTypeDef * pTempCanRxHeader = &TempCanRxHeader;

CAN_TxHeaderTypeDef CanTxHeader1399ShiftDmd = {};
CAN_TxHeaderTypeDef * pTempCanTxHeader = &CanTxHeader1399ShiftDmd;

uint8_t CanTxMailboxfullmsg = FLAG_CLEAR;  //flag set when CAN tx mailbox is full, this prevents repeated output

uint32_t CanErrorValue = 0;
uint16_t PositionMaxLimit = 0x380; //limit used to prevent further upshifting
uint16_t PositionMinLimit = 0x100; //limit used to prevent further downshifting
uint16_t PrevActuatorPosition = 0;
uint16_t ActuatorPosition = 0;
uint8_t ActuatorPositionState = 0; 	//bit0: indicates value has been updated used as a flag between CAN received ISR and main loop
									//bit1: set if CAN position signal has been received, this value will be reset by TIM1 timepout period expiring.
									//bit2: if actuator message 1 is to be processed (see serial command "AM1x"
									//bit3: actuator message flash state

uint32_t PositionSignalTimeoutCount = 0;
uint32_t PositionSignalTimeoutPeriod = 50;

uint32_t Shiftdemandfeedback = 0;	//bit7 using to indicate to main loop from TIM1 ISR
									//bit0~3 holds a error code
									//	0: shift demand completed
									//	1: CAN shift demand was blocked due to reported CAN position




uint8_t ActuatorMotorTemp = 0;
uint8_t ActuatorPcbTemp = 0;
uint8_t ActuatorMsg2State = 0; //bit flags:
								//bit 7; indicate to main loop that the 2nd actuator message has been received, thius will be cleared if timeout period expires
								//bit 6: actuator message 2 status has changed
								//bit 5: set by command "AM2x"
								//bit 4: set /cleared according to flash state
								//bit 1: PCB temperature has changed
								//bit 0: motor temperature has changed

uint32_t ActuatorMsg2FlashCount = 0; //value is decremented by TIM1 ISR
uint32_t ActuatorMsg2FlashTime = 350;

uint32_t ActuatorMsg2Timeoutcount = 0; //reset message timeout period, value decremented by TIM 1 ISR
uint32_t ActuatorMsg2TimeoutPeriod = 200;

uint8_t PrevActuatorMotorTemp = 0;
uint8_t PrevActuatorPcbTemp = 0;

uint32_t ActuatorMsg1FlashCount = 0; //if value is set to non-zero then value will be decremented by TIM1 ISR
uint32_t ActuatorMsg1FlashTime = 300;

static uint8_t ProcessIndex = 0; 	//main loop process pointer
									//1: Shift demand configuration help function

static uint8_t ProcessCount = 0;	//counter used to determine position within a main loop process

char tmpstr[200] = "";
char tempstring[200] = "";


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

//Function prototypes
void tempfunction(void);
void MainLoopProcess01(void);
void MainLoopProcess02(void);
uint8_t CanShiftDemand(uint8_t direction);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void tempfunction(void)
{
	ProcessCount = 50;
}


void MainLoopProcess02(void)
{
	//Created 22OCT2025
	//Last edited 23OCT2025
	//Function used by mainloop to output serial commands
	//See serial command "?"
	if (ProcessIndex == 0x02)
	{
		if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		{
			strcpy(tempstring, "");
			switch(ProcessCount)
			{
				case(0):
					  sprintf(tmpstr, "\e[2J\e[H"); //clear screen and home cursor
					  strcpy(tempstring, tmpstr);
					  strcpy(tmpstr, "Serial commands");
					  strcat(tempstring, tmpstr);

					  strcat(tempstring, "\e[3:1H"); //move cursor to 1st line, clear text,
					  strcat(tempstring, "\tSCx: Sequenced Shift demand control");
					  strcat(tempstring, "\e[4:1H"); //move cursor to 2nd line, clear text,
					  strcat(tempstring, "\tSCCx: Sequenced CAN Shift demand control");
					  break;

				case(1):
					  strcpy(tempstring, "\e[5:1H"); //move cursor to 3th line, clear text,
					  strcat(tempstring, "\tSDxxxx: Shift demand pulse width");
					  strcat(tempstring, "\e[6:1H"); //move cursor to 4th line, clear text,
					  strcat(tempstring, "\tPPAxxxx: Preload Pull activation delay (msec)");
					  break;

				case(2):
					  strcpy(tempstring, "\e[7:1H"); //move cursor to 5th line, clear text,
					  strcat(tempstring, "\tPPBxxxx: Preload Pull demand pulse width");
					  strcat(tempstring, "\e[8:1H"); //move cursor to 6th line, clear text,
					  strcat(tempstring, "\tPPCxxxx: Preload Push activation delay (msec)");
					  break;

				case(3):
					  strcpy(tempstring, "\e[9:1H"); //move cursor to 7th line, clear text,
					  strcat(tempstring, "\tPPDxxxx: Preload Push demand pulse width (msec)");
					  strcat(tempstring, "\e[10:1H"); //move cursor to 8th line, clear text,
					  strcat(tempstring, "\tRPCx: Utilise reported CAN position signal");
					  break;

				case(4):
					  strcpy(tempstring, "\e[11:1H"); //move cursor to 9th line, clear text,
					  strcat(tempstring, "\tAM1x: Actuator CAN message 1 processing");
					  strcat(tempstring, "\e[12:1H"); //move cursor to 10th line, clear text,
					  strcat(tempstring, "\tAM2x: Actuator CAN message 2 processing");
					  break;


				case(5):
					  strcpy(tempstring, "\e[13:1H"); //move cursor to 11th line, clear text,
					  strcat(tempstring, "\tRSDxxxx: set multiple shift-shift delay time (msec)");
					  break;

				default:

			}
			uint16_t stringlength = strlen(tempstring);
			if (stringlength != 0)
			{
				//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
				HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
				UartMsgSent = FLAG_SET;
			}

			ProcessCount++;
			if (ProcessCount > 10)
			{
			  ProcessIndex = 0; //disable main loop process
			}
		}
	}

}

void MainLoopProcess01(void)
{
	//Created 22OCT2025
	//Last edited 23OCT2025
	//Main loop process 01. This function is called by the main loop but triggered by serial command "SD?"
	//Designed to display actuator shift demand configuration info on the display
	//Makes use of global variables!!!

	//char tmpstr[200] = ""; //can't use local variables here as the data needs to exist after the function has been executed!!!
	//char tempstring[200] = "";

	if (ProcessIndex == 0x01)
	{
		if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		{
			strcpy(tempstring, "");
			switch(ProcessCount)
			{
			case(0):
				  //sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
				  sprintf(tmpstr, "\e[2J\e[H"); //clear screen and home cursor
				  strcpy(tempstring, tmpstr);
				  strcpy(tmpstr, "Shift demand sequencing configuration");
				  strcat(tempstring, tmpstr);

				  strcat(tempstring, "\e[4:1H\e[K"); //move cursor to 4th line, clear text,
				  strcat(tempstring, "\tShift Sequencing:\t\t");
				  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
				  {
					  strcat(tempstring, "Enabled");
				  }
				  else
				  {
					  strcat(tempstring, "Disabled");
				  }
				  break;

			case(1):
				  strcpy(tempstring, "\e[5:1H\e[K"); //move cursor to 5th line, clear text,
				  strcat(tempstring, "\tCAN Shift Sequencing:\t\t");
				  if ((DedicatedShiftControl & 0x40) != 0) //see serial command "SCCx"
				  {
					  strcat(tempstring, "Enabled");
				  }
				  else
				  {
					  strcat(tempstring, "Disabled");
				  }
				  break;

			case(2):
				  strcpy(tempstring, "\e[6:1H"); //move cursor to 6th line, clear text,
				  sprintf(tmpstr, "\tShift demand pulse width:\t%4dmsec", ShiftDemandPulseTime);
				  strcat(tempstring,tmpstr);

				  strcat(tempstring, "\e[7:1H"); //move cursor to 7th line, clear text,
				  sprintf(tmpstr, "\tPreload pull activation delay:\t%4dmsec", PreloadPullActivationTime); //PPAxxxx
				  strcat(tempstring,tmpstr);
				  break;

			case(3):
				  strcat(tempstring, "\e[8:1H"); //move cursor to 8th line, clear text,
				  sprintf(tmpstr, "\tPreload pull demand pulse:\t%4dmsec", PreloadPullDemandPulseTime); //PPBxxxx
				  strcat(tempstring,tmpstr);

				  strcat(tempstring, "\e[9:1H"); //move cursor to 9th line, clear text,
				  sprintf(tmpstr, "\tPreload push activation delay:\t%4dmsec", PreloadPushActivationTime); //PPCxxxx
				  strcat(tempstring,tmpstr);
				  break;

			case(4):
				  strcat(tempstring, "\e[10:1H"); //move cursor to 10th line, clear text,
				  sprintf(tmpstr, "\tPreload push demand pulse:\t%4dmsec", PreloadPushDemandPulseTime); //PPDxxxx
				  strcat(tempstring,tmpstr);
				  break;

			case(5):
				  strcpy(tempstring, "\e[11:1H"); //move cursor to 11th line, clear text,
				  strcat(tempstring, "\tUse CAN position:\t\t");
				  if ((DedicatedShiftControl & 0x20) != 0) //see serial command "RPCx"
				  {
					  strcat(tempstring, "Enabled");
				  }
				  else
				  {
					  strcat(tempstring, "Disabled");
				  }
				  break;

			case(6):
				  strcat(tempstring, "\e[12:1H"); //move cursor to 10th line, clear text,
				  sprintf(tmpstr, "\tMultiple shift-shift delay:\t%4dmsec", Shift2ShiftTime); //RSDxxxx
				  strcat(tempstring,tmpstr);
				  break;

			default:
			}

			uint16_t stringlength = strlen(tempstring);
			if (stringlength != 0)
			{
				//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
				HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
				UartMsgSent = FLAG_SET;
			}

			ProcessCount++;
			if (ProcessCount > 10)
			{
			  ProcessIndex = 0; //disable main loop process
			}
		}
	}
}


uint8_t CanShiftDemand(uint8_t direction)
{
	//Used by sequenced shift demand functions
	//Created 9OCT2025
	//input:
	//	direction:
	//		0: inactive shift demand
	//		1: upshift
	//		2(-1):downshift

	uint32_t x = 0;
	uint8_t CanTxError = 0;
	x = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	if (x != 0)
	{
		  uint8_t datapayload[8] = {0};

		  switch(direction)
		  {
		  	  case(1):
				//upshift
				datapayload[2] = 0x10;
				break;

		  	  //case(-1):
		  	  case(2):
				//downshift
				datapayload[2] = 0x20;
				break;


		  	  default:

		  }

		  //uint32_t Txmailbox = CAN_TX_MAILBOX0;
		  uint32_t Txmailbox = 0xff;
		  //uint32_t* pTxmailbox = Txmailbox;

		  if (HAL_CAN_AddTxMessage(&hcan1, pTempCanTxHeader, datapayload, &Txmailbox) != HAL_OK)
		  {
			  //there is a problem with sending a CAN message...
			  CanTxError = 1;
		  }

		  CanTxMailboxfullmsg = FLAG_CLEAR; //allow TX mailbox full message to be displayed if the mailbox becomes full again!

	}

	else
	{
		//CAN TX mailbox is full - can't send CAN message
		CanTxError = 2;
	}

	return CanTxError;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

	//char tmpstr[200] = "";
	//char tempstring[200] = "";


	uint16_t CanIdentifier = 0;
	uint8_t CanDlc = 0;
	uint8_t CanData[8] = {0};

	uint8_t CanTxMsgCount = 0; //generic CAN TX message counter

	uint8_t candatacount = 0;

	uint8_t ProcessReceivedCanData = FLAG_CLEAR;

	uint8_t I2cInitialisationFunction = 0;
	uint8_t I2cReadBlockFunction = 0; //Main loop function controlling variable

	uint8_t recognisedstring = FLAG_CLEAR;
	uint8_t prevcommandstringstate = FLAG_CLEAR;

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
	uint16_t readdata = 0;
	uint8_t attemptcount = 0;

	uint16_t StepIndex = 0;
	uint16_t SeqCmdError = 0;
								//0x10 problem obtaining header data
								//0x11 received sequencer index specifies data that already exists
								//0x20 problem storing sequencer command string.

	 uint16_t MaxSequencerCycles = 0;

	 uint8_t Timer1AnalogHeartbeat = 1;
	 uint16_t ScanMsgCount = 0;

	 uint16_t stringlength = 0;
	 uint8_t Resetcontrol = 0;

	 pTempCanTxHeader->DLC = 8;
	 pTempCanTxHeader->IDE = CAN_ID_STD;
	 pTempCanTxHeader->RTR = CAN_RTR_DATA;
	 //pCanTxHeader->StdId = 0x234; //standard identifier
	 pTempCanTxHeader->StdId = 0x102; //standard identifier used for loop back mode
	 pTempCanTxHeader->ExtId = 0x00;
	 pTempCanTxHeader->TransmitGlobalTime = 0;

	 uint8_t IoTestStatus = 0x00;	//bit 0 controls main loop IO test function see serial command "IOTx"


	 tempfunction();

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





  uint8_t CanFilterErr = 0;
  CAN_FilterTypeDef FilterConfig;
  CAN_FilterTypeDef* pFilterConfig = &FilterConfig;
  pFilterConfig->FilterIdLow = 0x123u<<5;
  //pFilterConfig->FilterIdHigh = 0x345u;
  //pFilterConfig->FilterIdHigh = 0x123u<<5;
  pFilterConfig->FilterIdHigh = 0x000u<<5; //receive everything
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


  if (HAL_CAN_ConfigFilter(&hcan1, pFilterConfig) != HAL_OK)
  {
	  //problem with CAN filter setup!
	  CanFilterErr = 0x01;
  }


  //pFilterConfig->FilterIdHigh = 0x345u;
  //pFilterConfig->FilterIdHigh = 0x123u<<5;
  pFilterConfig->FilterIdHigh = 0x111u<<5;
  pFilterConfig->FilterIdLow = 0x254u<<5;
  pFilterConfig->FilterActivation = CAN_FILTER_ENABLE;
  pFilterConfig->FilterBank = 1;
  pFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
  //pFilterConfig->FilterMaskIdHigh = 0x345u<<5;
  pFilterConfig->FilterMaskIdHigh = 0x567u<<5;
  pFilterConfig->FilterMaskIdLow = 0x354u<<5;
  pFilterConfig->FilterMode = CAN_FILTERMODE_IDLIST;
  pFilterConfig->FilterScale = CAN_FILTERSCALE_16BIT;
  //pFilterConfig->SlaveStartFilterBank = 0;
  //pFilterConfig->SlaveStartFilterBank = 14;


  if (HAL_CAN_ConfigFilter(&hcan1, pFilterConfig) != HAL_OK)
  {
	  //problem with CAN filter setup!
	  CanFilterErr = CanFilterErr | 0x02;
  }



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




  //HAL_StatusTypeDef HAL_CAN_ActivateNotification (CAN_HandleTypeDef * hcan, uint32_t ActiveITs)
  uint8_t CanError = 0;

//  CAN_IT_ERROR_WARNING
//  CAN_IT_ERROR_PASSIVE
//  CAN_IT_BUSOFF
//  CAN_IT_LAST_ERROR_CODE
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING
		  | CAN_IT_RX_FIFO0_OVERRUN
		  | CAN_IT_RX_FIFO0_FULL
		  | CAN_IT_ERROR
		  | CAN_IT_BUSOFF
		  ) != HAL_OK)
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
  stringlength = strlen(tempstring);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
  UartMsgSent = FLAG_SET;

  while (UartMsgSent == FLAG_SET)
  {
  }

  //Home cursor
  strcpy(tempstring, "\e[H");
  stringlength = strlen(tempstring);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
  UartMsgSent = FLAG_SET;


  while (UartMsgSent == FLAG_SET)
  {
  }

  strcpy(tempstring, PROJECTSTRING);
  strcat(tempstring, "\r\n");
  stringlength = strlen(tempstring);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
  UartMsgSent = FLAG_SET;

  while (UartMsgSent == FLAG_SET)
  {
  }

  strcpy(tempstring, "Last edited: ");
  strcat(tempstring, DATESTRING);
  strcat(tempstring, "\r\n");
  stringlength = strlen(tempstring);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
  UartMsgSent = FLAG_SET;

  while (UartMsgSent == FLAG_SET) //flag cleared by UART Tx ISR
  {
  }

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

	  MainLoopProcess01(); //see serial command "SD?"
	  MainLoopProcess02(); //see serial command "?"


	//Test for Main loop display update
	  if (screenblock == FLAG_CLEAR) //flag is cleared to prevent automatic/ period screen updates
		  	  	  	  	  	  	  	  	//flag is cleared by 'escape' character
	  {
		  if ((ActuatorMsg2State & 0x20) != 0) //main control flag to enable processing of actuator msg 2 see serial command "AM2x"
		  {
			  if ((ActuatorMsg2State & 0x40) != 0) //test for actuator 2nd message status change
			  {
				  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				  {
					  strcpy(tempstring, "");
					  if ((ActuatorMsg2State & 0x80) != 0) //test for actuator 2nd message received
					  {
						  //actuator 2nd message has been received
						  strcpy(tempstring, "");
						  if ((ActuatorMsg2State & 0x01) != 0)
						  {

							  sprintf(tmpstr, "\e[3;60H\e[K\e[1;36;40mMotor temp:%3ddegC\e[0m", ActuatorMotorTemp);
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = ActuatorMsg2State & 0xFE; //reset control flag
						  }

						  if ((ActuatorMsg2State & 0x02) != 0)
						  {
							  sprintf(tmpstr, "\e[4;60H\e[K\e[1;36;40mPCB temp:%3ddegC\e[0m", ActuatorPcbTemp);
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = ActuatorMsg2State & 0xFD; //reset control flag
						  }
					  }

					  else
					  {
						  if ((ActuatorMsg2State & 0x10) == 0)
						  {
							  //sprintf(tmpstr, "\e[3;40H\e[K\e[7;37;41m-- NO CAN --\e[0m", ActuatorPcbTemp);
							  //sprintf(tmpstr, "\e[3;60H\e[K\e[7;31;47m-- NO CAN --\e[0m");
							  sprintf(tmpstr, "\e[3;60H\e[K\e[1;31;40m-- NO CAN 1-\e[0m"); //RED text, BLK Background
							  strcat(tempstring, tmpstr);
							  //sprintf(tmpstr, "\e[4;40H\e[K\e[7;37;41m-- NO CAN --\e[0m");
							  sprintf(tmpstr, "\e[4;60H\e[K\e[7;37;43m-- NO CAN 2-\e[0m");
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = ActuatorMsg2State | 0x10; //set flag; update flash state flag
						  }
						  else
						  {
							  //sprintf(tmpstr, "\e[3;40H\e[K\e[7;37;41m-- NO CAN --\e[0m", ActuatorPcbTemp);
							  //sprintf(tmpstr, "\e[3;60H\e[K\e[1;31;47m-- NO CAN --\e[0m");
							  sprintf(tmpstr, "\e[3;60H\e[K\e[1;37;41m-- NO CAN 3-\e[0m"); //WHI text, RED Background
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[4;60H\e[K\e[1;37;41m-- NO CAN 4-\e[0m");
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = ActuatorMsg2State & 0xEF; //reset flag; update flash state flag
						  }

					  }
					  ActuatorMsg2State = ActuatorMsg2State & 0xBF; //reset control flag

					  uint16_t stringlength = strlen(tempstring);
					  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					  UartMsgSent = FLAG_SET;
				  }
			  }
		  }



		  if((Shiftdemandfeedback & 0x80) != 0) //bit set once shift demand has been completed or terminated
		  {
			  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
			  {
				  //sprintf(tmpstr, "\e[6;12H\e[7;36;40m\e[KNO CAN POSITION SIGNAL\e[0m"); //set reverse video red text + reset attributes
				  sprintf(tmpstr, "\e[3;1H\e[KShift demand feedback:0x%01lX", Shiftdemandfeedback & 0x0F);
				  strcpy(tempstring, tmpstr);

				  uint16_t stringlength = strlen(tempstring);
				  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
				  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
				  UartMsgSent = FLAG_SET;

				  Shiftdemandfeedback = 0;

			  }
		  }


		  if ((ActuatorPositionState & 0x04) != 0) //see "AM1x" serial command, bit set by default
		  {
			  if (ActuatorPositionState & 0x01) //set when reported actuator position (over CAN) has changed or set by serial command "SC1" or CAN message timeout period expiring (TIM1 ISR)
			  {
					if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
					{

						if ((ActuatorPositionState & 0x02) == 0) //Test for actuator's 0x254 message received
						{
							if ((ActuatorPositionState & 0x08) == 0) //test message flash state
							{
								//CAN position signal has not been received
								//sprintf(tmpstr, "\e[6;12H\e[1;36;40mNO CAN POSITION SIGNAL\e[0m"); //set cyan text + reset attributes
								sprintf(tmpstr, "\e[6;12H\e[7;36;40m\e[KNO CAN POSITION SIGNAL\e[0m"); //set reverse video red text + reset attributes
								strcpy(tempstring, tmpstr);
								ActuatorPositionState = ActuatorPositionState | 0x08; //set message flash state flag
							}
							else
							{
								sprintf(tmpstr, "\e[6;12H\e[1;36;40m\e[KNO CAN POSITION SIGNAL\e[0m"); //set reverse video red text + reset attributes
								strcpy(tempstring, tmpstr);
								ActuatorPositionState = ActuatorPositionState & 0xF7; //reset message flash state flag
							}
						}
						else
						{
							sprintf(tmpstr, "\e[6;12H\e[1;36;40m\e[KActuator pos:%d\e[0m", ActuatorPosition); //set cyan text + reset attributes
							strcpy(tempstring, tmpstr);
						}
						uint16_t stringlength = strlen(tempstring);
						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;


						ActuatorPositionState = ActuatorPositionState & 0xFE; //reset control bit
					}

			  }
		  }
	  }

	  if (CanAnalogScanState != 0) //function controlled by serial command "CASy"
	  {
		  //scan CAN and analogue signals together
		  if ((CanAnalogScanState & 0x02) != 0)
		  {
			  //copy scan value to both CAN message and analogue outputs
			  CanAnalogScanState = CanAnalogScanState & 0xFD; //reset output update bit

			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)ScanValue);


			  uint32_t x = 0;
			  uint8_t CanTxError = 0;
			  x = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
			  if (x != 0)
			  {
				  uint8_t datapayload[8] = {0};

				  datapayload[0] = (uint8_t)(ScanValue >> 8);
				  datapayload[1] = (uint8_t)(ScanValue);

				  datapayload[4] = (uint8_t)(ScanMsgCount>> 8);
				  datapayload[5] = (uint8_t)ScanMsgCount;
				  ScanMsgCount++;

				  //uint32_t Txmailbox = CAN_TX_MAILBOX0;
				  uint32_t Txmailbox = 0xff;
				  //uint32_t* pTxmailbox = Txmailbox;

				  if (HAL_CAN_AddTxMessage(&hcan1, pCanTxHeader, datapayload, &Txmailbox) != HAL_OK)
				  {
					  //there is a problem with sending a CAN message...
					  CanTxError = 1;
				  }
				  else
				  {
					  ScanMsgCount++;
				  }
				  CanTxMailboxfullmsg = FLAG_CLEAR; //allow TX mailbox full message to be displayed if the mailbox becomes full again!

			  }
			  else
			  {

			  }

			  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
			  {
				  if (UpdateScreen != 0)
				  {
					  UpdateScreen = 0; //reset flag.

					  sprintf(tempstring, "\e[4;1Scan value %4d", ScanValue);

					  if (x != 0)
					  {
						  if(CanTxError != 0)
						  {
							  sprintf(tmpstr, "\e[5;1H\e[KCAN error detected");
							  strcat(tempstring, tmpstr);
						  }
						  else
						  {
							sprintf(tmpstr, "\e[5;1H\e[KCAN message count:%d", ScanMsgCount);
							strcat(tempstring, tmpstr);
						  }
					  }
					  else
					  {
						  if (CanTxMailboxfullmsg == FLAG_CLEAR)
						  {
							  if (screenblock == FLAG_CLEAR)
							  {
								  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
								  {
									  sprintf(tempstring, "\e[6;1HCAN TX mailboxes FULL!");
									  //uint16_t stringlength = strlen(tempstring);
									  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
									  //HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
									  //UartMsgSent = FLAG_SET;
									  CanTxMailboxfullmsg = FLAG_SET;
								  }
							  }
						  }
					  }

					  uint16_t stringlength = strlen(tempstring);
					  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					  UartMsgSent = FLAG_SET;

				  }
			  }

		  }
	  }

	  if (GetSequencerState() != 0)
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
			  //decode the first entry in the buffer memory, set outputs and start step timer
			  SeqStepTime = GetSeqStepTime();
			  SetSequencerState(STEPTIMERUNNING);
		  }
	  }

	  if (Resetcontrol != 0)
	  {
		  if (UartMsgSent == FLAG_CLEAR)
		  {
			  if (FunctionDelay == 0)
			  {
				  HAL_NVIC_SystemReset();
			  }
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
						xmodempacketblockno = 1; //first x-modem block is labelled 1!
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
					//waiting for packet characters to arrive OR timeout period to expire
					if (FunctionDelay == 0) //test for timeout expiry.
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
					tempstring[0] = ACKCHAR; //ACK (0x06)
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
					XmodemStatus = 5; //terminate x-modem function
					break;

				  case(7):
					//if (FunctionDelay == 0) //2OCT2025: delay handling now taken care of by step 5
					//{
						sprintf(tmpstr, "\e[3;1H\e[KX-modem download finished"); //clear line
						strcpy(tempstring, tmpstr);

						uint16_t stringlength = strlen(tempstring);

						//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						UartMsgSent = FLAG_SET;
						XmodemStatus = 0;
					//}
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

								uint8_t data = 0;
								//for (uint8_t i=0; i<RxStringLen; i++)
								for (uint8_t i=0; i<128; i++)
								{
								  data = RxString[i+3];
								  CalculareCrc16(data);
								  Tempdata[opwriteptr] = data; //copy data payload bytes to a circular buffer
								  opwriteptr++;
								  if (opwriteptr >= OPBUFFERSIZE)
								  {
									  opwriteptr = 0;
								  }
								  opbytecount++;
								}

								//test calculated CRC with received value
								uint16_t CalcCrcValue = GetCrc16Val();
								uint8_t tempval = 0;
								if ((uint8_t)(CalcCrcValue >> 8) == RxString[131])
								{
									if ((uint8_t)(CalcCrcValue) == RxString[132])
									{
										//CRC value calculated/received correctly
										tempval = 1;

									}

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
									xmodempacketblockno++; //prepare for next packet reception
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
					while (opbytecount > 0) //test packet data payload count
					{
						data = Tempdata[opreadptr];

						//eliminate LF character
						uint8_t processlinechar = 1;
						if (linecharcount == 0)
						{
							if (data == LFCHAR) //test for Line feed (LF) character
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

						if (data == CRCHAR)
						{

							if (linecount < 64)
							{
								for (uint8_t i=0; i<linecharcount; i++)
								{
									if (i == (MAXLINELENGTH - 1))
									{
										break;
									}
									linearray[(linecount * MAXLINELENGTH) + i] = linestring[i];
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
					if (linestring[linecharcount-1] == CRCHAR) //test for complete line
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

										response = WriteSeqData(commanddata, cmddatacounter + 2, address);

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
										tempval = CalculateBlockCrc(byteptr, SEQMATENTRYSIZE-2);
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
							//linestring buffer my hold an incomplete line which will be completed with the next x-modem packet
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
							XmodemStatus = 5;
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
			//main loop function initiated by serial command "I2CFxx"
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
							blockcount = 0;
							readdata = 0;
							attemptcount = 0;
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
								uint32_t response = 0;
								uint8_t currentblock = 0;
								uint16_t stringlength = 0;

								uint16_t reminingdataqty = tempstruct.I2cQuantity - readdata;
								if(reminingdataqty > BLOCKSIZE)
								{
									currentblock = BLOCKSIZE;
								}
								else
								{
									currentblock = reminingdataqty;
								}
								blockcount++;

								for (uint8_t i = 0; i<BLOCKSIZE-1; i++)
								{
									*(byteptr + 1 + i) = *byteptr;
								}


								//uint32_t I2cWriteBlock(uint8_t DeviceAddress, uint16_t InternalAddress, uint8_t InternalAddressWidth, uint8_t* srcdata, uint8_t qty);
								response = I2cWriteBlock(SEQUENCERMEMORY, address, 2, byteptr, currentblock);

								//need to check her for device busy as device might still be writing the last block of data....

								if (response == 1)
								{
									//I2C device was busy, prepare to try again...
									attemptcount++;
									if (attemptcount > 100)
									{
										FillI2cMemoryFunction = 4;
									}
									else
									{
										FunctionDelay = 10;
									}
								}

								if (response == 0)
								{
									address = address + currentblock;
									readdata = readdata + currentblock;
									if (readdata >= tempstruct.I2cQuantity)
									{
										//block fill is complete
										FillI2cMemoryFunction = 3;
									}
									else
									{
										//continue around this loop until the specified block has been filled
										sprintf(tmpstr, "\e[8;1H\e[KBlock count:0x%02X, attempt:0x%02X", blockcount, attemptcount);
										strcpy(tempstring, tmpstr);

										stringlength = strlen(tempstring);
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
								sprintf(tmpstr, "\e[8;1H\e[KNo Data specified! (Quantity:0x%04X)", tempstruct.I2cQuantity);
								strcpy(tempstring, tmpstr);

								stringlength = strlen(tempstring);
								//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
								HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
								UartMsgSent = FLAG_SET;
								FillI2cMemoryFunction = 4;
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
						strcat(tempstring, tmpstr);


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
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {
			  //clear message from display
			  sprintf(tempstring, "\e[4;1H\e[KCAN error reported:0x%08lX", CanErrorValue); //move cursor to 4th line, clear text,
			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;



			  CanState = CanState & 0xFE; //clear control flag
		  }

	  }


	  if (CanRxFifoFull != 0)
	  {
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {
			  sprintf(tempstring, "\e[20;1H\e[KCAN Rx FIFO full!");

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
			  CanRxFifoFull = 0;
		  }
	  }


	  if (ProcessReceivedCanData == FLAG_CLEAR) //don't grab more data until the previous data has been processed and displayed.
	  {
		  if (CanDataReceived == FLAG_SET) //test for CAN receive complete callback activity
		  {
			  //grab a copy of the received data
			  CanIdentifier = pTempCanRxHeader->StdId;
			  CanDlc = pTempCanRxHeader->DLC;
			  for (uint8_t i=0; i<CanDlc; i++)
			  {
				  CanData[i] = TempCanRxData[i];
			  }

			  ProcessReceivedCanData = FLAG_SET;
			  CanDataReceived = FLAG_CLEAR;
			  //output received CAN data
		  }
	  }


	  if (ProcessReceivedCanData == FLAG_SET)
	  {
		  if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
		  {
			  char tempstring2[40] = "";
			  sprintf(tempstring, "\e[21;1H\e[KCAN Id:0x%3X, DLC:%d, data:", CanIdentifier, CanDlc);
			  for (uint8_t i=0; i<CanDlc; i++)
			  {
				  sprintf(tempstring2, "0x%02X,", CanData[i]);
				  strcat(tempstring, tempstring2);
			  }

			  sprintf(tempstring2, "(c:%d)", recmsgindex);
			  strcat(tempstring, tempstring2);

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;
			  ProcessReceivedCanData = FLAG_CLEAR;
		  }


	  }




	  if (Timer1AnalogHeartbeat != 0)
	  {
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
	  }


	  if (timer1heartbeat == FLAG_SET) //test flag set by elapsed time ISR
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
						  sprintf(tempstring, "\e[20;1H\e[KCAN Tx message %3d", CanTxMsgCount);
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
						  sprintf(tempstring, "\e[20;1H\e[KCAN TX mailboxes FULL!");
						  uint16_t stringlength = strlen(tempstring);
						  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
						  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
						  UartMsgSent = FLAG_SET;
						  CanTxMailboxfullmsg = FLAG_SET;
					  }
				  }
			  }
		  }


		  if (IoTestStatus == 0x01)
		  {
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
		  }


		  if (HAL_GPIO_ReadPin(MODE1_GPIO_Port, MODE1_Pin) != 0) //main loop monitoring of key switch input
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
			  prevcommandstringstate = FLAG_SET; //used to allow previous commad string acknowledgement to be cleared as soon as the next string is started


			  if (commandlength == 1) //test for 1 character commands
			  {
				  comp = strcmp(RxString, "R");
				  if (comp == 0)
				  {
					  //"R": Prepare to reset sequencer
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Sequencer resetting...");
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;


					  Resetcontrol = 1;
					  FunctionDelay = 1000; //value decremented by TIM ISR before actual reset
					  //HAL_NVIC_SystemReset();
				  }

				  comp = strcmp(RxString, "?");
				  if (comp == 0)
				  {
					  //"?": Display serial commands
					  screenblock = FLAG_SET; //prevent other main loop processed overwriting the screen
					  ProcessIndex = 0x02; //Enable main loop process
					  ProcessCount = 0;
					  recognisedstring = FLAG_SET;

				  }

			  }

			  if (commandlength == 2) //test for 2 character commands
			  {

				  comp = strcmp(RxString, "CH");
				  if (comp == 0)
				  {
					  //Check sequencer header data

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Checking header block:");
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
						  sprintf(tmpstr, "(error:0x%02lX)", tempval); //reset all attributes
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

			  if (commandlength == 3) //test for 3 character commands
			  {

				  comp = strcmp(RxString, "CDN");
				  if (comp == 0)
				  {
					  //"CDN": CAN downshift
					  //initiate CAN downshift with preload pulses
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  strcat(tmpstr, "CAN downshift");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0xC0) == 0xC0)
					  {
						  ShiftDemand = 0x40;
						  ShiftDemand = ShiftDemand | 0x01;
						  //ShiftDemandPulse = 100; //set duration of shift demand pulse
	//					  PreloadPullActivationtime = 50;
	//					  PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
	//					  PreloadPushActivationtime = 50;
	//					  PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse
					  }
					  else
					  {
						  strcpy(tmpstr, " Unavailable!");
						  strcat(tempstring, tmpstr);
					  }
					  recognisedstring = FLAG_SET;
				  }



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
						  sprintf(tmpstr, "(error = 0x%02lX)", response); //reset all attributes
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

				  comp = strcmp(RxString, "CUP");
				  if (comp == 0)
				  {
					  //"CUP": CAN upshift
					  //initiate CAN up shift with preload pulses

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  strcat(tmpstr, "CAN upshift");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0xC0) == 0xC0)
					  {
						  ShiftDemand = 0x80;
						  ShiftDemand = ShiftDemand | 0x01;
	//					  ShiftDemandPulse = 100; //set duration of shift demand pulse
	//					  PreloadPullActivationtime = 50;
	//					  PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
	//					  PreloadPushActivationtime = 50;
	//					  PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse
					  }
					  else
					  {
						  strcpy(tmpstr, " Unavailable!");
						  strcat(tempstring, tmpstr);
					  }
					  recognisedstring = FLAG_SET;
				  }


				  comp = strcmp(RxString, "LDN");
				  if (comp == 0)
				  {
					  //"LDN": logic level downshift
					  //initiate logic level down shift with preload pulses

					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  strcat(tmpstr, "Logic level downshift");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0)
					  {
						  ShiftDemand = 0x20;
						  ShiftDemand = ShiftDemand | 0x01;
	//					  ShiftDemandPulse = 100; //set duration of shift demand pulse
	//					  PreloadPullActivationtime = 50;
	//					  PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
	//					  PreloadPushActivationtime = 50;
	//					  PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse
					  }
					  else
					  {
						  strcpy(tmpstr, " Unavailable!");
						  strcat(tempstring, tmpstr);
					  }



					  recognisedstring = FLAG_SET;
				  }

				  comp = strcmp(RxString, "LUP");
				  if (comp == 0)
				  {
					  //"LUP": logic level upshift
					  //initiate logic level up shift with preload pulses
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  strcat(tmpstr, "Logic level upshift");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
					  {
						  ShiftDemand = 0x10;
						  ShiftDemand = ShiftDemand | 0x01;
	//					  ShiftDemandPulse = 100; //set duration of shift demand pulse
	//					  PreloadPullActivationtime = 50;
	//					  PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
	//					  PreloadPushActivationtime = 50;
	//					  PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse
					  }
					  else
					  {
						  strcpy(tmpstr, " Unavailable!");
						  strcat(tempstring, tmpstr);
					  }
					  recognisedstring = FLAG_SET;
				  }

				  comp = strncmp(RxString, "SC", 2);
				  if (comp == 0)
				  {
					  if (RxString[2] == '1')
					  {
						  //"SC1": Enable dedicated shift demand sequencing

						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Dedicated shift demand sequencing ENABLED");
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  DedicatedShiftControl = DedicatedShiftControl | 0x80;
						  ActuatorPositionState = 0x01; //initiate displaying of CAN actuator position

						  //ActuatorMsg2Timeoutcount = ActuatorMsg2TimeoutPeriod; //enable monitoring of actuator 2nd CAN message

						  recognisedstring = FLAG_SET;
					  }
					  else
					  {
						  //"SC0"
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Dedicated shift demand sequencing DISABLED");
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  DedicatedShiftControl = 0;

						  recognisedstring = FLAG_SET;
					  }
				  }

				  comp = strncmp(RxString, "SD?", 3);
				  if (comp == 0)
				  {
					  //serial command "SD?"
					  //split into a main loop process

					  screenblock = FLAG_SET; //prevent other main loop processed overwriting the screen
					  ProcessIndex = 0x01; //Enable main loop process
					  ProcessCount = 0;
					  recognisedstring = FLAG_SET;

				  }

				  //comp = strcmp(RxString, "XF"); //"XFy" set X-modem received data format
				  comp = strncmp(RxString, "XF", 2); //"XFy" set X-modem received data format
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

					  if (dataformat != 99)
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

			  if (commandlength == 4) //4 character command strings
			  {

				  comp = strncmp(RxString, "AM", 2); //"AM"
				  if (comp == 0)
				  {
					  if (RxString[2] == '1')
					  {
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Actuator message 1 processing ");

						  if (RxString[3] == '1')
						  {
							  //"AM11": Enable processing of actuator messzage 2
							  sprintf(tmpstr, "Enabled"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
							  ActuatorPositionState = ActuatorPositionState | 0x04;
							  PositionSignalTimeoutCount = PositionSignalTimeoutPeriod; //reset timeout period, value decremented by TIM1 ISR
							  recognisedstring = FLAG_SET;
						  }
						  else
						  {
							  //"AM10": disable processing of actuator messzage 2
							  sprintf(tmpstr, "Disabled"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
							  ActuatorPositionState = ActuatorPositionState & 0xFB;
							  recognisedstring = FLAG_SET;
						  }
					  }


					  if (RxString[2] == '2')
					  {
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Actuator message 2 processing ");

						  if (RxString[3] == '1')
						  {
							  //"AM21": Enable processing of actuator messzage 2
							  sprintf(tmpstr, "Enabled"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = 0x20;
							  ActuatorMsg2Timeoutcount = ActuatorMsg2TimeoutPeriod; //enable monitoring of actuator 2nd CAN message
							  recognisedstring = FLAG_SET;
						  }
						  else
						  {
							  //"AM20": disable processing of actuator messzage 2
							  sprintf(tmpstr, "Disabled"); //move cursor to 3rd line, clear text,
							  strcat(tempstring, tmpstr);
							  ActuatorMsg2State = 0;
							  recognisedstring = FLAG_SET;
						  }
					  }
				  }


				  //comp = strcmp(RxString, "CAS"); //"CASy": CAN Analogue scan
				  comp = strncmp(RxString, "CAS", 3); //"CASy": CAN Analogue scan
				  if (comp == 0)
				  {
					  if (RxString[3] == '0')
					  {
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "CAN Analog Scan function disabled");
						  CanAnalogScanState = 0; //flag to main loop
						  recognisedstring = FLAG_SET;
					  }

					  else if (RxString[3] == '1')
					  {
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "CAN Analog Scan");

						  CanAnalogScanState = 1; //flag to main loop
						  ScanValue = 0;
						  ScanUpdatetimeRefreshValue = 1;
						  ScanUpdateTime = ScanUpdatetimeRefreshValue;
						  Timer1AnalogHeartbeat = 0;
						  ScanMsgCount = 0;
						  screenblock = FLAG_CLEAR;
						  recognisedstring = FLAG_SET;
					  }
				  }

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


				  comp = strncmp(RxString, "IOT", 3); //"CASy": CAN Analogue scan
				  if (comp == 0)
				  {
					  if (RxString[3] == '0')
					  {
						  //sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "IO Test Disabled");
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  IoTestStatus = 0;
						  recognisedstring = FLAG_SET;

					  }
					  if (RxString[3] == '1')
					  {
						  //sprintf(tmpstr, "\e[3;1H\e[K\e[1;37;42m"); //move cursor to 3rd line, clear text, white text on green background
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "IO Test Enabled");
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  IoTestStatus = 0x01;
						  recognisedstring = FLAG_SET;

					  }
				  }


				  comp = strcmp(RxString, "MCDN");
				  if (comp == 0)
				  {
					  //"MCDN" Multiple CAN downshift sequence
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Multiple CAN down shifts");
					  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
					  strcat(tempstring, tmpstr);

					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;

					  strcpy(tmpstr, "\e[4;1H");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
					  {
						  if ((DedicatedShiftControl & 0x40) != 0) //see serial command "SCCx"
						  {
							  if ((DedicatedShiftControl & 0x20) != 0) //see serial command "RPCx"
							  {
								  //CAN feedback specified
								  if ((ActuatorPositionState & 0x02) == 0) //test for CAN position message reception
								  {
									  strcpy(tmpstr, "Activated (1)");
									  strcat(tempstring, tmpstr);

									  ShiftDemandCount = 0;
									  ShiftDemand = 0x40;
									  ShiftDemand = ShiftDemand | 0x01;
									  Multishift = 0x80;
								  }
								  else
								  {
									  strcpy(tmpstr, "Error: CAN Feedback required!");
									  strcat(tempstring, tmpstr);
								  }
							  }
							  else
							  {
								  strcpy(tmpstr, "Activated (2)");
								  strcat(tempstring, tmpstr);

								  ShiftDemandCount = 0;
								  ShiftDemand = 0x40;
								  ShiftDemand = ShiftDemand | 0x01;
								  Multishift = 0x80;
							  }
						  }
						  else
						  {
							  strcpy(tmpstr, "Error: CAN shift sequencing not enabled");
							  strcat(tempstring, tmpstr);
						  }
					  }
					  else
					  {
						  strcpy(tmpstr, "Shift Sequencing NOT enabled");
						  strcat(tempstring, tmpstr);
					  }

				  }

				  comp = strcmp(RxString, "MCUP");
				  if (comp == 0)
				  {
					  //"MCUP" Multiple CAN upshift sequence
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Multiple CAN up shifts");
					  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
					  strcat(tempstring, tmpstr);

					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;

					  strcpy(tmpstr, "\e[4;1H");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
					  {
						  if ((DedicatedShiftControl & 0x40) != 0) //see serial command "SCCx"
						  {
							  if ((DedicatedShiftControl & 0x20) != 0) //see serial command "RPCx"
							  {
								  //CAN feedback specified
								  if ((ActuatorPositionState & 0x02) == 0) //test for CAN position message reception
								  {
									  strcpy(tmpstr, "Activated (1)");
									  strcat(tempstring, tmpstr);

									  ShiftDemandCount = 0;
									  ShiftDemand = 0x80;
									  ShiftDemand = ShiftDemand | 0x01;
									  Multishift = 0x80;
								  }
								  else
								  {
									  strcpy(tmpstr, "Error: CAN Feedback required!");
									  strcat(tempstring, tmpstr);
								  }
							  }
							  else
							  {
								  strcpy(tmpstr, "Activated (2)");
								  strcat(tempstring, tmpstr);

								  ShiftDemandCount = 0;
								  ShiftDemand = 0x80;
								  ShiftDemand = ShiftDemand | 0x01;
								  Multishift = 0x80;
							  }
						  }
						  else
						  {
							  strcpy(tmpstr, "Error: CAN shift sequencing not enabled");
							  strcat(tempstring, tmpstr);
						  }
					  }
					  else
					  {
						  strcpy(tmpstr, "Shift Sequencing NOT enabled");
						  strcat(tempstring, tmpstr);
					  }
				  }

				  comp = strcmp(RxString, "MLDN");
				  if (comp == 0)
				  {
					  //"MLDN" Multiple logic level downshift sequence

					  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Multiple logic level down shifts:");

					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;

					  strcpy(tmpstr, "\e[4;1H");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
					  {

						  if ((DedicatedShiftControl & 0x20) != 0) //see serial command "RPCx"
						  {
							  //CAN feedback specified
							  if ((ActuatorPositionState & 0x02) == 0) //test for CAN position message reception
							  {
								  //Actuator position CAN message present
								  strcpy(tmpstr, "Activated (1)");
								  strcat(tempstring, tmpstr);

								  ShiftDemandCount = 0;
								  ShiftDemand = 0x20;
								  ShiftDemand = ShiftDemand | 0x01;
								  Multishift = 0x80;
							  }
							  else
							  {
								  strcpy(tmpstr, "Error: CAN Feedback signal required!");
								  strcat(tempstring, tmpstr);
							  }
						  }
						  else
						  {
							  strcpy(tmpstr, "Activated (2)");
							  strcat(tempstring, tmpstr);

							  ShiftDemandCount = 0;
							  ShiftDemand = 0x20;
							  ShiftDemand = ShiftDemand | 0x01;
							  Multishift = 0x80;
						  }

					  }
					  else
					  {
						  strcpy(tmpstr, "Shift Sequencing NOT enabled");
						  strcat(tempstring, tmpstr);
					  }

				  }

				  comp = strcmp(RxString, "MLUP");
				  if (comp == 0)
				  {
					  //"MLUP" Multiple logic level upshift sequence

					  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 5th line, clear text,
					  strcat(tempstring, tmpstr);
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Multiple logic level up shifts:");

					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;

					  strcpy(tmpstr, "\e[4;1H");
					  strcat(tempstring, tmpstr);

					  if ((DedicatedShiftControl & 0x80) != 0) //see serial command "SCx"
					  {
						  if ((DedicatedShiftControl & 0x20) != 0) //see serial command "RPCx"
						  {
							  //CAN feedback specified
							  if ((ActuatorPositionState & 0x02) == 0) //test for CAN position message reception
							  {
								  //Actuator position CAN message present
								  strcpy(tmpstr, "Activated (1)");
								  strcat(tempstring, tmpstr);

								  ShiftDemandCount = 0;
								  ShiftDemand = 0x10;
								  ShiftDemand = ShiftDemand | 0x01;
								  Multishift = 0x80;
							  }
							  else
							  {
								  strcpy(tmpstr, "Error: CAN Feedback signal required!");
								  strcat(tempstring, tmpstr);
							  }
						  }
						  else
						  {
							  strcpy(tmpstr, "Activated (2)");
							  strcat(tempstring, tmpstr);

							  ShiftDemandCount = 0;
							  ShiftDemand = 0x10;
							  ShiftDemand = ShiftDemand | 0x01;
							  Multishift = 0x80;
						  }

					  }
					  else
					  {
						  strcpy(tmpstr, "Shift Sequencing NOT enabled");
						  strcat(tempstring, tmpstr);
					  }

				  }

				  comp = strncmp(RxString, "RPC", 3);
				  if (comp == 0)
				  {
					  if (RxString[3] == '0')
					  {
						  //Serial command "RPC0": Disable reading of P1399 CAN position
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Shift control irrespective of CAN position");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  DedicatedShiftControl = DedicatedShiftControl & 0xDF; //clear bit
						  recognisedstring = FLAG_SET;
					  }

					  if (RxString[3] == '1')
					  {
						  //Serial command "RPC1": Enable reading of P1399 CAN position (enabled by default)
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Shift control based on reported CAN position");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  DedicatedShiftControl = DedicatedShiftControl | 0x20; //set bit
						  ActuatorPositionState = ActuatorPositionState | 0x01; //update display with status
						  recognisedstring = FLAG_SET;
					  }

				  }

				  comp = strncmp(RxString, "SCC", 3);
				  if (comp == 0)
				  {
					  if (RxString[3] == '0')
					  {
						  //"SCC0"
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Shift control sequencing CAN Disabled");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);
						  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  DedicatedShiftControl = DedicatedShiftControl & 0xBF; //disable shift demand sequence CAN output

						  recognisedstring = FLAG_SET;


					  }
					  if (RxString[3] == '1')
					  {
						  //"SCC1"
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Shift control CAN sequencing:");
						  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  if ((DedicatedShiftControl & 0x20) != 0)
						  {
							  //CAN position feedback required
							  if ((ActuatorPositionState & 0x02) != 0) //see serial command "RPCx"
							  {
								  //actuator CAN positon has been reported
								  strcat(tempstring, "Enabled");
								  DedicatedShiftControl = DedicatedShiftControl | 0x40; //Enable shift demand sequence CAN output
							  }
							  else
							  {
								  strcat(tempstring, "Error: CAN position NOT fedback!");
							  }

						  }
						  else
						  {
							  strcat(tempstring, "Enabled; CAN position not required");
							  DedicatedShiftControl = DedicatedShiftControl | 0x40; //Enable shift demand sequence CAN output
						  }

						  //sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
						  //strcat(tempstring, tmpstr);
						  //sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
						  //strcat(tempstring, tmpstr);

						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);

						  recognisedstring = FLAG_SET;
					  }
				  }


				  comp = strncmp(RxString, "SEQ", 3);
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

							  sprintf(tmpstr, "Max steps:%5d", MaxSequencerSteps); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);

							  MaxSequencerCycles = (uint16_t)(*(byteptr + 6)) << 8 | (uint16_t)(*(byteptr+7));
							  sprintf(tmpstr, "Max cycles:%5d", MaxSequencerCycles); //reset all attributes
							  strcat(tempstring, tmpstr);
							  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to 4th line, clear text,
							  strcat(tempstring, tmpstr);


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
							  sprintf(tmpstr, "(error:0x%02lX)", tempval); //reset all attributes
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


			  if (commandlength == 5) //5 character command strings
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


			  if (commandlength == 6)	//6 character command strings
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

				  comp = strncmp(RxString, "I2CF", 4); //I2CFxx
				  if (comp == 0)
				  {

					  	//Fill block of I2C memory
						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Fill I2C memory: ");
						sprintf(tmpstr, "\e[4;1H\e[K"); //clear line
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "\e[5;1H\e[K"); //
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "\e[6;1H\e[K"); //
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "\e[7;1H\e[K"); //
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "\e[8;1H\e[K"); //
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

				  comp = strncmp(RxString, "SD", 2); //"SDxxxx"
				  if (comp == 0)
				  {
					  //"SDxxxx": Set shift demand pulse width
						sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						strcat(tempstring, "Set shift demand pulse width: ");


						uint32_t UserVal = 0;
						UserVal = ExtractValueFromString(RxString, 2, 4);

						if ((UserVal & 0x80000000) == 0)
						{
							//now convert ascii decimal string into hex
							//uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
							UserVal = Bcd2Hex(UserVal & 0xFFFF);


							sprintf(tmpstr, "\e[4;1H\e[K%4ld msec", UserVal);
							strcat(tempstring, tmpstr);
							ShiftDemandPulseTime = UserVal;

						}
						else
						{
							sprintf(tmpstr, "Data value invalid!\r\n");
							strcat(tempstring, tmpstr);
							//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						}

						recognisedstring = FLAG_SET;


						sprintf(tmpstr, "\e[0m"); //reset all attributes
						strcat(tempstring, tmpstr);
						recognisedstring = FLAG_SET;

				  }

			  }

			  if (commandlength == 7) //7 character command strings
			  {
				  comp = strncmp(RxString, "PP", 2); //PP
				  if (comp == 0)
				  {
					  if (RxString[2] == 'A')
					  {
						  //"PPAxxxx": Set preload pull activation delay time
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Set Preload pull activation delay time: ");

						  uint32_t UserVal = 0;
						  UserVal = ExtractValueFromString(RxString, 3, 4);

						  if ((UserVal & 0x80000000) == 0)
						  {
							  //now convert ascii decimal string into hex
							  //uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
							  UserVal = Bcd2Hex(UserVal & 0xFFFF);

							  //function = UserVal & 0xFF;
							  //functionRun = 1; //allow test function to execute within main loop

							  sprintf(tmpstr, "\e[4;1H\e[K%4ld msec", UserVal);
							  strcat(tempstring, tmpstr);
							  PreloadPullActivationTime = UserVal;

						  }
						  else
						  {
							  sprintf(tmpstr, "Data value invalid!\r\n");
							  strcat(tempstring, tmpstr);
							  //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						  }
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;

					  }

					  if (RxString[2] == 'B')
					  {
						  //"PPBxxxx": Set Prelaod pull pulse width
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Set Preload pull activation time: ");

						  uint32_t UserVal = 0;
						  UserVal = ExtractValueFromString(RxString, 3, 4);

						  if ((UserVal & 0x80000000) == 0)
						  {
							  //now convert ascii decimal string into hex
							  //uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
							  UserVal = Bcd2Hex(UserVal & 0xFFFF);

							  //function = UserVal & 0xFF;
							  //functionRun = 1; //allow test function to execute within main loop

							  sprintf(tmpstr, "\e[4;1H\e[K%4ld msec", UserVal);
							  strcat(tempstring, tmpstr);
							  PreloadPullDemandPulseTime = UserVal;

						  }
						  else
						  {
							  sprintf(tmpstr, "Data value invalid!\r\n");
							  strcat(tempstring, tmpstr);
							  //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						  }
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;

					  }

					  if (RxString[2] == 'C')
					  {
						  //"PPCxxxx": Set Prelaod push activation delay time
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Set Preload push activation delay time: ");

						  uint32_t UserVal = 0;
						  UserVal = ExtractValueFromString(RxString, 3, 4);

						  if ((UserVal & 0x80000000) == 0)
						  {
							  //now convert ascii decimal string into hex
							  //uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
							  UserVal = Bcd2Hex(UserVal & 0xFFFF);

							  //function = UserVal & 0xFF;
							  //functionRun = 1; //allow test function to execute within main loop

							  sprintf(tmpstr, "\e[4;1H\e[K%4ld msec", UserVal);
							  strcat(tempstring, tmpstr);
							  PreloadPushActivationTime = UserVal;

						  }
						  else
						  {
							  sprintf(tmpstr, "Data value invalid!\r\n");
							  strcat(tempstring, tmpstr);
							  //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						  }
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;

					  }

					  if (RxString[2] == 'D')
					  {
						  //"PPDxxxx": Set Preload push demand time
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcpy(tempstring, tmpstr);
						  strcat(tempstring, "Set Preload push activation pulse time: ");

						  uint32_t UserVal = 0;
						  UserVal = ExtractValueFromString(RxString, 3, 4);

						  if ((UserVal & 0x80000000) == 0)
						  {
							  //now convert ascii decimal string into hex
							  //uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
							  UserVal = Bcd2Hex(UserVal & 0xFFFF);

							  //function = UserVal & 0xFF;
							  //functionRun = 1; //allow test function to execute within main loop

							  sprintf(tmpstr, "\e[4;1H\e[K%4ld msec", UserVal);
							  strcat(tempstring, tmpstr);
							  PreloadPushDemandPulseTime = UserVal;

						  }
						  else
						  {
							  sprintf(tmpstr, "Data value invalid!\r\n");
							  strcat(tempstring, tmpstr);
							  //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
						  }
						  sprintf(tmpstr, "\e[0m"); //reset all attributes
						  strcat(tempstring, tmpstr);
						  recognisedstring = FLAG_SET;

					  }
				  }
				  comp = strncmp(RxString, "RSD", 3); //RSD
				  if (comp == 0)
				  {
					  //"RSDxxxx": Repeat shift demand delay time
					  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
					  strcpy(tempstring, tmpstr);
					  strcat(tempstring, "Set repeat shift demand delay time: ");

					  uint32_t UserVal = 0;
					  UserVal = ExtractValueFromString(RxString, 3, 4);

					  if ((UserVal & 0x80000000) == 0)
					  {
						  //now convert ascii decimal string into hex
						  //uint32_t Bcd2Hex(uint32_t InputVal); //function to convert from BCD string into single Hex value
						  UserVal = Bcd2Hex(UserVal & 0xFFFF);

						  //function = UserVal & 0xFF;
						  //functionRun = 1; //allow test function to execute within main loop

						  sprintf(tmpstr, "\e[4;1H\e[K0x%4ld msec", UserVal);
						  strcat(tempstring, tmpstr);
						  Shift2ShiftTime = UserVal;

					  }
					  else
					  {
						  sprintf(tmpstr, "Data value invalid!\r\n");
						  strcat(tempstring, tmpstr);
						  //HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
					  }
					  sprintf(tmpstr, "\e[0m"); //reset all attributes
					  strcat(tempstring, tmpstr);
					  recognisedstring = FLAG_SET;
				  }
			  }


			  if (commandlength == 8) //8 character command strings
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


			  if (commandlength == 10) //10 character command strings
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

			  if (commandlength > 3) // >3 character command strings
			  {
				  comp = strncmp(RxString, "ESC", 3); //ESC*
				  if (comp == 0)
				  {
					  //get length of string
					  uint16_t stringlength = commandlength - 3;
					  char* charptr = NULL;
					  charptr = tmpstr;
					  for (uint8_t j=0; j<stringlength; j++)
					  {
						  *(charptr + j) = RxString[3+j]; //copy escape sequence characters
					  }

					  //problem here is that the escape sequence is going to get overwritten by the main loop display routines...

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
			  //printf(tmpstr, "\e[2;1H\e[K"); //move cursor to 2nd line, clear
			  sprintf(tmpstr, "\e[2J\e[H"); //clear screen and home cursor
			  strcpy(tempstring, tmpstr);

			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;


			  //End of escape functions
			  //update CAN position status
			  if ((DedicatedShiftControl & 0x20) != 0) //see "RPC1" serial command, bit set by default
			  {
				  ActuatorPositionState = ActuatorPositionState | 0x01; //flag to main loop code
			  }

			  if ((ActuatorMsg2State & 0x20) != 0) //test flag set by serial command "AM21"
			  {
				  //prepare to update screen with actuator message 2 data
				  ActuatorMsg2State = ActuatorMsg2State | 0x01;
				  ActuatorMsg2State = ActuatorMsg2State | 0x02;
				  ActuatorMsg2State = ActuatorMsg2State | 0x40; //flag to main loop to update display
			  }

			  if ((ActuatorPositionState & 0x04) != 0) //test flag set by serial cpommad "AM11"
			  {
				  ActuatorPositionState = ActuatorPositionState | 0x01; //force update of CAN position display
			  }

			  RxState = RxState & 0xEF; //clear flag
		  }
	  }



	  if ((RxState & 0x08) != 0) //test for Escape character
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {

			  //char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[2;1H\e[K\e[1;37;41m"); //move cursor to 2nd line, white text on red background
			  strcpy(tempstring, tmpstr);
			  strcat(tempstring, "ESCAPE");
			  sprintf(tmpstr, "\e[0m"); //reset all attributes
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[1;1H\e[K"); //clear string construction line
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[4;1H\e[K"); //move cursor to 4th line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[5;1H\e[K"); //move cursor to  5th line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[6;1H\e[K"); //move cursor to  6th line, clear text,
			  strcat(tempstring, tmpstr);

			  sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to  7th line, clear text,
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


	  if ((RxState & 0x02) != 0) //check for serial receive buffer overflow
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
			  //Xmodem comms is currently disabled
			  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
			  {

				  //process received serial data as basic string commands
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

					  strcpy(tempstring, "");
//					  if (recognisedstring == FLAG_CLEAR)
//					  {
//						  //clear 'unrecognised string' message
//						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
//						  strcat(tempstring, tmpstr);
//						  recognisedstring = FLAG_SET; //set flag to prevent continuous clearing of screen line with each received new string character
//					  }
					  if (prevcommandstringstate == FLAG_SET) //flag is set as soon as a command string is processed
					  {
						  prevcommandstringstate = FLAG_CLEAR;
						  sprintf(tmpstr, "\e[3;1H\e[K"); //move cursor to 3rd line, clear text,
						  strcat(tempstring, tmpstr);

					  }


					  sprintf(tmpstr, "\e[1;1H\e[K\e[1;33;40m"); //set cursor to line 1, clear existing data, set yellow background
					  strcat(tempstring, tmpstr);
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

			  if (XmodemStatus == 0x04) //initial 'C' has been sent to host so this should be a response packet...
			  {
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


//		 uint8_t ShiftDemand = 0;	//bit 7:set for CAN upshift
//		 	 	 	 	 	 	 	 //bit 6 set for CAN downshift
//		 	 	 	 	 	 	 	 //bit 5: set for logic level upshift
//		 	 	 	 	 	 	 	 //bit 4: set for logic level downshift
//		 uint8_t ShiftState  = 0;
//									 //bit 7:set for CAN upshift
//		 	 	 	 	 	 	 	 //bit 6 set for CAN downshift
//		 	 	 	 	 	 	 	 //bit 5: set for logic level upshift
//		 	 	 	 	 	 	 	 //bit 4: set for logic level downshift
//		 uint16_t ShiftDemandPulse = 100; //set duration of shift demand pulse

//		 uint16_t PreloadPullDemandPulse = 100; //sets duration of preload pull demand pulse
//		 uint16_t PreloadPushActivationtime = 50;
//		 uint16_t PreloadPushDemandPulse = 100; //sets duration of preload push demand pulse

		//uint16_t PreloadPullActivationTime = 50;
		//uint16_t PreloadPullActivationCount = 0;
		//uint16_t PreloadPullDemandPulseTime = 100;
		//uint16_t PreloadPullDemandCount = 0;
		//uint16_t PreloadPushDelayTime = 100;
		//uint16_t PreloadPushActivationTime = 100;
		//uint16_t PreloadPushActivationCount = 0;
		//uint16_t PreloadPushDemandPulseTime = 100;
		//uint16_t PreloadPushDemandCount = 0;

		if (ActuatorMsg1FlashCount != 0)
		{
			ActuatorMsg1FlashCount--;
			if (ActuatorMsg1FlashCount == 0)
			{
				ActuatorPositionState = ActuatorPositionState | 0x01; //flag to main loop  to update display
				ActuatorMsg1FlashCount = ActuatorMsg1FlashTime;
			}
		}

		if (ActuatorMsg2FlashCount != 0)
		{
			ActuatorMsg2FlashCount--;
			if (ActuatorMsg2FlashCount == 0)
			{
				ActuatorMsg2State = ActuatorMsg2State | 0x40; //update display
				ActuatorMsg2FlashCount = ActuatorMsg2FlashTime; //reset flash period
			}
		}


		if (ActuatorMsg2Timeoutcount != 0)
		{
			ActuatorMsg2Timeoutcount--;
			if (ActuatorMsg2Timeoutcount == 0)
			{
				ActuatorMsg2State = ActuatorMsg2State & 0x7F; //reset flag - signal to main loop that actuator 2nd message hasn't been received recently
				ActuatorMsg2State = ActuatorMsg2State | 0x40; //indicate to main loop that message status has changed
				ActuatorMsg2FlashCount = ActuatorMsg2FlashTime; //value decremented by TIM1 ISR
			}
		}



		if (PositionSignalTimeoutCount != 0) //this value is reset by reception of actuator 0x254 CAN message and serial command "AM11"
		{
			PositionSignalTimeoutCount--;
			if (PositionSignalTimeoutCount == 0)
			{
				ActuatorPositionState = ActuatorPositionState & 0xFD; //reset status flag
				ActuatorPositionState = ActuatorPositionState | 0x01; //flag to main loop to update display

				ActuatorMsg1FlashCount = ActuatorMsg1FlashTime; //value decremented by TIM1 ISR
			}
		}

		if ((DedicatedShiftControl & 0x80) != 0)	//see serial command "SCx"
		{
			uint8_t Errval = 0;
			if ((ShiftDemand & 0x0F) == 0x01) //see serial commands "LUP","LDN","CUP", "CDN", "MLUP", "MLDN"
			{

				//test for repeated up-shifts
				uint8_t InitiateShiftDemand = 0;
				if ((ActuatorPositionState & 0x02) != 0) //only enable shift demand if position feedback has been received
				{
					if ((ShiftDemand & 0xA0) != 0) //bits set by serial commands LUP,CUP, MLUP, MCUP,
					{
						if (ActuatorPosition < PositionMaxLimit)
						{
							InitiateShiftDemand = 1;
						}
						else
						{
							Shiftdemandfeedback = Shiftdemandfeedback & 0xF0;
							Shiftdemandfeedback = Shiftdemandfeedback | 0x01; 	//error 1
							//Shiftdemandfeedback = Shiftdemandfeedback | 0x80;
						}

					}
					if ((ShiftDemand & 0x50) != 0) //bits set by serial commands "LDN", "CDN", "MLDN", "MCDN",
					{
						if (ActuatorPosition > PositionMinLimit)
						{
							InitiateShiftDemand = 1;
						}

						else
						{
							Shiftdemandfeedback = Shiftdemandfeedback & 0xF0;
							Shiftdemandfeedback = Shiftdemandfeedback | 0x02; 	//error 2
							//Shiftdemandfeedback = Shiftdemandfeedback | 0x80;
						}
					}
				}


				if ((DedicatedShiftControl & 0x20) == 0) //see serial command "RPC0"
				{
					InitiateShiftDemand = 1;
				}

				if (InitiateShiftDemand == 1)
				{
					ShiftDemandPulseCount = ShiftDemandPulseTime;
					PreloadPullActivationCount = PreloadPullActivationTime + 1; //add offset to ensure each sequence step is executed

					if (Multishift != 0) //see serial commands "MLUP","MLDN"
					{
						Shift2ShiftCount = Shift2ShiftTime; //set time to next shift demand (multiple shift requested!)
					}


					ShiftDemand = ShiftDemand | 0x02; //advance state count
				}

				else
				{
					ShiftDemand = 0;
					Multishift  = 0;

					if ((Shiftdemandfeedback & 0x0F) == 0)
					{
						//Shiftdemandfeedback = Shiftdemandfeedback & 0xF0;
						Shiftdemandfeedback = Shiftdemandfeedback | 0x03; 	//error 3

					}
					Shiftdemandfeedback = Shiftdemandfeedback | 0x80;

				}
			}


			if (ShiftDemandPulseCount != 0)
			{
				//apply valid shift demand
				if ((DedicatedShiftControl & 0x40) != 0) //see serial command "SCCx"
				{
					if ((ShiftDemand & 0xC0) != 0)
					{
						//provide CAN shift demand signal update
						if ((ShiftDemand & 0x80) != 0)
						{
							//apply CAN upshift signal state

							Errval = CanShiftDemand(1);
							if (Errval != 0)
							{
								DedicatedShiftControl = DedicatedShiftControl & 0xBF; //kill CAN shift demands
							}
						}
						if ((ShiftDemand & 0x40) != 0)
						{
							//apply CAN downshift signal state
							//Errval = CanShiftDemand(-1);
							Errval = CanShiftDemand(2);
							if (Errval != 0)
							{
								DedicatedShiftControl = DedicatedShiftControl & 0xBF; //kill CAN shift demands
							}
						}
					}
				}
				if ((ShiftDemand & 0x30) != 0)
				{
					if ((ShiftDemand & 0x20) != 0)
					{
						//apply logic level upshift signal state
						HAL_GPIO_WritePin(GPIOD, HSD_1_Pin, GPIO_PIN_SET);

					}
					if ((ShiftDemand & 0x10) != 0)
					{
						//apply logic level downshift signal state
						HAL_GPIO_WritePin(GPIOD, HSD_2_Pin, GPIO_PIN_SET);
					}
				}

				ShiftDemandPulseCount--;
			}

			else
			{
				//apply CAN & logic level inactive shift demand signal states
				HAL_GPIO_WritePin(GPIOD, HSD_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, HSD_2_Pin, GPIO_PIN_RESET);

				if ((DedicatedShiftControl & 0x40) != 0) //see serial command "SCCx"
				{
					Errval = CanShiftDemand(0); //output inactive shift demand signal
					if (Errval != 0)
					{
						DedicatedShiftControl = DedicatedShiftControl & 0xBF; //kill CAN shift demands
					}
				}
			}



			if (PreloadPullActivationCount != 0)
			{
				PreloadPullActivationCount--;
				if (PreloadPullActivationCount == 0)
				{
					ShiftDemand = ShiftDemand & 0xF0;
					ShiftDemand = ShiftDemand | 0x03; //advance state count
					PreloadPullDemandCount = PreloadPullDemandPulseTime + 1; //ensure non zero value to ensure each sequence step is executed
				}
			}

			if (PreloadPullDemandCount != 0)
			{
				if (PreloadPullDemandPulseTime != 0) //prevent output glitches is time is set to zero
				{
					//activate preload 'pull' signal
					HAL_GPIO_WritePin(GPIOD, HSD_3_Pin, GPIO_PIN_SET); //activate preload 'pull' signal
				}
				PreloadPullDemandCount--;

				if (PreloadPullDemandCount == 0)
				{
					ShiftDemand = ShiftDemand & 0xF0;
					ShiftDemand = ShiftDemand | 0x04; //advance state count
					PreloadPushActivationCount = PreloadPushActivationTime + 1;
				}
			}
			else
			{
				//deactivate preload 'pull' signal
				HAL_GPIO_WritePin(GPIOD, HSD_3_Pin, GPIO_PIN_RESET);
			}

			if (PreloadPushActivationCount != 0)
			{
				PreloadPushActivationCount--;
				if (PreloadPushActivationCount == 0)
				{
					ShiftDemand = ShiftDemand & 0xF0;
					ShiftDemand = ShiftDemand | 0x05; //advance state count
					PreloadPushDemandCount = PreloadPushDemandPulseTime + 1;
				}
			}

			if (PreloadPushDemandCount != 0)
			{
				//activate preload 'push' signal
				if (PreloadPushDemandPulseTime != 0)
				{
					HAL_GPIO_WritePin(GPIOD, HSD_4_Pin, GPIO_PIN_SET); //activate preload push signal
				}
				PreloadPushDemandCount--;
				if (PreloadPushDemandCount == 0)
				{
					ShiftDemand = ShiftDemand & 0xF0;
					ShiftDemand = ShiftDemand | 0x06; //advance state count

				}
			}
			else
			{
				//deactivate preload 'push' signal
				HAL_GPIO_WritePin(GPIOD, HSD_4_Pin, GPIO_PIN_RESET);

				if ((ShiftDemand & 0x0F) == 0x06)
				{
					//shift demand completed
					if (Multishift != 0)
					{
						ShiftDemand = ShiftDemand & 0xF0; //clear progress counter bits

						if ((ActuatorPositionState & 0x02) == 0)
						{
							if (ShiftDemandCount < 8)
							{
								//ShiftDemand = ShiftDemand | 0x07; //advance state count
								ShiftDemandCount++;
							}
							else
							{
								Multishift = 0; //disable multiple shifting
							}
						}

					}
					else
					{
						ShiftDemand = ShiftDemand & 0xF0;

						Shiftdemandfeedback = Shiftdemandfeedback & 0xF0; //error 0
						Shiftdemandfeedback = Shiftdemandfeedback | 0x80;
					}
				}


			}

			if (Multishift != 0) //see serial commands "MLUP", "MLDN"
			{
				if (Shift2ShiftCount != 0)
				{
					Shift2ShiftCount--;
					if (Shift2ShiftCount == 0)
					{
						ShiftDemand = ShiftDemand | 0x01; //prepare to initiate a new shift demand
					}
				}
			}

		}



		if (CanAnalogScanState != 0)
		{

			ScanUpdateTime--;
			if (ScanUpdateTime == 0)
			{
				ScanUpdateTime = ScanUpdatetimeRefreshValue; //update step time
				CanAnalogScanState = CanAnalogScanState | 0x02; //flag to main loop
				UpdateScreen = 1;

			}
			ScanValue++;

		}

		if (GetSequencerState() != 0) //check to see is sequencer has been enabled
		{
			//Sequencer function is enabled
			SeqStepTime--;
			if (SeqStepTime == 0)
			{
				//indicate to main loop that new sequence step is to be started
				SetSequencerState(NEXTSTEPSETUP);
			}
		}

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

		DecrementI2cTiming(); //Used as timeout period for I2C comms

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

		CanErrorValue = HAL_CAN_GetError(hcan);


	}
}


void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	//Created 8OCT2025
	// call back function - called from stm32l4xx_hal_can.c
	//FIFO 0 is full interrupt handler
	if (hcan->Instance == CAN1)
	{
		CanRxFifoFull = 1;
	}
}


//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//CAN message received
	//Last edited 8OCT2025
	if (hcan->Instance == CAN1)
	{
		uint32_t qty = 0;
		qty = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
		if (qty != 0)
		{

			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, pCanRxHeader, CanRxData);

			if (CanDataReceived == FLAG_CLEAR)
			{

				//main loop is ready to process a new message
				TempCanRxHeader = CanRxHeader;

				uint8_t* srcptr = NULL;
				uint8_t* dstptr = NULL;
				srcptr = &CanRxData[0];
				dstptr = &TempCanRxData[0];
				for (uint8_t i=0; i<8; i++)
				{
					*dstptr++ = *srcptr++;
				}
				//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, pCanRxHeader, CanRxData);
				recmsgindex = reccount;
				CanDataReceived = FLAG_SET;
			}


			if (pCanRxHeader->StdId == 0x254)
			{
				//keep track of actuator output shaft position
				ActuatorPosition = (CanRxData[4] & 0x03) << 8;
				ActuatorPosition = ActuatorPosition | CanRxData[3];

				if ((ActuatorPositionState & 0x02) == 0)
				{
					ActuatorPositionState = ActuatorPositionState | 0x02; 	//record that an actuator message has been received, this will be cleared if timeout period expires
					ActuatorPositionState = ActuatorPositionState | 0x01; 	//flag to main loop to update displayed position

					ActuatorPositionState = ActuatorPositionState & 0xF7; 	//reset flash state flag
					ActuatorMsg1FlashCount = 0;								//prevent message flashing
				}

				PositionSignalTimeoutCount = PositionSignalTimeoutPeriod; 	//reset timeout period, value decremented by TIM1 ISR
				if (ActuatorPosition != PrevActuatorPosition)
				{
					PrevActuatorPosition = ActuatorPosition;

					ActuatorPositionState = ActuatorPositionState | 0x01; 	//flag to main loop to update displayed position
				}
			}

			if (pCanRxHeader->StdId == 0x354)
			{
				//obtain actuator temperatures
				ActuatorMotorTemp = CanRxData[0];
				ActuatorPcbTemp = CanRxData[1];
				if ((ActuatorMsg2State & 0x80) == 0)
				{
					ActuatorMsg2State = ActuatorMsg2State | 0x80; 	//indicate to main loop that the 2nd actuator message has been received, this will be cleared if timeout period expires
					ActuatorMsg2State = ActuatorMsg2State | 0x01;	//flag to main loop that value has changed - force update
					ActuatorMsg2State = ActuatorMsg2State | 0x02;	//flag to main loop that value has changed - force update
					ActuatorMsg2State = ActuatorMsg2State | 0x40; 	//prepare to update display

					ActuatorMsg2State = ActuatorMsg2State & 0xEF;	//reset flash state flag
					ActuatorMsg2FlashCount = 0;						//prevent message flashing
				}

				ActuatorMsg2Timeoutcount = ActuatorMsg2TimeoutPeriod; //reset message timeout period, value decremented by TIM 1 ISR
				if (ActuatorMotorTemp != PrevActuatorMotorTemp)
				{
					PrevActuatorMotorTemp = ActuatorMotorTemp;
					ActuatorMsg2State = ActuatorMsg2State | 0x01;	//flag to main loop that value has changed
					ActuatorMsg2State = ActuatorMsg2State | 0x40; 	//prepare to update display
				}

				if (ActuatorPcbTemp != PrevActuatorPcbTemp)
				{
					PrevActuatorPcbTemp = ActuatorPcbTemp;
					ActuatorMsg2State = ActuatorMsg2State | 0x02;	//flag to main loop that value has changed
					ActuatorMsg2State = ActuatorMsg2State | 0x40; 	//prepare to update display
				}
			}
			reccount++;

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
	//Last edited 29SEP2025
	if (huart->Instance == USART3)
	//if (huart->Instance == USART1)
	{
		if (XmodemStatus == 0)
		{
			//copy characters from primary to secondary buffers (Xmodem comms disabled)
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
//			}
		}

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
