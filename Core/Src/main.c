/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define RXBUFFERLENGTH 20
#define ESCAPEDISPLAYPERIOD 500

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
volatile char RxBuffer2[20] = "";
//volatile static uint8_t UartRxData = FLAG_SET;
volatile static uint8_t RxState = 0; //UART3 state bits:
										//0: character arrived
										//1: buffer overflow
										//2: new string
										//3: escape character detected
										//4: clear escape character message from VT100 screeen
										//5: set when received struing is to be processed

volatile static uint8_t RxWritePtr = 0;
volatile static uint8_t RxBufferCount = 0;

volatile static uint16_t EscapeClearCount = 0;

volatile static uint16_t FunctionDelay = 0; //value decremented by TIM ISR

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
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t mainloopcount = 0;
	uint16_t DacVal = 0;
	uint16_t DacVal2 = 0xFFF;
	uint8_t index = 0;
	//uint8_t rxstringlength = 0;
	char RxString[20] = "";

	uint8_t RxStringLen = 0;
	uint8_t RxReadPtr = 0;

	char tmpstr[100] = "";
	char tempstring[200] = "";


	uint16_t CanIdentifier = 0;
	uint8_t CanDlc = 0;
	uint8_t CanData[8] = {0};

	uint8_t CanTxMsgCount = 0; //generic CAN TX message counter
	uint8_t CanTxMailboxfullmsg = FLAG_CLEAR;  //flag set when CAN tx mailbox is full, this prevents repeated output
	uint8_t candatacount = 0;

	uint8_t ProcessRececivedCanData = FLAG_CLEAR;

	uint8_t I2cInitialisationFunction = 0;
	uint8_t I2cReadBlockFunction = 0;

	uint8_t recognisedstring = FLAG_CLEAR;

	uint32_t UserVal = 0;
	uint8_t screenblock = FLAG_CLEAR;

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
	  if (I2cReadBlockFunction != 0)
	  {
		  //result of "I2CR" serial command; outputs block of I2c data to the display
		  //use setup commands beforehand:
		  //I2CDxx
		  //I2cAxxxx
		  //KI2cQxxx
		  //I2CAx
		  switch(I2cReadBlockFunction)
		  {
		  	  case(1):
				//initialisation
				//display pointers
				if (UartMsgSent == FLAG_CLEAR) //flag cleared by UART TX complete ISR
				{
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
					uint8_t* byteptr;
					byteptr = ReadSmallI2CDatablock4();
					if (*byteptr == 0)
					{
						sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "I2C data block read - now to display...");
						strcat(tempstring, tmpstr);
					}
					else
					{
						sprintf(tmpstr, "\e[7;1H\e[K"); //move cursor to 3rd line, clear text,
						strcpy(tempstring, tmpstr);
						sprintf(tmpstr, "I2C data block read reports error: 0x%02X", *byteptr);
						strcat(tempstring, tmpstr);
					}
					uint16_t stringlength = strlen(tempstring);

					//HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
					HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
					UartMsgSent = FLAG_SET;
					FunctionDelay = 1000; //value decremented by TIM ISR
					I2cReadBlockFunction = 4;
				}
				break;

			case(4):
				I2cReadBlockFunction = 0; //Disable this main loop function
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
			  char tmpstr[50] = "";
			  recognisedstring = FLAG_CLEAR;

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
						strncpy(valstring, RxString[4], 2); //obtain value characters

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
								sprintf(tmpstr, "There was a problem reading a byte from I2C memory!");
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
			  char tmpstr[20] = "";
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

			  char tmpstr[20] = "";
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


			  char tmpstr[20] = "";
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


	  if ((RxState & 0x02) != 0) //check for reception of a new character
	  {
		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {
			  //indicate buffer overflow
			  char tmpstr[20] = "";
			  sprintf(tmpstr, "\e[1;1H\e[K\e[1;37;41m"); //move cursor to 1st line, clear and text, then prepare white text on red background
			  strcpy(tempstring, tmpstr);
			  strcat(tempstring, "Buffer overflow!");
			  sprintf(tmpstr, "\e[0m"); //reset all attributes
			  strcat(tempstring, tmpstr);


			  uint16_t stringlength = strlen(tempstring);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tempstring, stringlength); //FTDI USB interface
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *) tempstring, stringlength); //RS485 port
			  UartMsgSent = FLAG_SET;

			  RxState = RxState & 0xFD; //clear bit.
		  }
	  }


	  if ((RxState & 0x01) != 0) //check for reception of a new character
	  {

		  if (UartMsgSent == FLAG_CLEAR) //check previous serial data has been sent
		  {

			  //processed received serial data.
			  RxState = RxState & 0xFE; //clear bit

			  char tmpstr[20]="";
			  uint8_t charindex = RxReadPtr;
			  RxStringLen = 0;
			  RxString[0] = 0; //terminate string

			  if (RxBufferCount != 0)
			  {
			  for (uint8_t i=0; i<RxBufferCount; i++)
				  {

					  if (RxBuffer2[charindex] == 0x0d)
					  {
						  if (RxStringLen != 0)
						  {

							  RxState = RxState | 0x04;
							  RxBufferCount = RxBufferCount - (RxStringLen + 1);
						  }
						  RxReadPtr = charindex+1;
						  break;
					  }
					  else
					  {
						  RxString[i] = RxBuffer2[charindex];
						  RxStringLen++;
						  RxString[i+1] = 0; //terminate string
					  }
					  charindex++;
					  if (charindex > RXBUFFERLENGTH)
					  {
						  charindex = 0;
					  }

				  }

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


			  //prepare to receive more data
			  //HAL_UART_Receive_IT (&huart1, (uint8_t *) RxBuffer1, 2); //this enables UART reception interrupt
			  //HAL_UART_Receive_IT (&huart3, (uint8_t *) RxBuffer1, 2); //this enables UART reception interrupt

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
		static uint16_t timer1count = TIMER1PERIOD;
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
				HAL_GPIO_TogglePin(GPIOC, MODE1_LED_Pin);

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
	if (huart->Instance == USART3)
	//if (huart->Instance == USART1)
	{
		//copy characters from primary to secondary buffers
		if (RxBuffer1[0] == 0x1B)
		{
			RxState = RxState | 0x08; //indicate to main loop that escape character has been detected
		}


		else if (RxBufferCount < RXBUFFERLENGTH)
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
				RxBuffer2[RxWritePtr] = 0; //terminate string
				if (RxWritePtr > RXBUFFERLENGTH) //test for wrap around
				{
					RxWritePtr = 0;
				}
				RxBufferCount++;
			}
			RxState = RxState | 0x01; //indicate new character arrival

		}
		else
		{
			RxState = RxState | 0x02; //indicate rx buffer overflow
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
