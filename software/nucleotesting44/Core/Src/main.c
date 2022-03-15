/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32767)
#define TEST_LENGTH_SAMPLES ((uint32_t) 2048)
#define BUF_SIZE 4096
#define UINT16_OFFSET 32768
#define SHORT_MAX 32767
#define FFT_SIZE 2048
#define SAMPLERATE 312500

#define SNR_THRESHOLD_F32    75.0f
#define BLOCK_SIZE            64
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
/* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE              32
#else
#define NUM_TAPS_ARRAY_SIZE              63
#endif
#define NUM_TAPS              63

#include "math.h"
#include "usbd_cdc_if.h"
#include "complex.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "../../Drivers/gnu-ais/callbacks.h"
#include "../../Drivers/gnu-ais/filter.h"
#include "../../Drivers/gnu-ais/hmalloc.h"
#include "../../Drivers/gnu-ais/protodec.h"
#include "../../Drivers/gnu-ais/receiver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x300400c0
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x300400c0))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
ALIGN_32BYTES (static uint16_t   aADC1ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);
ALIGN_32BYTES (static uint16_t   aADC3ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);
ALIGN_32BYTES (static uint16_t 	 adc1_data[ADC_CONVERTED_DATA_BUFFER_SIZE]); //I signal
ALIGN_32BYTES (static uint16_t 	 adc3_data[ADC_CONVERTED_DATA_BUFFER_SIZE]); //Q signal

static struct receiver *rx_a = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char *data = "Hello world\n"; //a must have

char buffer[64];

static float32_t testOutput[FFT_SIZE*2];

uint32_t fftSize = FFT_SIZE;
uint32_t ifftFlag = 0;		// regular FFT
uint32_t doBitReverse = 1;  // reverse bit to output.

uint32_t refIndex = 213, testIndex = 0;

//extern uint16_t mysine[1500]; //sine signal for testing

extern uint32_t myAISIdac[1500]; //AIS I signal for dac channel 1
extern uint32_t myAISQdac[1500]; //AIS Q signal for dac channel 2

/* extern uint16_t myAISI[1290];	//AIS signal for testing
   extern uint16_t myAISQ[1290];	//AIS signal for testing
*/
float64_t temp_I = 0;
float64_t temp_Q = 0;

static complex complex_data;
static complex prev_complex;
static complex temp_complex;
static int16_t demodulated_IQ[2048];

uint32_t prim;
float32_t maxValue;



uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = 0;

float32_t  snr;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//arm_status status;
	//status = ARM_MATH_SUCCESS;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // start ADCs. Saves data to aADCxConvertedData


  uint16_t startflag;
  uint16_t endflag;
  uint16_t startflag1;
  uint16_t endflag1;
  uint16_t min_val;
  uint16_t max_val;

  rx_a = init_receiver('A', 1, 0, 0);	// PLL
  char stringBuf[100];
  float32_t data_Buffer[FFT_SIZE*2] = {0};



	  if (HAL_ADC_Start_DMA(&hadc1,
	                          (uint32_t *)aADC1ConvertedData,
	                          ADC_CONVERTED_DATA_BUFFER_SIZE
	                         ) != HAL_OK)
	    {
	      Error_Handler(); //does nothing-> TODO: error handler
	    }
	  if (HAL_ADC_Start_DMA(&hadc3,
	                        (uint32_t *)aADC3ConvertedData,
	                        ADC_CONVERTED_DATA_BUFFER_SIZE
	                       ) != HAL_OK)
	  {
	    Error_Handler(); //does nothing-> TODO: error handler
	  }


	  HAL_TIM_Base_Start(&htim1);

	  // start DACs. Takes 12b data, does not like 16b :( .


	  if (HAL_DAC_Start_DMA(&hdac1,
			  	  	    DAC_CHANNEL_2,
						    //(uint32_t *)aADCxConvertedData,
							//ADC_CONVERTED_DATA_BUFFER_SIZE/2,
							(uint32_t *)myAISQdac,
							1500,
							DAC_ALIGN_12B_R)
			  	  	    != HAL_OK)
	  {
		  Error_Handler();
	  }
	  if (HAL_DAC_Start_DMA(&hdac1,
			  	  	    DAC_CHANNEL_1,
						    //(uint32_t *)aADCxConvertedData,
							//ADC_CONVERTED_DATA_BUFFER_SIZE/2,
							(uint32_t *)myAISIdac,
							1500,
							DAC_ALIGN_12B_R)
			  	  	    != HAL_OK)
	  {
		  Error_Handler();
	  }

	  HAL_TIM_Base_Start(&htim2);
	  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  memset(stringBuf, '\0', 100);
	  memset(data_Buffer, 0, FFT_SIZE*sizeof(float32_t)*2);
	  memset(testOutput, 0, FFT_SIZE*sizeof(float32_t)*2);
	  prev_complex = 0;
	  complex_data = 0;
	  temp_complex = 0;
	  temp_I = 0;
	  temp_Q = 0;
	  startflag = 0;
	  endflag   = 0;
	  startflag1 = 0;
	  endflag1 = 0;
	  min_val = 0;
	  max_val = 0xffff;


	  // memcpy(adc1_data, aADC1ConvertedData, 2048*4);
	  // memcpy(adc3_data, aADC3ConvertedData, 2048*4);

	  prim = __get_PRIMASK();
	  __disable_irq();


	  // finds signal start and end, TODO: make more universal
	  for(uint16_t i = 1; i<ADC_CONVERTED_DATA_BUFFER_SIZE; i++){

		  if(adc1_data[i]> max_val){
			  max_val = adc1_data[i];
		  }else if(adc1_data[i]<min_val){
			  min_val = adc1_data[i];
		  }

		  if(startflag1 == 0 && endflag1 == 0){
			  if(adc3_data[i]<UINT16_OFFSET+400 && adc3_data[i]>UINT16_OFFSET-400){
				  startflag++;
			  }else if(startflag>5){
				  startflag1 = i-100; //startflag finalized here
				  endflag1 = 0;
				  endflag = 0;
			  }else{
				  startflag = 0;
			  }
		  }
		  if(startflag1 != 0 && endflag1 == 0){
		  	  if(endflag >5){
				  endflag1 = i-6; //endflag finalized here
			  }else if(adc3_data[i]<UINT16_OFFSET+400 && adc3_data[i]>UINT16_OFFSET-400){
				  endflag++;
			  }else{
				  endflag = 0;
			  }
		  }
		  if(endflag1-startflag1 >= 2048){
			  endflag1 = 0;
			  startflag1 = 0;
		  }
	  }
	  int j = 0;
	  for(int i = startflag1; i < endflag1; i++){

		  data_Buffer[j] = (float32_t)(adc1_data[i] - UINT16_OFFSET )/(UINT16_OFFSET);
		  j++;
		  data_Buffer[j] = (float32_t)(adc3_data[i] - UINT16_OFFSET )/(UINT16_OFFSET);
		  j++;

	  }
	  j = 0;

	  for(uint16_t i = 1; i<ADC_CONVERTED_DATA_BUFFER_SIZE; i++){
		  arm_cfft_f32(&arm_cfft_sR_f32_len2048, data_Buffer, ifftFlag, doBitReverse);
		  arm_cmplx_mag_f32(data_Buffer, testOutput, FFT_SIZE);
		  arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
	  }
	  // demodulate signal, saved to demodulated_IQ
	  gmsk_demod(startflag1, endflag1, demodulated_IQ);

	  demodulated_IQ[0]=0;

	  receiver_run(rx_a, demodulated_IQ, endflag1-startflag1);

	  j = 0;

	  // from example arm_fft_bin.example.c:
	  /* Process the data through the CFFT/CIFFT module */
	  arm_cfft_f32(&arm_cfft_sR_f32_len2048, data_Buffer, ifftFlag, doBitReverse);
	  arm_cmplx_mag_f32(data_Buffer, testOutput, FFT_SIZE);
	  arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

	  //arm_rfft_fast_f32(&instance, data_Buffer, testOutput, 0);

	  /* Process the data through the Complex Magnitude Module for
		 calculating the magnitude at each bin */
	  //arm_cmplx_mag_f32(data_Buffer, testOutput, fftSize);

	  // MATLAB compatible data transfer. Transfer as characters, MATLAB reads as correct data length.:
	  data = "flag data:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
	  sprintf(stringBuf, "%d\n", startflag1 );
	  HAL_UART_Transmit(&huart3,  (uint8_t *)stringBuf, strlen(stringBuf), HAL_MAX_DELAY);
	  sprintf(stringBuf, "%d\n", endflag1 );
	  HAL_UART_Transmit(&huart3,  (uint8_t *)stringBuf, strlen(stringBuf), HAL_MAX_DELAY);
	  data = "flag data end:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);



	  data = "demod data:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  //HAL_UART_Transmit(&huart3, (uint8_t *) demodulated_IQ,
	  	//		  	  1290*2, HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart3, (uint8_t *) demodulated_IQ,
			  	  (endflag1-startflag1)*2, HAL_MAX_DELAY);

	  data = "demod data end:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  //transmit NMEA
	  sprintf(stringBuf, "rx_a PLL decoded data in NMEA format: %s \n", rx_a->decoder->nmea );
	  HAL_UART_Transmit(&huart3, (uint8_t *)stringBuf, strlen(stringBuf), HAL_MAX_DELAY);

	  //transmit FFT
	  data = "FFT data:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart3, (uint8_t *) testOutput,
			  	  	  	  FFT_SIZE*4, HAL_MAX_DELAY);

	  data = "FFT data end.\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  //Transfer ADC data:

	  data = "ADC1 data:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart3, (uint8_t *) adc1_data+(startflag1*2),
			  (endflag1-startflag1)*2, HAL_MAX_DELAY);
	  data = "ADC1 data end.\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  data = "ADC3 data:\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart3, (uint8_t *) adc3_data+(startflag1*2),
			  (endflag1-startflag1)*2, HAL_MAX_DELAY);
	  data = "ADC3 data end.\n";
	  HAL_UART_Transmit(&huart3, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	  //delay because why not
	  //HAL_Delay(1000);
	  __enable_irq();

	  memset(demodulated_IQ, 0, 2048*sizeof(int16_t));
	  memset(rx_a->decoder->nmea, '\0', NMEABUFFER_LEN);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  // f = 50 MHz
  // Samplerate = f/(ClockPrescaler*(SamplingTime+Resolution/2+0.5))/2
  // atm samplerate = 312500Hz, it good resolution upto +-150 kHz
  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  // tim clock 240 MHz
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 770;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 770;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	prim = __get_PRIMASK();
	__disable_irq();
	if(hadc == &hadc1){
		memcpy(adc1_data+ADC_CONVERTED_DATA_BUFFER_SIZE/2, aADC1ConvertedData+ADC_CONVERTED_DATA_BUFFER_SIZE/2, ADC_CONVERTED_DATA_BUFFER_SIZE);
	    memcpy(adc3_data+ADC_CONVERTED_DATA_BUFFER_SIZE/2, aADC3ConvertedData+ADC_CONVERTED_DATA_BUFFER_SIZE/2, ADC_CONVERTED_DATA_BUFFER_SIZE);
	    //memcpy(adc1_data, aADC1ConvertedData, 2048*4);
	    //memcpy(adc3_data, aADC3ConvertedData, 2048*4);


	}
	if(!prim){
		__enable_irq();
	}

}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	prim = __get_PRIMASK();
	  if(hadc == &hadc1){
		  memcpy(adc1_data, aADC1ConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
		  memcpy(adc3_data, aADC3ConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
	  }
		if(!prim){
			__enable_irq();
		}
}
/* Demodulates gmsk modulated signal that has been downmixed
 * startflag = signal start in adcx_data
 * signalend = singal end in adcx_data
 * demodulated_IQ = array where demodulated signal is stored.
 * */
void gmsk_demod(int startflag, int endflag, int16_t *demodulated_IQ){

	for(int i = startflag; i < endflag; i++){

		/*temp_I = -(float64_t)(adc1_data[i] - (max_val-min_val)/2) / ((max_val-min_val)/2);// cast uint16 value to float64_t
		temp_Q = -(float64_t)(adc3_data[i] - (max_val-min_val)/2) / ((max_val-min_val)/2);*/

		temp_I = -(float64_t)(adc1_data[i] - (UINT16_OFFSET)) / (UINT16_OFFSET);// cast uint16 value to float64_t
		temp_Q = -(float64_t)(adc3_data[i] - (UINT16_OFFSET)) / (UINT16_OFFSET);

		complex_data = (temp_I + (temp_Q * _Complex_I));
		temp_complex = complex_data * conj(prev_complex); // polar discriminator
		demodulated_IQ[i-startflag] = (int16_t)(SHORT_MAX * (atan2(cimag(temp_complex), creal(temp_complex)) / M_PI));
		prev_complex = complex_data;

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

