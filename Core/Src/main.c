/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
#include <stdint.h>
#include "Tweezer_phase_Lookup.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFF_SIZE 512
#define SAMPLE_BUFF_SIZE 128 //2 periods
#define BIAS_BUFF_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim6_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// random data for test
int D = 0;
// base display data array
uint16_t Display[4] = {0,0,0,0};
// array for individual values
int data[4];
// lookup table for data
uint16_t X0[4] = {1,3,2,3};
uint16_t X1[4] = {0,2,2,0};
uint16_t X2[4] = {1,1,3,2};
uint16_t X3[4] = {1,2,3,2};
uint16_t X4[4] = {0,2,3,1};
uint16_t X5[4] = {1,2,1,3};
uint16_t X6[4] = {1,3,1,3};
uint16_t X7[4] = {0,2,2,2};
uint16_t X8[4] = {1,3,3,3};
uint16_t X9[4] = {1,2,3,3};
//DISPLAYYYYYYYY ^^^^^^^^^^
static uint16_t ODR_Buff[TX_BUFF_SIZE];
static uint16_t samples[SAMPLE_BUFF_SIZE] = {0};
static uint16_t processBuff[SAMPLE_BUFF_SIZE/2] = {0};

static double cosAngLut[32];
static double sinAngLut[32];

static double even[32];
static double odd[32];
static double demol[32];

static double k = 1; //Frequency bin of interest 1hz

static double real = 0; //Real part of sensor signal
static double imag = 0; //Imaginary part of sensor signal

static double magnitude = 0; //Amplitude of sensor signal
static double phaseRad = 0;  //Phase in radians
static double phaseDeg = 0;  //Phase in degrees
static double phaseDegDisp = 0;

static int quadrant = 1;

uint8_t dataRdyFlag = 0;
float logging[1024] = {0};
int timing[1024] = {0};
int logcount = 0;
float offset = 310.3122;
float tempf;
int tempi;
//uint16_t data[2] = {0xFFFF, 0x0000};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// configures data into an array of 4 individual values
void configure(int* ptr, int D){
      //int dec = D*100;
		int dec = D;
      ptr += 3;
      for(int i = 0; i < 4; i++){
            int x = pow(10,i);
            *ptr = (dec/x)%10;
            ptr--;
      }
}
void DisplayUpdate(uint16_t* displayptr, int* dataptr){
      int LS = 3;
      uint16_t *originaldisplayptr = displayptr;

      for(int j = 2; j < 6; j++){
            displayptr = originaldisplayptr;
            switch (*dataptr){
                  case 0:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X0[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 1:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X1[i] << LS);
                              printf("LS = %i\n", X1[i] << LS);
                              printf("display ptr = %i\n", *displayptr);
                              displayptr++;
                        }
                        break;
                  case 2:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X2[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 3:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X3[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 4:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X4[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 5:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X5[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 6:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X6[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 7:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X7[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 8:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X8[i] << LS);
                              displayptr++;
                        }
                        break;
                  case 9:
                        for(int i = 0; i < sizeof(data)/sizeof(data[0]); i++){
                              *displayptr = Display[i] ^ (X9[i] << LS);
                              displayptr++;
                        }
                        break;
            }
            LS =  LS + j;
            dataptr++;
      }
}
//DISPLAYYYYYY^^^^^^^
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}
void generateLuts()
{
    for (int i = 0; i < 32; i++)
    {
        double angle = (2 * M_PI * (double)i * k) / 32;

        cosAngLut[i] = cos(angle);
        sinAngLut[i] = sin(angle);
    }
}
uint32_t calcBuffLoc(uint32_t startPos, uint32_t offSet, uint32_t buffSize)
{
    uint32_t newLoc = startPos + offSet;

    if (newLoc < buffSize)
    {
        return newLoc;
    }
    else
    {
        return newLoc - buffSize; // Loop Around
    }
}
float constrainAngle(float x){
	x= fmod(x,360);
	if (x<0){
		x += 360;
	}
	return x;
}
void generate_ODR_Buff()
{
	const uint8_t singlePhaseBuff[TX_BUFF_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
	//const uint8_t biasBuff[BIAS_BUFF_SIZE] = {0, 0, 0, 1, 1, 1, 1, 1};//ORIGINAL
	//const uint8_t biasBuff[BIAS_BUFF_SIZE] = {0, 0, 0, 0, 0, 0, 0, 1};//shorter active period
	//const uint8_t biasBuff[BIAS_BUFF_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};//NO BIAS TEST
	const uint8_t biasBuff[BIAS_BUFF_SIZE] = {1, 1, 1, 0, 0, 0, 0, 0};// FOR TESTING ON BREADBOARD

	uint32_t tx0Loc, tx1Loc, tx2Loc, tx3Loc, tx4Loc, tx5Loc, tx6Loc, tx7Loc = 0; // 0 ... 512-1: TX buffer locations
	uint32_t biasLoc = 0;                                                        // 0 ... 8-1: Bias buffer location
	uint32_t period = 0;                                                         // Period signal current output pin state
	uint32_t timeBase = 0;                                                       // Timebase signal current output pin state
	uint32_t adcTrig = 0;                                                        // ADC external trigger signal current output pin state
	uint32_t ODRVal = 0;                                                        // Output data register state for all signals combined

	for (uint32_t i = 0; i < TX_BUFF_SIZE; i++)
	{
		tx0Loc = i;
		/* Calculate other TX signals LUT location 45 degrees phase shifted */
		tx1Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 1, TX_BUFF_SIZE);
		tx2Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 2, TX_BUFF_SIZE);
		tx3Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 3, TX_BUFF_SIZE);
		tx4Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 4, TX_BUFF_SIZE);
		tx5Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 5, TX_BUFF_SIZE);
		tx6Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 6, TX_BUFF_SIZE);
		tx7Loc = calcBuffLoc(tx0Loc, TX_BUFF_SIZE / 8 * 7, TX_BUFF_SIZE);

		/* calculate debug signals */
		period = (tx0Loc < TX_BUFF_SIZE / 2) ? 0 : 1;
		timeBase = (tx0Loc % 2) ? 0 : 1; //flips every sample, timebase for debugging

		/* Calculate ADC external trigger signal */
		adcTrig = ((tx0Loc + 7) % 8) ? 0 : 1; //ADC trigger signal every 8T, Offset by 6 to align ADC trigger (rising edge) on second period when the bias is low.
		//adcTrig = ((tx0Loc) % 8) ? 0 : 1;
		/* building the OCTL value */
		ODRVal = 0; //Set all pins low default

		if (singlePhaseBuff[tx0Loc])
			ODRVal |= PWM0_Pin;
		if (singlePhaseBuff[tx1Loc])
			ODRVal |= PWM45_Pin;
		if (singlePhaseBuff[tx2Loc])
			ODRVal |= PWM90_Pin;
		if (singlePhaseBuff[tx3Loc])
			ODRVal |= PWM135_Pin;
		if (singlePhaseBuff[tx4Loc])
			ODRVal |= PWM180_Pin;
		if (singlePhaseBuff[tx5Loc])
			ODRVal |= PWM225_Pin; //STM32l433RCTxP does not have Port C Pin 5
		if (singlePhaseBuff[tx6Loc])
			ODRVal |= PWM270_Pin;
		if (singlePhaseBuff[tx7Loc])
			ODRVal |= PWM315_Pin;
		if (biasBuff[biasLoc])
			ODRVal |= BIAS_Pin; // Bias signal: 8T Period 3T LOW 5T HIGH
		if (adcTrig)
			ODRVal |= ADC_TRIG_OUT_Pin; // ADC external trigger signal: 8T Period
		if (timeBase)
			ODRVal |= TIMEBASE_Pin;     // DBG signal: TimeBase smallest period 1T
		if (period)
			ODRVal |= PERIOD_Pin; // DBG signal: Period is the modulated Sine period. 512/2=256, 256T HIGH followed by 256T LOW

		/* Store OCTL value in buffer */
		ODR_Buff[i] = ODRVal;

		/* Loop over bias signal template buffer when end is reached*/
		biasLoc++;
		if (biasLoc >= BIAS_BUFF_SIZE)
			biasLoc = 0;
	}
}
void sensor_signalProcessing()
{
    /*Split even and odd samples */
    int eLoc = 0;
    int oLoc = 0;
    for (uint32_t i = 0; i < 64; i++)
    {
        if (i % 2)
        {
            even[eLoc] = (double)processBuff[i];
            eLoc++;
        }
        else
        {
            odd[oLoc] = (double)processBuff[i];
            oLoc++;
        }
    }

    /* demodulate: convert sampled signal to sine*/
    for (uint32_t i = 0; i < 32; i++)
    {
        demol[i] = even[i] - odd[i];
    }

    //Demol: LP filter test 4 sample avg
    /*for (uint32_t i = 0; i < 32; i++)
    {
        demolLp[i] = (demol[i] + demol[calcBuffLoc(i, 1, 32)] + demol[calcBuffLoc(i, 2, 32)] + demol[calcBuffLoc(i, 3, 32)]) / 4;
    }*/

    //Demol: LP filter test 8 sample avg
    // for (int i = 0; i < 32; i++)
    // {
    //     demolLp[i] = (demol[i] + demol[calcBuffLoc(i, 1, 32)] + demol[calcBuffLoc(i, 2, 32)] + demol[calcBuffLoc(i, 3, 32)] + demol[calcBuffLoc(i, 4, 32)] + demol[calcBuffLoc(i, 5, 32)] + demol[calcBuffLoc(i, 6, 32)] + demol[calcBuffLoc(i, 7, 32)]) / 8;
    // }

    /// calculate DFT for single frequency bin @ 1Hz
    /// The sampling frequency in Hz = 64 hz
    ///
    /// Bin frequency = k*SamplerateHz/SampleSize = 1*64/64 = 1Hz
    // double sumrealTemp = 0;
    // double sumimagTemp = 0;
    // for (int i = 0; i < 32; i++)
    // {
    //     double angle = (2 * M_PI * (double)i * k) / 32;

    //     // sumrealTemp += (double)demol[i] * cos(angle);
    //     // sumimagTemp += -(double)demol[i] * sin(angle);

    //     sumrealTemp += (double)demolLp[i] * cos(angle);
    //     sumimagTemp += -(double)demolLp[i] * sin(angle);
    // }

    /* Fast dft using precomputed lookup tables for cos and sine part*/
    double sumrealTemp = 0;
    double sumimagTemp = 0;

    for (uint32_t i = 0; i < 32; i++)
    {
        sumrealTemp += (double)demol[i] * cosAngLut[i];
        sumimagTemp += -(double)demol[i] * sinAngLut[i];
    }

    // dft coefficients at the frequency bin of interest (1hz) in complex form
    real = sumrealTemp; //X-axis
    imag = sumimagTemp; //Y-axis

    // Calculate phase
    if (real >= 0 && imag >= 0)
    {
        //1-Quadrant (top right) [+, +]
        phaseRad = atan2(imag, real);
        quadrant = 1;
    }
    else if (real < 0 && imag >= 0)
    {
        //2-Quadrant (top left) [-, +]
        phaseRad = atan2(imag, real);
        quadrant = 2;
    }
    else if (real < 0 && imag < 0)
    {
        //3-Quadrant (bottom left) [-, -]
        phaseRad = 2 * M_PI + atan2(imag, real);
        quadrant = 3;
    }
    else if (real >= 0 && imag < 0)
    {
        //4-Quadrant (bottom right) [+, -] #Optimization: do not check last quadrant just use else
        phaseRad = 2 * M_PI + atan2(imag, real);
        quadrant = 4;
    }

    phaseDeg = (phaseRad * 180) / M_PI; // 0 to 360 degrees
    phaseDegDisp = round((phaseRad * 180) / M_PI);
    if(logcount<1024){
    	logging[logcount] = phaseDeg;
    	timing[logcount] = HAL_GetTick();
    	logcount++;
    }
    else{
    	logcount = 1025;
    }
    // Calculate magnitude
    magnitude = sqrt(fabs(real) + fabs(imag));
}
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LCD_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  generate_ODR_Buff();
  generateLuts();
  HAL_DMA_Start(&hdma_tim6_up, (uint32_t)&ODR_Buff, (uint32_t)&GPIOC->ODR, TX_BUFF_SIZE);
  __HAL_TIM_ENABLE_DMA(&htim6, TIM_DMA_UPDATE);
  HAL_TIM_Base_Start(&htim6);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)samples, SAMPLE_BUFF_SIZE);
  //__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);



	/*//find the size of array
	int size = sizeof(data)/sizeof(data[0]);

	//declare new array to store reverse of original array
	int k=0, reverse[size];

	//Loop from back and assign value to new array
	for(int i=size-1; i>=0; ){
		  reverse[k++] = data[i--];
	}*/


	for(int i = 0; i < 4; i++){
		  printf("Display = %d \n", Display[i]);
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(dataRdyFlag)
	  {
		  memset(Display, 0x00, 8);
		  	  tempf = constrainAngle(phaseDeg-offset);
		  	  tempf *= 100;
		  	  tempi = (int)tempf;
		  	  configure(data, angleLut[tempi]);
		  	  DisplayUpdate(Display, data);
		  	  HAL_LCD_Clear(&hlcd);
		  	  HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER0, 0xffff, Display[0]);
		  	  HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER2, 0xffff, Display[1]);
		  	  HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER4, 0xffff, Display[2]);
		  	  HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER6, 0xffff, Display[3]);
		  	  HAL_LCD_UpdateDisplayRequest(&hlcd);
		  dataRdyFlag = 0;
		  sensor_signalProcessing();
		  printf("%i\n\r", (int)phaseDegDisp);
		  HAL_Delay(50);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EXT_IT11;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_31;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_3;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_3;
  hlcd.Init.DeadTime = LCD_DEADTIME_3;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_1;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_ENABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  hlcd.Init.HighDrive = LCD_HIGHDRIVE_ENABLE;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the High Driver
  */
  __HAL_LCD_HIGHDRIVER_ENABLE(&hlcd);
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 246-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWM0_Pin|PWM45_Pin|PWM90_Pin|PWM135_Pin
                          |PWM180_Pin|PWM225_Pin|PWM270_Pin|PWM315_Pin
                          |BIAS_Pin|ADC_TRIG_OUT_Pin|PERIOD_Pin|TIMEBASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PWM0_Pin PWM45_Pin PWM90_Pin PWM135_Pin
                           PWM180_Pin PWM225_Pin PWM270_Pin PWM315_Pin
                           ADC_TRIG_OUT_Pin PERIOD_Pin TIMEBASE_Pin */
  GPIO_InitStruct.Pin = PWM0_Pin|PWM45_Pin|PWM90_Pin|PWM135_Pin
                          |PWM180_Pin|PWM225_Pin|PWM270_Pin|PWM315_Pin
                          |ADC_TRIG_OUT_Pin|PERIOD_Pin|TIMEBASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BIAS_Pin */
  GPIO_InitStruct.Pin = BIAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BIAS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(processBuff, samples, 128);
	dataRdyFlag = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(processBuff, samples+64, 128);
	dataRdyFlag = 1;
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

