/**
  ******************************************************************************
  * This is the template for Informatica Industrial II. Lunes tarde group.
  * Based on the STMicroelectronics HAL template
  *
  * See http://aperles.blogs.upv.es/arm-cortex-m-practico-1-introduccion-a-los-microcontroladores-stm32-de-st/
  *
  * Good look,
  * Angel Perles
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "vl53l0x_sigma.h"
#include <string.h>
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include <limits.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;
char *RangingConfigTxt[3] = {"LR", "HS", "HA"};

typedef enum {
	RANGE_VALUE 	= 0, /*!< Range displayed in cm */
	BAR_GRAPH 		= 1, /*!< Range displayed as a bar graph : one bar per sensor */
} DemoMode_e;
char *DemoModeTxt[2] = {"rng", "bar"};

/* Private define ------------------------------------------------------------*/
#define debug_printf    trace_printf
/** Time the initial 53L0 message is shown at power up */
#define WelcomeTime 660
#define TXBUFFERSIZE    (COUNTOF(buffInit) - 1)
#define TXBUFFERDEVSIZE    (COUNTOF(buff) - 1)
// I2C addresses of GPIO expanders on the X-NUCLEO-53L1A1
#define EXPANDER_1_ADDR 0x52 // 0x42 << 1
#define EXPANDER_2_ADDR 0x86 // 0x43 << 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 * Global ranging struct
 */
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;
/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
	int LeakyFactorFix8 = (int)( 0.6 *256);
	char uartBUFF[200];
	/** How many device detect set by @a DetectSensors()*/
	int nDevPresent=0;
	/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
	int nDevMask;
	
	VL53L0X_Dev_t VL53L0XDevs[]={
					{.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
					{.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
					{.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
	};
/** range low (and high) in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
	int RangeLow=100;

/** range medium in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
	int RangeMedium=300;
	extern UART_HandleTypeDef huart2;
	extern I2C_HandleTypeDef hi2c1;
  uint8_t buffInit[] = " *** UART2 Init OK ***\n\r ";
	uint8_t buff[50];
  VL53L0X_RangingMeasurementData_t RangingData;
  VL53L0X_Dev_t  vl53l1_c; // center module 
  VL53L0X_DEV    Dev = &vl53l1_c; 
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/
/**
 * Reset all sensor then do presence detection
 *
 * All present devices are data initiated and assigned to their final I2C address
 * @return
 */
int DetectSensors(int SetDisplay) {
    int i;
    uint16_t Id;
    int status;
    int FinalAddress;

    char PresentMsg[5]="    ";
    /* Reset all */
    nDevPresent = 0;
    for (i = 0; i < 3; i++)
        status = XNUCLEO53L0A1_ResetId(i, 0);

    /* detect all sensors (even on-board)*/
    for (i = 0; i < 3; i++) {
        VL53L0X_Dev_t *pDev;
        pDev = &VL53L0XDevs[i];
        pDev->I2cDevAddr = 0x52;
        pDev->Present = 0;
        status = XNUCLEO53L0A1_ResetId( pDev->Id, 1);
        HAL_Delay(2);
        FinalAddress=0x52+(i+1)*2;

        do {
        	/* Set I2C standard mode (400 KHz) before doing the first register access */
        	if (status == VL53L0X_ERROR_NONE)
        		status = VL53L0X_WrByte(pDev, 0x88, 0x00);

        	/* Try to read one register using default 0x52 address */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
            if (status) {
								sprintf(uartBUFF, "#%d Read id fail\n\r", i);
								HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
                break;
            }
            if (Id == 0xEEAA) {
				/* Sensor is found => Change its I2C address to final one */
                status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
                if (status != 0) {
						        sprintf(uartBUFF, "#%d VL53L0X_SetDeviceAddress fail\n\r", i);
						        HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
                    break;
                }
                pDev->I2cDevAddr = FinalAddress;
                /* Check all is OK with the new I2C address and initialize the sensor */
                status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
                if (status != 0) {
						        sprintf(uartBUFF, "#%d VL53L0X_RdWord fail\n\r", i);
						        HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
					break;
				}

                status = VL53L0X_DataInit(pDev);
                if( status == 0 ){
                    pDev->Present = 1;
                }
                else{
                    Error_Handler("main.c",152);
                    break;
                }

										sprintf(uartBUFF, "VL53L0X %d Present and initiated to final 0x%x\n\r", pDev->Id, pDev->I2cDevAddr);
										HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);								
										nDevPresent++;
										nDevMask |= 1 << i;
										pDev->Present = 1;
            }
            else {
										sprintf(uartBUFF, "#%d unknown ID %x\n\r", i, Id);
										HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);								
                   status = 1;
            }
        } while (0);
        /* if fail r can't use for any reason then put the  device back to reset */
        if (status) {
            XNUCLEO53L0A1_ResetId(i, 0);
        }
    }
    /* Display detected sensor(s) */
    if( SetDisplay ){
        for(i=0; i<3; i++){
            if( VL53L0XDevs[i].Present ){
                PresentMsg[i+1]=VL53L0XDevs[i].DevLetter;
            }
        }
        PresentMsg[0]=' ';
        XNUCLEO53L0A1_SetDisplayString(PresentMsg);
        HAL_Delay(1000);
    }

    return nDevPresent;
}

/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err){
    char msg[16];
    sprintf(msg,"Er%d", err);
    XNUCLEO53L0A1_SetDisplayString(msg);
    while(1){};
}

void ResetAndDetectSensor(int SetDisplay){
    int nSensor;
    nSensor = DetectSensors(SetDisplay);
    /* at least one sensor and if one it must be the built-in one  */
    if( (nSensor <=0) ||  (nSensor ==1 && VL53L0XDevs[1].Present==0) ){
        HandleError(66);
    }
}

/** Timer
 *
 * Used to get time stamp for UART logging
 */
TIM_HandleTypeDef htim5;

/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim5);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);

}

void TimeStamp_Init(){
    MX_TIM5_Init();
	__TIM5_CLK_ENABLE();
}

void TimeStamp_Reset(){
    HAL_TIM_Base_Start(&htim5);
    htim5.Instance->CNT=0;
}
	
uint32_t TimeStamp_Get(){
    return htim5.Instance->CNT;
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void SetupSingleShot(RangingConfig_e rangingConfig){
    int i;
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

    for( i=0; i<3; i++){
        if( VL53L0XDevs[i].Present){
            status=VL53L0X_StaticInit(&VL53L0XDevs[i]);
            if( status ){
                //debug_printf("VL53L0X_StaticInit %d failed\n",i);
            }

            status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   //debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
			if( status ){
			   //debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
			}

            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               //debug_printf("VL53L0X_SetDeviceMode failed\n");
            }

            status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
			if( status ){
			   //debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
			if( status ){
			   //debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}
			/* Ranging configuration */
            switch(rangingConfig) {
            case LONG_RANGE:
            	signalLimit = (FixPoint1616_t)(0.1*65536);
            	sigmaLimit = (FixPoint1616_t)(60*65536);
            	timingBudget = 33000;
            	preRangeVcselPeriod = 18;
            	finalRangeVcselPeriod = 14;
            	break;
            case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(18*65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            case HIGH_SPEED:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(32*65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            default:
										sprintf(uartBUFF, "Not Supported\n\r");
										HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);		
            	//debug_printf("Not Supported");
            }

            status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
			if( status ){
			   //debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
			if( status ){
			   //debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  timingBudget);
            if( status ){
               //debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
            }

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
			if( status ){
			   //debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
			if( status ){
			   //debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   //debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

            VL53L0XDevs[i].LeakyFirst=1;
        }
    }
}

char RangeToLetter(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    char c;
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyRange < RangeLow ){
            c='_';
        }
        else if( pDev->LeakyRange < RangeMedium ){
                c='=';
        }
        else {
            c = '~';
        }

    }
    else{
        c='-';
    }
    return c;
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else{
        pDev->LeakyFirst = 1;
    }
}

int BSP_GetPushButton(void){
    GPIO_PinState state ;
    state = HAL_GPIO_ReadPin(BSP_BP_PORT, BSP_BP_PIN);
    return state;
}

/**
 * When button is already pressed it waits for user to release it.
 * if button remains pressed for a given time it returns true.
 * This is used to detect mode switch by long press on blue Push Button
 *
 * As soon as time is elapsed -rb- is displayed to let user know the mode
 * switch is taken into account
 *
 * @return True if button remains pressed more than specified time
 */
int PusbButton_WaitUnPress(void){
    uint32_t TimeStarted;
    TimeStarted = HAL_GetTick();
    while( !BSP_GetPushButton() ){ ; /* debounce */
        if(HAL_GetTick()- TimeStarted> 1000){
            XNUCLEO53L0A1_SetDisplayString (" rb ");
        }
    }
    return  HAL_GetTick() - TimeStarted>1000;

}

/**
  * @brief  Implement the ranging demo with all modes managed through the blue button (short and long press)
  *            This function implements a while loop until the blue button is pressed
  * @param UseSensorsMask Mask of any sensors to use if not only one present
  * @param rangingConfig Ranging configuration to be used (same for all sensors)
  * @retval None
  */
int RangeDemo(int UseSensorsMask, RangingConfig_e rangingConfig){
    int over=0;
    int status;
    char StrDisplay[5];
    char c;
    int i;
    int nSensorToUse;
    int SingleSensorNo=0;

    /* Setup all sensors in Single Shot mode */
    SetupSingleShot(rangingConfig);

    /* Which sensor to use ? */
    for(i=0, nSensorToUse=0; i<3; i++){
        if(( UseSensorsMask& (1<<i) ) && VL53L0XDevs[i].Present ){
            nSensorToUse++;
            if( nSensorToUse==1 )
                SingleSensorNo=i;
        }
    }
    if( nSensorToUse == 0 ){
        return -1;
    }

    /* Start ranging until blue button is pressed */
    do{
        if( nSensorToUse >1 ){
        	/* Multiple devices */
            strcpy(StrDisplay, "    ");
            for( i=0; i<3; i++){
                if( ! VL53L0XDevs[i].Present  || (UseSensorsMask & (1<<i))==0 )
                    continue;
                /* Call All-In-One blocking API function */
                status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[i],&RangingMeasurementData);
                if( status ){
                    HandleError(555);
                }
                /* Push data logging to UART */
                sprintf(uartBUFF, "%d,%u,%d,%d,%d\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);								
								HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
                /* Store new ranging distance */
                Sensor_SetNewRange(&VL53L0XDevs[i],&RangingMeasurementData);
                /* Translate distance in bar graph (multiple device) */
                c = RangeToLetter(&VL53L0XDevs[i],&RangingMeasurementData);
                StrDisplay[i+1]=c;
            }
        }
        else{
            /* only one sensor */
        	/* Call All-In-One blocking API function */
            status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[SingleSensorNo],&RangingMeasurementData);
            if( status ==0 ){
            	/* Push data logging to UART */
										sprintf(uartBUFF, "%d,%u,%d,%d,%d\n", VL53L0XDevs[SingleSensorNo].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
										HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);									
            	Sensor_SetNewRange(&VL53L0XDevs[SingleSensorNo],&RangingMeasurementData);
                /* Display distance in cm */
            	if( RangingMeasurementData.RangeStatus == 0 ){
                    sprintf(StrDisplay, "%3dc",(int)VL53L0XDevs[SingleSensorNo].LeakyRange/10);
                }
                else{
                    sprintf(StrDisplay, "----");
                    StrDisplay[0]=VL53L0XDevs[SingleSensorNo].DevLetter;
                }
            }
            else{
                HandleError(ERR_DEMO_RANGE_ONE);
            }
        }
        XNUCLEO53L0A1_SetDisplayString(StrDisplay);
        /* Check blue button */
        if( !BSP_GetPushButton() ){
            over=1;
            break;
        }
    }while( !over);
    /* Wait button to be un-pressed to decide if it is a short or long press */
    status=PusbButton_WaitUnPress();
    return status;
}

/** Main program **************************************************************/
int main(void)
{
  int ExitWithLongPress;
  RangingConfig_e RangingConfig = LONG_RANGE;
  DemoMode_e DemoMode = RANGE_VALUE;
  int UseSensorsMask = 1<<XNUCLEO53L0A1_DEV_CENTER;	
  /* STM32F4xx HAL library initialization */
  HAL_Init();
  /* Configure the System clock to 80 MHz */
  SystemClock_Config();
 /* Initialize all configured peripherals */
	MY_USART2_UART_Init();
	HAL_UART_Transmit(&huart2, (uint8_t *)buffInit, TXBUFFERSIZE, 100);	
	/*User code*/
	GPIO_LED_Config();
	
	// initialize vl53l1x communication parameters
  XNUCLEO53L0A1_Init();
	sprintf(uartBUFF, "Hi I am Ranging VL53L0X mcu L476RG\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);			
  XNUCLEO53L0A1_SetDisplayString("53L0");
  HAL_Delay(WelcomeTime);
  ResetAndDetectSensor(1);

  /* Set VL53L0X API trace level */
  VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_NONE); // No Trace
  //VL53L0X_trace_config(NULL,TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL); // Full trace
	
  /* Infinite loop */
  while (1)
  {
      /* Display demo mode */
      XNUCLEO53L0A1_SetDisplayString(DemoModeTxt[DemoMode]);
      HAL_Delay(500);

      /* Display Ranging config */
	  XNUCLEO53L0A1_SetDisplayString(RangingConfigTxt[RangingConfig]);
	  HAL_Delay(500);

	  /* Reset and Detect all sensors */
      ResetAndDetectSensor(0);

      /* Reset Timestamping */
      TimeStamp_Reset();
		
      /* Start Ranging demo */
      ExitWithLongPress = RangeDemo(UseSensorsMask, RangingConfig);

      /* Blue button has been pressed (long or short press) */
      if(ExitWithLongPress){
    	  /* Long press : change demo mode if multiple sensors present*/
    	  if( nDevPresent >1 ){
    		  /* If more than one sensor is present then toggle demo mode */
    		  DemoMode = (DemoMode == RANGE_VALUE) ? BAR_GRAPH : RANGE_VALUE;
    		  UseSensorsMask = (DemoMode == BAR_GRAPH) ? 0x7 : 1<<XNUCLEO53L0A1_DEV_CENTER;
    	  }
      } else {
    	  /* Short press : change ranging config */
    	  RangingConfig = (RangingConfig == LONG_RANGE) ? HIGH_SPEED : ((RangingConfig == HIGH_SPEED) ? HIGH_ACCURACY : LONG_RANGE);
      }		
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
