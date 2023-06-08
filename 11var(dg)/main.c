/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//карта регистров
#define DEVID_ID			0x00
#define THRESH_TAP			0x1D
#define OFSX 			    0x1E
#define OFSY 				0x0F
#define OFSZ 				0x20
#define DUR 				0x21
#define Latent 				0x22
#define Window 				0x23
#define THRESH_ACT			0x24
#define THRESH_INACT		0x25
#define TIME_INACT			0x26
#define ACT_INACT_CTL		0x27
#define THRESH_FF			0x28
#define TIME_FF 			0x29
#define TAP_AXES			0x2A
#define ACT_TAP_STATUS		0x2B
#define BW_RATE 			0x2C
#define POWER_CTL			0x2D
#define INT_ENABLE			0x2E
#define INT_MAP 			0x2F
#define INT_SOURCE			0x30
#define DATA_FORMAT			0x31
#define DATAX0   			0x32
#define DATAX1   			0x33
#define DATAY0   			0x34
#define DATAY1   			0x35
#define DATAZ0   			0x36
#define DATAZ1   			0x37
#define FIFO_CTL  			0x38
#define FIFO_STATUS  		0x39


//полоса частот и чвстота дискретизации
#define BWRATE_0_10 	0x00
#define BWRATE_0_20 	0x01
#define BWRATE_0_39 	0x02
#define BWRATE_0_78 	0x03
#define BWRATE_1_56 	0x04
#define BWRATE_3_13 	0x05
#define BWRATE_6_25 	0x06
#define BWRATE_12_5 	0x07
#define BWRATE_25 		0x08
#define BWRATE_50 		0x09
#define BWRATE_100		0x0A
#define BWRATE_200		0x0B
#define BWRATE_400		0x0C
#define BWRATE_800		0x0D
#define BWRATE_1600   	0x0E
#define BWRATE_3200   	0x0F

//диапазон
#define RANGE_2G		0x00
#define RANGE_4G		0x01
#define RANGE_8G		0x02
#define RANGE_16G		0x03



//прерывания
#define DATA_READY 		0x80
#define SINGLE_TAP 		0x40
#define DOUBLE_TAP	    0x20
#define Activity   		0x10
#define Inactivity 		0x08
#define FREE_FALL		0x04
#define Watermark		0x02
#define Overrun			0x01


//



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t data_rec[6]; //массив для хранения данных при чтении
int16_t x, y, z;//переменные для хранения значений ускорения в необработанном формате
float xg, yg, zg; //переменные для значений ускорения пересчитанных в доли ускорения свободного падения

uint8_t adxl_addr = 0x53;
int timeout = 100;

bool upd_flg=false;
bool ff_flg=false;
bool launch_flg=false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IPCC_Init(void);
static void MX_RF_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write(uint8_t address_reg, uint8_t value) {//запись в регистр
	uint8_t data[2];//массив для хранения посылки
	data[0] = address_reg;//cначала передаем адрес регистра в который будем читать
	data[1] = value; //Затем значение которое нужно записываем
	HAL_I2C_Master_Transmit(&hi2c1, adxl_addr, data, 2, timeout)  ;//отправляем массив с адресом и значением
}



uint8_t adxl_read(uint8_t address_reg) {//чтение с регистра одного байта
	address_reg |= 0x80;  // маска для задания бита чтения
	uint8_t data[1]={0}; //переменная для прочитанных данных

	HAL_I2C_Master_Transmit(&hi2c1, adxl_addr, address_reg, 1, timeout);//посылаем адрес регистра, с которого хотим читать

	HAL_I2C_Master_Receive(&hi2c1, adxl_addr, data, 1, timeout);//читаем байт в переменную data
	return data;
}




void adxl_init(void) {//инициализация акселерометра и настройка

	adxl_write(DATA_FORMAT, RANGE_8G);  //настраиваем диапазон +- 8g

	adxl_write(POWER_CTL, 0x00);  // выход из режима сна
	adxl_write(POWER_CTL, 0x08);  //включаем преобразование
	adxl_write(THRESH_FF, 0x04);  //настраиваем значение free fall treshold = 0.5g
	adxl_write(TIME_FF, 0x02);     //настраиваем время free fall
	//по двум параметрам выше срабатывает прерывание

	adxl_write(INT_ENABLE, FREE_FALL);     //включаем прерывание от free fall
	adxl_write(INT_MAP, FREE_FALL);  //назначаем его на вывод IN1

	adxl_write(INT_ENABLE, DATA_READY); //включаем прерывание по готовности данных
	adxl_write(INT_MAP, !DATA_READY);  //назначаем его на вывод IN0

}

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
  HAL_Init();//подключение библиотеки HAl
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config(); //инициализация Bluetooth стека

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); //инициализация тактирования

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
   MX_IPCC_Init();//инициализация Inter-processor communication controller

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); //инициализация портов ввода вывода
  MX_I2C1_Init(); //инициализация I2C
  MX_RF_Init(); //инициализация радиомодуля
  MX_RTC_Init(); //инициализация real time clock
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init(); //
  adxl_init(); //инициализация и настройка акселерометра



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process(); //запуск функций, связанных со стеком Bluetooth

    if(launch_flg==true){
    	uint16_t dataX = adxl_read(DATAX0)|(adxl_read(DATAX1)<<8);
    	uint16_t dataY = adxl_read(DATAY0)|(adxl_read(DATAY1)<<8);
    	uint16_t dataZ = adxl_read(DATAZ0)|(adxl_read(DATAZ1)<<8);
    	if (){

    	}

    }



    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}





void EXTI4_15_IRQHandler(void) //Прерывание по старту импульса с пинов РА4(INT1) и РА5(INT2)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET){   //Узнаем на каком выводе РА5 или РА4 был импульс
		if( upd_flg == false){      //Если этот флаг не активен, то
			 HAL_TIM_Base_Start(&htim2); //Запускаем таймер для считывания данных
			  upd_flg=true;
		  }
	}
	else{
		if( ff_flg == false){
			HAL_TIM_Base_Start(&htim2); //Запускаем таймер для считывания данных
			ff_flg= true;
		}
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5|GPIO_PIN_6); //сброс флага прерывания
}



void TIM2_IRQHandler(void)//прерывание от таймера - запускает акселерометр каждые 2.5мс
{
	launch_flg = true;
	__HAL_TIM_SET_COUNTER(&htim2, 0); //Сбрасываем значение счетчика для дальнейшего счета
	HAL_TIM_IRQHandler(&htim2); //Сброс флагов таймера

}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) //настройки тактирования(настраивались в кодогенераторе Cube)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)//настройки тактирования для радиочасти (настраивались в кодогенераторе Cube)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */
  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) //настройки i2c (настраивались в кодогенераторе Cube)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)//настройки интерфейса связи процессорных ядер(настраивались в кодогенераторе Cube)
{

  /* USER CODE BEGIN IPCC_Init 0 */
  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */
  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */
  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)//настройки часов реального времени(настраивались в кодогенераторе Cube)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 640-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */
  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */
  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */
  /* USER CODE END RF_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) //настройки портов ввода вывода(настраивались в кодогенераторе Cube)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
