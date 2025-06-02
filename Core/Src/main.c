/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/BSP/LCD/lcd.h"
#include "arm_math.h"
#include "../../Drivers/BSP/RSS/rss.h"	//三点定位法
#include "../../Drivers/BSP/BEEP/beep.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 480 * 20	//采样点
#define SAMPLE_RATE 16000	//采样率
#define PI 			3.14159265359f
#define SYNC_CODE_NUM 8
#define THRESHOLD 1800
#define LED_CODE_NUM 8
#define FFH_CODE_NUM 8
#define LED_MESSAGE_NUM 8	//LED消息长度
#define SYNC_NUM_MAX 15 * 20

#define SOUND_FLAG	A	//接收到LED1的数据为A时进入接收音频信息状态
#define TONE_MAX_NUM   100	//音频信息缓冲区大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq);
//void filter_test();

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[SAMPLE_SIZE];	//ADC采样数据缓冲区

__IO uint8_t AdcConvEnd = 0;		//ADC采样完成标志

uint16_t capture_buffer[2];			//输入捕获值
uint8_t capture_state = 0;			//0为上升沿捕获，1为下降沿捕获

uint8_t logic_buffer[SAMPLE_SIZE];	//逻辑数据缓冲区，1或0
uint16_t sync_code[SYNC_CODE_NUM] = {1,1,1,0,1,0,1,0};
uint16_t LED0_code[LED_CODE_NUM] = {0,0,0,0,1,0,1,0};	//10
uint16_t LED1_code[LED_CODE_NUM] = {0,0,1,0,1,1,0,1};
uint16_t LED2_code[LED_CODE_NUM] = {0,0,0,1,1,0,0,1};
uint16_t FFH[FFH_CODE_NUM] = {1,1,1,1,1,1,1,1};

int sync_index[SYNC_NUM_MAX];	//存放同步头的索引值
uint8_t sync_num = 0;	//同步头个数
int led_index[SYNC_NUM_MAX];		//存放对应同步头索引值的led类型，0为led0，1为led1，2为led2，-1为无

//存放led对应的光强值
uint32_t led0_intensity = 0;
uint32_t led1_intensity = 0;
uint32_t led2_intensity = 0;

//存放led的位置
Point led_locations[3] = {{0, 10}, {-10, 0}, {10, 0}};	//led0在(0, 10)，led1在(-10, 0)，led2在(10, 0)

//存放PD位置
Point PDlocation = {-1, -1};	//初始化为无效值

//存放led消息
uint8_t led0_message = 0;
uint8_t led1_message = 0;
uint8_t led2_message = 0;

//存放音频的音调和节拍信息
float tones[TONE_MAX_NUM];
uint8_t beats[TONE_MAX_NUM];
uint8_t tone_index = 0;
uint8_t is_recording = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  lcd_init();
//  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

//  lcd_show_string(30, 90, 50, 16, 16, "1K: ", RED);
//  lcd_show_string(30, 110, 50, 16, 16, "2K: ", RED);
//  lcd_show_string(30, 130, 50, 16, 16, "5K: ", RED);
	//初始化同步索引与led索引数组为全-1
	for(int i = 0; i < SYNC_NUM_MAX; i++)
  	{
		sync_index[i] = -1;
		led_index[i] = -1;
  	}

  //filter_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(capture_state == 2 && AdcConvEnd)	//检测buffer是否填满
	  {
		  AdcConvEnd = 0;
		  //HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		  capture_state = 0;
		  HAL_TIM_Base_Stop(&htim3);	//一个buffer填满后重新捕获以同步时钟

		  to_logic();
		  sync();
		  find_led();
		  get_led_intensity();

		  get_led_message();

		  get_location();	//获取PD位置
		  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
		  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

		  char string_x[30];
		  char string_y[30];
		  sprintf(string_x, "x: %.3f", PDlocation.x);
		  sprintf(string_y, "y: %.3f", PDlocation.y);
		  char led0_message_str[30];
		  char led1_message_str[30];
		  char led2_message_str[30];
		  sprintf(led0_message_str, "LED0: %d", led0_message);
		  sprintf(led1_message_str, "LED1: %X", led1_message);
		  sprintf(led2_message_str, "LED2: %d", led2_message);
		  lcd_clear(WHITE);
		  lcd_show_xnum(30, 30, led0_intensity, 4, 16, 0X80, RED);
		  lcd_show_xnum(110, 30, led1_intensity, 4, 16, 0X80, RED);
		  lcd_show_xnum(190, 30, led2_intensity, 4, 16, 0X80, RED);
	//		  lcd_show_xnum(30, 240, x, 4, 16, 0X00, RED);
	//		  lcd_show_xnum(60, 240, x_small, 4, 16, 0X00, RED);
	//		  lcd_show_xnum(30, 270, y, 4, 16, 0X00, RED);
	//		  lcd_show_xnum(60, 270, y_small, 4, 16, 0X00, RED);
		  lcd_show_string(60, 60, 200, 32, 32, string_x, BLUE);
		  lcd_show_string(60, 90, 200, 32, 32, string_y, BLUE);
		  lcd_show_string(60, 150, 200, 32, 32, led0_message_str, DARKBLUE);
		  lcd_show_string(60, 180, 200, 32, 32, led1_message_str, DARKBLUE);
		  lcd_show_string(60, 210, 200, 32, 32, led2_message_str, DARKBLUE);
	  }


//		  lcd_show_string(30, 90, 50, 16, 16, "1K: ", RED);
//		  lcd_show_string(30, 110, 50, 16, 16, "2K: ", RED);
//		  lcd_show_string(30, 130, 50, 16, 16, "5K: ", RED);
		  //lcd_clear(WHITE);


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		__HAL_TIM_SET_COUNTER(&htim3, 0);	//计时值清零
		HAL_TIM_Base_Start(&htim3);		//开启16k定时器3
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer, SAMPLE_SIZE);	//进行ADCDMA采集
		HAL_TIM_Base_Stop_IT(&htim2);	//关闭32k定时器中断
	}
//	if(htim->Instance == TIM3)
//	{
//		HAL_GPIO_TogglePin(SYNC_GPIO_Port, SYNC_Pin);
//	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		switch(capture_state)
		{
			case 0 :
				capture_buffer[0] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);	//捕获上升沿
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
				capture_state = 1;
				break;
			case 1 :
				capture_buffer[1] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);	//捕获下降沿
				if(capture_buffer[1] - capture_buffer[0] <= 50)	//高电平时间太短，判定为误判
				{
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
					capture_state = 0;
				}
				else
				{
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
					capture_state = 2;
					HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);	//关闭捕获

//					HAL_TIM_Base_Start(&htim3);		//开启定时器
					__HAL_TIM_SET_COUNTER(&htim2, 0);	//计时值清零
					HAL_TIM_Base_Start_IT(&htim2);	//开始32k定时器中断
//					HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer, SAMPLE_SIZE);	//进行ADCDMA采集
				}
				break;
		}

	}

}

void to_logic()
{
	for(int i = 0; i < SAMPLE_SIZE; i++)
	{
		if(adc_buffer[i] > THRESHOLD)
		{
			logic_buffer[i] = 1;
		}
		else
		{
			logic_buffer[i] = 0;
		}
	}
}

void sync()
{
	sync_num = 0;
	for(int i = 0; i < SAMPLE_SIZE - SYNC_CODE_NUM; i++)
	{
		uint8_t sync_score = 0;
		for(int j = i; j < i + SYNC_CODE_NUM; j++)
		{
			if(logic_buffer[j] == sync_code[j - i])
			{
				sync_score++;
			}
		}
		if(sync_score >= 8)
		{
			sync_index[sync_num] = i;
			sync_num++;
		}
	}
}

void find_led()
{
	for(int i = 0; i < sync_num - 1; i++)	//舍弃最后一个同步头，防止越界
	{
		int index = sync_index[i];
		uint8_t led0_score = 0;
		uint8_t led1_score = 0;
		uint8_t led2_score = 0;
		if(index > -1)
		{
			for(int j = index + SYNC_CODE_NUM; j < index + SYNC_CODE_NUM + LED_CODE_NUM; j++)
			{
				if(logic_buffer[j] == LED0_code[j - index - SYNC_CODE_NUM])
				{
					led0_score++;
				}
				if(logic_buffer[j] == LED1_code[j - index - SYNC_CODE_NUM])
				{
					led1_score++;
				}
				if(logic_buffer[j] == LED2_code[j - index - SYNC_CODE_NUM])
				{
					led2_score++;
				}
			}
			if(led0_score >= 8)
			{
				led_index[i] = 0;
			}
			else if(led1_score >= 8)
			{
				led_index[i] = 1;
			}
			else if(led2_score >= 8)
			{
				led_index[i] = 2;
			}
		}
	}
}

void get_led_intensity()
{
	uint32_t led0_intensity_sum = 0;
	uint32_t led1_intensity_sum = 0;
	uint32_t led2_intensity_sum = 0;

	uint16_t led0_num = 0;
	uint16_t led1_num = 0;
	uint16_t led2_num = 0;

	for(int i = 0; i < sync_num - 1; i++)	//舍弃最后一个同步头，防止越界
	{
		int led = led_index[i];
		int index = sync_index[i];
		switch(led)
		{
			case 0:
				for(int j = 1; j < FFH_CODE_NUM; j++)
				{
					led0_intensity_sum += adc_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM];
					led0_num++;
				}
				break;
			case 1:
				for(int j = 1; j < FFH_CODE_NUM; j++)
				{
					led1_intensity_sum += adc_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM];
					led1_num++;
				}
				break;
			case 2:
				for(int j = 1; j < FFH_CODE_NUM; j++)
				{
					led2_intensity_sum += adc_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM];
					led2_num++;
				}
				break;
			default :

				break;
		}
	}
	led0_intensity = (led0_num > 0 && led0_intensity_sum > 0) ? led0_intensity_sum / led0_num : led0_intensity;	//避免除数为0
	led1_intensity = (led1_num > 0 && led1_intensity_sum > 0) ? led1_intensity_sum / led1_num : led1_intensity;
	led2_intensity = (led2_num > 0 && led2_intensity_sum > 0) ? led2_intensity_sum / led2_num : led2_intensity;
}

void get_led_message()
{

	for(int i = 0; i < sync_num - 5; i++)	//舍弃最后几个同步头，防止越界
	{
		int led = led_index[i];
		int index = sync_index[i];
		switch(led)
		{
			case 0:
				led0_message = 0;
				for(int j = 0; j < LED_MESSAGE_NUM; j++)
				{
					uint8_t logic = logic_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM + FFH_CODE_NUM];
					led0_message |= (logic << (LED_MESSAGE_NUM - 1 - j));	//将逻辑值转换为二进制消息
				}
				break;
			case 1:
				led1_message = 0;
				for(int j = 0; j < LED_MESSAGE_NUM; j++)
				{
					uint8_t logic = logic_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM + FFH_CODE_NUM];
					led1_message |= (logic << (LED_MESSAGE_NUM - 1 - j));	//将逻辑值转换为二进制消息
				}
				break;
			case 2:
				led2_message = 0;
				for(int j = 0; j < LED_MESSAGE_NUM; j++)
				{
					uint8_t logic = logic_buffer[j + index + SYNC_CODE_NUM + LED_CODE_NUM + FFH_CODE_NUM];
					led2_message |= (logic << (LED_MESSAGE_NUM - 1 - j));	//将逻辑值转换为二进制消息
				}
				break;
			default :

				break;
		}
	}
}

void get_location()
{
	uint16_t location = 0;
	float led0_radius = 0.0f;
	float led1_radius = 0.0f;
	float led2_radius = 0.0f;

//	led0_radius = -8.258245051149834e-11*led0_intensity*led0_intensity*led0_intensity*led0_intensity+8.603303598173248e-7*led0_intensity*led0_intensity*led0_intensity-0.003343692130742398*led0_intensity*led0_intensity+5.716859893344553*led0_intensity-3581.1780710049975;	//根据实验数据拟合的函数
//	led1_radius = -3.5960151839135584e-10*led1_intensity*led1_intensity*led1_intensity*led1_intensity+0.000003851077174734846*led1_intensity*led1_intensity*led1_intensity-0.015389238714354235*led1_intensity*led1_intensity+27.158329518946147*led1_intensity-17805.994712478332;
//	led2_radius =  -5.040206617568613e-11*led2_intensity*led2_intensity*led2_intensity*led2_intensity+4.751187260064662e-7*led2_intensity*led2_intensity*led2_intensity-0.0016747497267775738*led2_intensity*led2_intensity+2.58985725128424*led2_intensity-1435.2323446989374;

	if(led0_intensity <= 3325 && led0_intensity >= 3300){
		led0_radius = -0.2*led0_intensity+665;
	}else if(led0_intensity <= 3300 && led0_intensity >= 3200){
		led0_radius = -0.05*led0_intensity+170;
	}else if(led0_intensity <= 3200 && led0_intensity >= 3075){
		led0_radius = -0.04*led0_intensity+138;
	}else if(led0_intensity <= 3075 && led0_intensity >= 2900){
		led0_radius = -0.0286*led0_intensity+102.8571;
	}else if(led0_intensity <= 2900 && led0_intensity >= 2730){
		led0_radius = -0.0294*led0_intensity+105.2941;
	}else if(led0_intensity <= 2730 && led0_intensity >= 2560){
		led0_radius = -0.0294*led0_intensity+105.2941;
	}else if(led0_intensity <= 2560 && led0_intensity >= 2360){
		led0_radius = -0.025*led0_intensity+94;
	}else if(led0_intensity <= 2360){
		led0_radius = -0.025*led0_intensity+94;
	}else{
		led0_radius = 1;
	}

	if(led1_intensity <= 3270 && led1_intensity >= 3210){
			led1_radius = -0.0833*led1_intensity+272.5;
		}else if(led1_intensity <= 3210 && led1_intensity >= 3150){
			led1_radius = -0.0833*led1_intensity+272.5;
		}else if(led1_intensity <= 3150 && led1_intensity >= 3030){
			led1_radius = -0.0417*led1_intensity+141.25;
		}else if(led1_intensity <= 3030 && led1_intensity >= 2890){
			led1_radius = -0.0357*led1_intensity+123.2143;
		}else if(led1_intensity <= 2890 && led1_intensity >= 2740){
			led1_radius = -0.0333*led1_intensity+116.3333;
		}else if(led1_intensity <= 2740 && led1_intensity >= 2530){
			led1_radius = -0.0238*led1_intensity+90.2381;
		}else if(led1_intensity <= 2530 && led1_intensity >= 2330){
			led1_radius = -0.025*led1_intensity+93.25;
		}else if(led1_intensity <= 2330){
			led1_radius = -0.0312*led1_intensity+107.8125;
		}else{
			led1_radius = 1;
		}

	if(led2_intensity <= 3260 && led2_intensity >= 3220){
			led2_radius = -0.125*led2_intensity+407.5;
		}else if(led2_intensity <= 3220 && led2_intensity >= 3120){
			led2_radius = -0.05*led2_intensity+166;
		}else if(led2_intensity <= 3120 && led2_intensity >= 2990){
			led2_radius = -0.0385*led2_intensity+130;
		}else if(led2_intensity <= 2990 && led2_intensity >= 2835){
			led2_radius = -0.0323*led2_intensity+111.4516;
		}else if(led2_intensity <= 2835 && led2_intensity >= 2660){
			led2_radius = -0.0286*led2_intensity+101;
		}else if(led2_intensity <= 2660 && led2_intensity >= 2475){
			led2_radius = -0.0270*led2_intensity+96.8919;
		}else if(led2_intensity <= 2475 && led2_intensity >= 2275){
			led2_radius = -0.025*led2_intensity+91.875;
		}else if(led2_intensity <= 2275){
			led2_radius =  -0.0225*led2_intensity+86.2387;
		}else{
			led2_radius = 1;
		}

	float distances[3] = {led0_radius > 0 ? led0_radius : 1, led1_radius > 0 ? led1_radius : 1, led2_radius > 0 ? led2_radius : 1};	//存放距离

	PDlocation = getPDlocation(distances, led_locations);	//调用三点定位法获取PD位置
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
