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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 360 * 20	//采样点
#define SAMPLE_RATE 16000	//采样率
#define PI 			3.14159265359f
#define SYNC_CODE_NUM 8
#define THRESHOLD 1800
#define LED_CODE_NUM 8
#define FFH_CODE_NUM 8
#define SYNC_NUM_MAX 15 * 20
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
uint16_t sync_code[SYNC_CODE_NUM] = {1,0,1,0,1,0,1,0};
uint16_t LED0_code[LED_CODE_NUM] = {0,0,0,0,1,0,1,0};
uint16_t LED1_code[LED_CODE_NUM] = {0,0,1,0,1,1,0,1};
uint16_t LED2_code[LED_CODE_NUM] = {0,0,0,1,1,0,0,1};
uint16_t FFH[FFH_CODE_NUM] = {1,1,1,1,1,1,1,1};

int sync_index[SYNC_NUM_MAX] = {-1};	//存放同步头的索引值
uint8_t sync_num = 0;	//同步头个数
int led_index[SYNC_NUM_MAX] = {-1};		//存放对应同步头索引值的led类型，0为led0，1为led1，2为led2，-1为无

//存放led对应的光强值
uint32_t led0_intensity = 0;
uint32_t led1_intensity = 0;
uint32_t led2_intensity = 0;




/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{             
    float x;                //x坐标
    float y;                //y坐标
}Point;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// ... 现有变量声明 ...

// 添加定位相关变量
Point current_position = {0.0f, 0.0f};  // 当前位置
float distances[3] = {0.0f, 0.0f, 0.0f};  // 三个LED的距离

// LED固定位置坐标
static const Point LED_POSITIONS[3] = {
    {-10.0f, 0.0f},  // LED0位置
    {10.0f, 0.0f},   // LED1位置
    {0.0f, 10.0f}    // LED2位置
};
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
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  lcd_init();
//  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

//  lcd_show_string(30, 90, 50, 16, 16, "1K: ", RED);
//  lcd_show_string(30, 110, 50, 16, 16, "2K: ", RED);
//  lcd_show_string(30, 130, 50, 16, 16, "5K: ", RED);

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
		  calculate_distance();
		  calculate_position();
		  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
		  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		  lcd_clear(WHITE);
		  lcd_show_xnum(30, 150, led0_intensity, 4, 16, 0X80, RED);
		  lcd_show_xnum(30, 180, led1_intensity, 4, 16, 0X80, RED);
		  lcd_show_xnum(30, 210, led2_intensity, 4, 16, 0X80, RED);
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
	led0_intensity = led0_intensity_sum / led0_num;
	led1_intensity = led1_intensity_sum / led1_num;
	led2_intensity = led2_intensity_sum / led2_num;
}


// 光强度转换为距离的函数
float calculate_distance() {
    // TODO: 根据实际测试数据实现光强到距离的转换
	distances[0]= -2E-05f * led0_intensity * led0_intensity + 0.0516f * led0_intensity + 0.2903f;
	distances[1]= -2E-05f * led1_intensity * led1_intensity + 0.0899f * led1_intensity - 44.861f;
	distances[2]= -2E-05f * led2_intensity * led2_intensity + 0.0773f * led2_intensity - 26.894f;
    // 这里需要添加您的转换公式

}

// 三点定位函数
Point threePoints(float *dis, Point *ps) 
{
    Point p = {0,0}; //初始化点为无效值
    if (dis == NULL || ps == NULL)
        return p;

    for (int i = 0; i < 3; ++i)
    {
        //检查距离是否有问题
        if (dis[i] < 0)
            return p;

        for (int j = i + 1; j < 3; ++j) 
        {
            //圆心距离PQ
            float p2p = (float)sqrt((ps[i].x - ps[j].x)*(ps[i].x - ps[j].x) +
                                    (ps[i].y - ps[j].y)*(ps[i].y - ps[j].y));
            //判断两圆是否相交
            if (dis[i] + dis[j] <= p2p) 
            {
                //不相交，按比例求
                p.x += ps[i].x + (ps[j].x - ps[i].x)*dis[i] / (dis[i] + dis[j]);
                p.y += ps[i].y + (ps[j].y - ps[i].y)*dis[i] / (dis[i] + dis[j]);
            }
            else
            {
                //相交则套用公式
                //PC
                float dr = p2p / 2 + (dis[i] * dis[i] - dis[j] * dis[j]) / (2 * p2p); 
                //x = xp + (xq-xp) * PC / PQ
                p.x += ps[i].x + (ps[j].x - ps[i].x)*dr / p2p;
                //y = yp + (yq-yp) * PC / PQ
                p.y += ps[i].y + (ps[j].y - ps[i].y)*dr / p2p;
            }
        }
    }
    
    //三个圆两两求点，最终得到三个点，求其均值
    p.x /= 3;
    p.y /= 3;

    return p;
}

// 位置计算函数
void calculate_position(void) {
    // 计算每个LED的距离
    
}

/*
@param: fft_in: fft变换的输入, 对于本项目应该是滤波器的输出
@param: mag: 根据fft变换后求得的正弦信号幅值
@param: freq: 根据fft变换后求得的正弦信号频率
*/
//void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq) {
//	float32_t fft_out[SAMPLE_SIZE];
//	float32_t magnitude[SAMPLE_SIZE/2];
//	float32_t maxValue = 0.0;
//	uint16_t maxIndex = 0;
//
//	arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);
//	arm_cmplx_mag_f32(fft_out, magnitude, SAMPLE_SIZE/2);
//	maxValue = magnitude[1];
//	maxIndex = 1;
//	for(int i = 2; i < SAMPLE_SIZE/2; i++) {
//		if(magnitude[i] > maxValue) {
//			maxValue = magnitude[i];
//			maxIndex = i;
//		}
//	}
//	*mag = (maxValue * 2.0 / SAMPLE_SIZE) * 3.3f / 4096.0f;
//	*freq = (float32_t)maxIndex * SAMPLE_RATE / SAMPLE_SIZE;
//}
//
//void filter_test() {
//	float32_t input_signal[SAMPLE_SIZE];
//	float32_t out_1k[SAMPLE_SIZE];
//	float32_t out_2k[SAMPLE_SIZE];
//	float32_t out_5k[SAMPLE_SIZE];
//
//	float32_t mag1k = 0.0;
//	float32_t freq1k = 0.0;
//	float32_t mag2k = 0.0;
//	float32_t freq2k = 0.0;
//	float32_t mag5k = 0.0;
//	float32_t freq5k = 0.0;
//
//	//生成合成信号
//	for(int i = 0; i < SAMPLE_SIZE; i++) {
//		float t = (float)i / SAMPLE_RATE;
//
//		float pwm1k = (fmodf(t * 1000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
//		float pwm2k = (fmodf(t * 2000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
//		float pwm5k = (fmodf(t * 5000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
//		input_signal[i] = (uint16_t)((pwm1k + pwm2k + pwm5k) * 4096.f / 3.3f);
//	}
//
//	arm_biquad_cascade_df1_f32(&S_1k, input_signal, out_1k, SAMPLE_SIZE);
//	arm_biquad_cascade_df1_f32(&S_2k, input_signal, out_2k, SAMPLE_SIZE);
//	arm_biquad_cascade_df1_f32(&S_5k, input_signal, out_5k, SAMPLE_SIZE);
//
//	fft_process(out_1k, &mag1k, &freq1k);
//	fft_process(out_2k, &mag2k, &freq2k);
//	fft_process(out_5k, &mag5k, &freq5k);
//
//	char mag1k_str[10];
//	char freq1k_str[10];
//	char mag2k_str[10];
//	char freq2k_str[10];
//	char mag5k_str[10];
//	char freq5k_str[10];
//
//	sprintf(mag1k_str, "%.3f", mag1k);
//	sprintf(freq1k_str, "%.3f", freq1k);
//	sprintf(mag2k_str, "%.3f", mag2k);
//	sprintf(freq2k_str, "%.3f", freq2k);
//	sprintf(mag5k_str, "%.3f", mag5k);
//	sprintf(freq5k_str, "%.3f", freq5k);
//
//	lcd_show_string(30, 150, 200, 16, 16, mag1k_str, BLUE);
//	lcd_show_string(30, 170, 200, 16, 16, freq1k_str, BLUE);
//	lcd_show_string(30, 190, 200, 16, 16, mag2k_str, BLUE);
//	lcd_show_string(30, 210, 200, 16, 16, freq2k_str, BLUE);
//	lcd_show_string(30, 230, 200, 16, 16, mag5k_str, BLUE);
//	lcd_show_string(30, 250, 200, 16, 16, freq5k_str, BLUE);
//}
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
