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
#include "arm_math.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 1024	//采样点
#define SAMPLE_RATE 44100	//采样率
#define NUM_STAGES 	12		//IIR滤波器的节数
#define PI 			3.14159265359f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq);
void filter_test();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[SAMPLE_SIZE];	//ADC采样数据缓冲区

float32_t output_1k[SAMPLE_SIZE];		//1k滤波输出
float32_t output_2k[SAMPLE_SIZE];		//2k滤波输出
float32_t output_5k[SAMPLE_SIZE];		//5k滤波输出

float32_t mag1 = 0.0;
float32_t freq1 = 0.0;
float32_t mag2 = 0.0;
float32_t freq2 = 0.0;
float32_t mag5 = 0.0;
float32_t freq5 = 0.0;

__IO uint8_t AdcConvEnd = 0;		//ADC采样完成标志

float32_t filter1k_coeffs[5 * NUM_STAGES] = {
		0.0151253954452778,	0,	-0.0151253954452778,	1.97091236178119,	-0.995639426968661,
		0.0151253954452778,	0,	-0.0151253954452778,	1.98025200304691,	-0.996464594350910,
		0.0150681544818176,	0,	-0.0150681544818176,	1.96307133631485,	-0.987348771804651,
		0.0150681544818176,	0,	-0.0150681544818176,	1.97320131755016,	-0.989589426865339,
		0.0150173823729940,	0,	-0.0150173823729940,	1.95666118808648,	-0.980194505262190,
		0.0150173823729940,	0,	-0.0150173823729940,	1.96644993347593,	-0.983242491693912,
		0.0149763055613663,	0,	-0.0149763055613663,	1.95215178685886,	-0.974719002881203,
		0.0149763055613663,	0,	-0.0149763055613663,	1.96034359939868,	-0.977759379462191,
		0.0149474624714876,	0,	-0.0149474624714876,	1.94980329862502,	-0.971270330868219,
		0.0149474624714876,	0,	-0.0149474624714876,	1.95525536559874,	-0.973493250382313,
		0.0149325974973672,	0,	-0.0149325974973672,	1.94965925987815,	-0.969983821351031,
		0.0149325974973672,	0,	-0.0149325974973672,	1.95157231053033,	-0.970797076736173,
};
float32_t filter1k_state[4 * NUM_STAGES];	//历史状态缓冲区

float32_t filter2k_coeffs[5 * NUM_STAGES] = {
		0.0302586319840269,	0,	-0.0302586319840269,	1.89316844324442,	-0.991296247751760,
		0.0302586319840269,	0,	-0.0302586319840269,	1.92847482911976,	-0.992913402460189,
		0.0300313627713143,	0,	-0.0300313627713143,	1.87889223511091,	-0.974847185685376,
		0.0300313627713143,	0,	-0.0300313627713143,	1.91429419887443,	-0.979205769015317,
		0.0298318937589531,	0,	-0.0298318937589531,	1.86805017517718,	-0.960748778688919,
		0.0298318937589531,	0,	-0.0298318937589531,	1.90033712170697,	-0.966639312044064,
		0.0296719427211078,	0,	-0.0296719427211078,	1.86133986824491,	-0.950009192878943,
		0.0296719427211078,	0,	-0.0296719427211078,	1.88728078996523,	-0.955854553168001,
		0.0295603806262901,	0,	-0.0295603806262901,	1.85903753130330,	-0.943256468996250,
		0.0295603806262901,	0,	-0.0295603806262901,	1.87585926870832,	-0.947514797398655,
		0.0295031239037419,	0,	-0.0295031239037419,	1.86102264065125,	-0.940721824307259,
		0.0295031239037419,	0,	-0.0295031239037419,	1.86685077411769,	-0.942276856898966,
};
float32_t filter2k_state[4 * NUM_STAGES];

float32_t filter5k_coeffs[5 * NUM_STAGES] = {
		0.0764409286375503,	0,	-0.0764409286375503,	1.39319484211737,	-0.978355411731109,
		0.0764409286375503,	0,	-0.0764409286375503,	1.59411240800055,	-0.981854652160744,
		0.0750266960934088,	0,	-0.0750266960934088,	1.37251974419307,	-0.938121759700834,
		0.0750266960934088,	0,	-0.0750266960934088,	1.56090360602338,	-0.947347954653341,
		0.0738232862834897,	0,	-0.0738232862834897,	1.36285856394326,	-0.904188310612822,
		0.0738232862834897,	0,	-0.0738232862834897,	1.52550587294765,	-0.916428780562072,
		0.0728825141021567,	0,	-0.0728825141021567,	1.36414235472618,	-0.878518754877327,
		0.0728825141021567,	0,	-0.0728825141021567,	1.48950444734561,	-0.890492950172149,
		0.0722385852128557,	0,	-0.0722385852128557,	1.37553970244748,	-0.862270578809138,
		0.0722385852128557,	0,	-0.0722385852128557,	1.45460066033068,	-0.870909690378008,
		0.0719118970515237,	0,	-0.0719118970515237,	1.39564107169777,	-0.855845682851572,
		0.0719118970515237,	0,	-0.0719118970515237,	1.42265339618337,	-0.858985120388060,
};
float32_t filter5k_state[4 * NUM_STAGES];

arm_biquad_casd_df1_inst_f32 S_1k;	//1k filter
arm_biquad_casd_df1_inst_f32 S_2k;	//2k filter
arm_biquad_casd_df1_inst_f32 S_5k;	//5k filter
arm_rfft_fast_instance_f32 fft_instance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  arm_biquad_cascade_df1_init_f32(&S_1k, NUM_STAGES, filter1k_coeffs, filter1k_state);	//1k filter initialization
  arm_biquad_cascade_df1_init_f32(&S_2k, NUM_STAGES, filter2k_coeffs, filter2k_state);	//2k filter initialization
  arm_biquad_cascade_df1_init_f32(&S_5k, NUM_STAGES, filter5k_coeffs, filter5k_state);	//5k filter initialization

  arm_rfft_fast_init_f32(&fft_instance, SAMPLE_SIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  lcd_init();
  HAL_TIM_Base_Init(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, SAMPLE_SIZE);

  lcd_show_string(30,  50, 200, 16, 16, "STM32", RED);
  lcd_show_string(30,  70, 200, 16, 16, "Filter Output", RED);
  lcd_show_string(30, 90, 50, 16, 16, "1K: ", RED);
  lcd_show_string(30, 110, 50, 16, 16, "2K: ", RED);
  lcd_show_string(30, 130, 50, 16, 16, "5K: ", RED);

  //filter_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(AdcConvEnd == 1) {
		  //滤波
		  arm_biquad_cascade_df1_f32(&S_1k, (float32_t*)adc_buffer, output_1k, SAMPLE_SIZE);
		  arm_biquad_cascade_df1_f32(&S_2k, (float32_t*)adc_buffer, output_2k, SAMPLE_SIZE);
		  arm_biquad_cascade_df1_f32(&S_5k, (float32_t*)adc_buffer, output_5k, SAMPLE_SIZE);

		  //作fft变换求频率和幅值
		  fft_process(output_1k, &mag1, &freq1);
		  fft_process(output_2k, &mag2, &freq2);
		  fft_process(output_5k, &mag5, &freq5);

		  //显示滤波结果
		  char str_1k[50];
		  char str_2k[50];
		  char str_5k[50];

		  sprintf(str_1k, "%.3f V   %.3f Hz", mag1, freq1);
		  sprintf(str_2k, "%.3f V   %.3f Hz", mag2, freq2);
		  sprintf(str_5k, "%.3f V   %.3f Hz", mag5, freq5);

		  lcd_show_string(80, 90, 200, 16, 16, str_1k, BLUE);
		  lcd_show_string(80, 110, 200, 16, 16, str_2k, BLUE);
		  lcd_show_string(80, 130, 200, 16, 16, str_5k, BLUE);

		  AdcConvEnd = 0;
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, SAMPLE_SIZE);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
/*
@param: fft_in: fft变换的输入, 对于本项目应该是滤波器的输出
@param: mag: 根据fft变换后求得的正弦信号幅值
@param: freq: 根据fft变换后求得的正弦信号频率
*/
void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq) {
	float32_t fft_out[SAMPLE_SIZE];
	float32_t magnitude[SAMPLE_SIZE/2];
	float32_t maxValue = 0.0;
	uint16_t maxIndex = 0;

	arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);
	arm_cmplx_mag_f32(fft_out, magnitude, SAMPLE_SIZE/2);
	maxValue = magnitude[1];
	maxIndex = 1;
	for(int i = 2; i < SAMPLE_SIZE/2; i++) {
		if(magnitude[i] > maxValue) {
			maxValue = magnitude[i];
			maxIndex = i;
		}
	}
	*mag = (maxValue * 2.0 / SAMPLE_SIZE) * 3.3f / 4096.f;
	*freq = (float32_t)maxIndex * SAMPLE_RATE / SAMPLE_SIZE;
}

void filter_test() {
	float32_t input_signal[SAMPLE_SIZE];
	float32_t out_1k[SAMPLE_SIZE];
	float32_t out_2k[SAMPLE_SIZE];
	float32_t out_5k[SAMPLE_SIZE];

	float32_t mag1k = 0.0;
	float32_t freq1k = 0.0;
	float32_t mag2k = 0.0;
	float32_t freq2k = 0.0;
	float32_t mag5k = 0.0;
	float32_t freq5k = 0.0;

	//生成合成信号
	for(int i = 0; i < SAMPLE_SIZE; i++) {
		float t = (float)i / SAMPLE_RATE;

		float pwm1k = (fmodf(t * 1000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
		float pwm2k = (fmodf(t * 2000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
		float pwm5k = (fmodf(t * 5000.0f, 1.0f) < 0.5) ? 1.0f : 0.0f;
		input_signal[i] = (uint16_t)((pwm1k + pwm2k + pwm5k) * 4096.f / 3.3f);
	}

	arm_biquad_cascade_df1_f32(&S_1k, input_signal, out_1k, SAMPLE_SIZE);
	arm_biquad_cascade_df1_f32(&S_2k, input_signal, out_2k, SAMPLE_SIZE);
	arm_biquad_cascade_df1_f32(&S_5k, input_signal, out_5k, SAMPLE_SIZE);

	fft_process(out_1k, &mag1k, &freq1k);
	fft_process(out_2k, &mag2k, &freq2k);
	fft_process(out_5k, &mag5k, &freq5k);

	char mag1k_str[10];
	char freq1k_str[10];
	char mag2k_str[10];
	char freq2k_str[10];
	char mag5k_str[10];
	char freq5k_str[10];

	sprintf(mag1k_str, "%.3f", mag1k);
	sprintf(freq1k_str, "%.3f", freq1k);
	sprintf(mag2k_str, "%.3f", mag2k);
	sprintf(freq2k_str, "%.3f", freq2k);
	sprintf(mag5k_str, "%.3f", mag5k);
	sprintf(freq5k_str, "%.3f", freq5k);

	lcd_show_string(30, 150, 200, 16, 16, mag1k_str, BLUE);
	lcd_show_string(30, 170, 200, 16, 16, freq1k_str, BLUE);
	lcd_show_string(30, 190, 200, 16, 16, mag2k_str, BLUE);
	lcd_show_string(30, 210, 200, 16, 16, freq2k_str, BLUE);
	lcd_show_string(30, 230, 200, 16, 16, mag5k_str, BLUE);
	lcd_show_string(30, 250, 200, 16, 16, freq5k_str, BLUE);
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
