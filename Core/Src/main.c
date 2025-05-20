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
#include <stdio.h>
#include "arm_math.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 2048	//采样点
#define SAMPLE_RATE 44100	//采样率
#define NUM_STAGES 	12		//IIR滤波器的节数
#define PI 			3.14159265359f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq);
void filter_test();
void filter_process(float32_t* filter_in, float32_t* filter_out, int blockSize, int type);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t adc_buffer[SAMPLE_SIZE];	//ADC采样数据缓冲区
volatile float32_t adc_float[SAMPLE_SIZE];

volatile uint8_t AdcConvEnd = 0;		//ADC采样完成标志

const float32_t filter1k_coeffs[5 * NUM_STAGES] = {
		1.0f, 0.0f, -1.0f, 1.971502409451940884821397048654034733772f, -0.995923156557958710877187513688113540411f,
		1.0f, 0.0f, -1.0f, 1.980232763562814080060547894390765577555f, -0.996653029951012903175922019727295264602f,
		1.0f, 0.0f, -1.0f, 1.964161976516262786418565156054683029652f, -0.988164911720096239555743977689417079091f,
		1.0f, 0.0f, -1.0f, 1.973560003825037600933001158409751951694f, -0.9901473659559605344782085012411698699f,
		1.0f, 0.0f, -1.0f, 1.958145747056967866939203304355032742023f, -0.981455833916661135596370968414703384042f,
		1.0f, 0.0f, -1.0f, 1.96718061973285163190894309082068502903f, -0.984152866362351552709242241689935326576f,
		1.0f, 0.0f, -1.0f, 1.953893157013089965090557598159648478031f, -0.976301913169343005272082791634602472186f,
		1.0f, 0.0f, -1.0f, 1.961428036858220647076223031035624444485f, -0.978992146053803580230123770888894796371f,
		1.0f, 0.0f, -1.0f, 1.951652133625503049429994462116155773401f, -0.973032066583144006344241461192723363638f,
		1.0f, 0.0f, -1.0f, 1.95665622336051936436263076757313683629f, -0.974998828760961888661995544680394232273f,
		1.0f, 0.0f, -1.0f, 1.951472062823417452648300240980461239815f, -0.971780675498783552690440501464763656259f,
		1.0f, 0.0f, -1.0f, 1.953226107777264353160262544406577944756f, -0.972500171153397485745983885863097384572f
};
float32_t filter1k_state[4 * NUM_STAGES];	//历史状态缓冲区
const float64_t scale_1k = 0.014220685854816499105179872231019544415f *
						   0.014220685854816499105179872231019544415f *
						   0.014170063934703226379840934612275304971f *
						   0.014170063934703226379840934612275304971f *
						   0.014125134218908735883601934801845345646f *
						   0.014125134218908735883601934801845345646f *
						   0.014088764505500703944074203377567755524f *
						   0.014088764505500703944074203377567755524f *
						   0.01406321613736874996669623527623116388f  *
						   0.01406321613736874996669623527623116388f  *
						   0.014050045804000254115462276160997134866f *
						   0.014050045804000254115462276160997134866f ;

const float32_t filter2k_coeffs[5 * NUM_STAGES] = {
		1.0f, 0.0f, -1.0f, 1.907502786738354938478323674644343554974f, -0.996109351487678762460120651667239144444f,
		1.0f, 0.0f, -1.0f, 1.923699323461407484359142472385428845882f, -0.996466733395733883504874484060565009713f,
		1.0f, 0.0f, -1.0f, 1.900982464016073514301297109341248869896f, -0.98867087671377917867943097007810138166f,
		1.0f, 0.0f, -1.0f, 1.916633874113788493787069455720484256744f, -0.989640645349980085931917983543826267123f,
		1.0f, 0.0f, -1.0f, 1.895866794152588408195470037753693759441f, -0.982145095737304463234806917171226814389f,
		1.0f, 0.0f, -1.0f, 1.909732012762139774508796108420938253403f, -0.983462195503834402110499013360822573304f,
		1.0f, 0.0f, -1.0f, 1.892497895658920459283081072499044239521f, -0.976990724305196844845511350285960361362f,
		1.0f, 0.0f, -1.0f, 1.90340090912507409193210605735657736659f,  -0.978301923848673538053333231800934299827f,
		1.0f, 0.0f, -1.0f, 1.891064709921030484451875963713973760605f, -0.973536598840808009569514069880824536085f,
		1.0f, 0.0f, -1.0f, 1.89803515862429872385064300033263862133f,  -0.974493538707266515608296231221174821258f,
		1.0f, 0.0f, -1.0f, 1.891600905883061845003112466656602919102f, -0.971965509756440582123104832135140895844f,
		1.0f, 0.0f, -1.0f, 1.893998891657468153226773210917599499226f, -0.972315235221609008853249633830273523927f
};
float32_t filter2k_state[4 * NUM_STAGES];
const float64_t scale_2k = 0.01422068585481763881850358899328057305f  *
						   0.01422068585481763881850358899328057305f  *
						   0.014170063934704357419547271490500861546f *
						   0.014170063934704357419547271490500861546f *
						   0.014125134218909861719137843749649618985f *
						   0.014125134218909861719137843749649618985f *
						   0.014088764505501822840716208418143651215f *
						   0.014088764505501822840716208418143651215f *
						   0.014063216137369865393891288363192870747f *
						   0.014063216137369865393891288363192870747f *
						   0.014050045804001367807933853271151747322f *
						   0.014050045804001367807933853271151747322f ;

const float32_t filter5k_coeffs[5 * NUM_STAGES] = {
		1.0f,  0.0f,  -1.0f,  1.492183689967542692400570558675099164248f,  -0.996227425714277470270019421150209382176f,
		1.0f,  0.0f,  -1.0f,  1.529138013514522276281581980583723634481f,  -0.996348630806060953091218834742903709412f,
		1.0f,  0.0f,  -1.0f,  1.488065181411297510649660580384079366922f,  -0.988991253176423223969493392360163852572f,
		1.0f,  0.0f,  -1.0f,  1.522542655552455492795616009971126914024f,  -0.989320058521498069126209884416311979294f,
		1.0f,  0.0f,  -1.0f,  1.485746310954767057666003893245942890644f,  -0.982580270232098484939342597499489784241f,
		1.0f,  0.0f,  -1.0f,  1.515384388849891905692857108078896999359f,  -0.983026630413579738387852557934820652008f,
		1.0f,  0.0f,  -1.0f,  1.485359735798303937315267830854281783104f,  -0.977424069887797175582022646267432719469f,
		1.0f,  0.0f,  -1.0f,  1.508121451890213959856623660016339272261f,  -0.97786818906537287254820967064006254077f,
		1.0f,  0.0f,  -1.0f,  1.486901102314397649095667475194204598665f,  -0.973852977588563373245733600924722850323f,
		1.0f,  0.0f,  -1.0f,  1.501218380867903823272513363917823880911f,  -0.974176951858358286528982716845348477364f,
		1.0f,  0.0f,  -1.0f,  1.490237206577873951829360521514900028706f,  -0.972081174516537416252504044678062200546f,
		1.0f,  0.0f,  -1.0f,  1.495122087977886593535004067234694957733f,  -0.972199542611393630053839842730667442083f
};
float32_t filter5k_state[4 * NUM_STAGES];
const float64_t scale_5k = 0.014220685854818063825755203311018703971f *
						   0.014220685854818063825755203311018703971f *
						   0.014170063934704777222628457877817709232f *
						   0.014170063934704777222628457877817709232f *
						   0.014125134218910279787495554160159372259f *
						   0.014125134218910279787495554160159372259f *
						   0.014088764505502239174350442851846310077f *
						   0.014088764505502239174350442851846310077f *
						   0.014063216137370279992802046820088435197f *
						   0.014063216137370279992802046820088435197f *
						   0.01405004580400178067212113575124021736f  *
						   0.01405004580400178067212113575124021736f  ;

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
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  lcd_init();
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, SAMPLE_SIZE);

  lcd_show_string(30, 90, 50, 16, 16, "500: ", RED);
  lcd_show_string(30, 110, 50, 16, 16, "2K: ", RED);
  lcd_show_string(30, 130, 50, 16, 16, "3K: ", RED);

  //filter_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(AdcConvEnd == 1 ) {
		  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

		  float32_t output_1k[SAMPLE_SIZE];		//1k滤波输出
		  float32_t output_2k[SAMPLE_SIZE];		//2k滤波输出
		  float32_t output_5k[SAMPLE_SIZE];		//5k滤波输出

		  float32_t mag1 = 0.0;
		  float32_t freq1 = 0.0;
		  float32_t mag2 = 0.0;
		  float32_t freq2 = 0.0;
		  float32_t mag5 = 0.0;
		  float32_t freq5 = 0.0;

		  for(int i = 0; i < SAMPLE_SIZE; i++) {
		  	  adc_float[i] = (float32_t)adc_buffer[i];
		  }

		  float32_t fft_output[SAMPLE_SIZE];
		  float32_t magnitude[SAMPLE_SIZE/2];
		  float32_t mag_4k = 0.0, mag_2k = 0.0, mag_3k = 0.0;

		  arm_rfft_fast_f32(&fft_instance, adc_float, fft_output, 0);
		  arm_cmplx_mag_f32(fft_output, magnitude, SAMPLE_SIZE/2);

		  mag_2k = ((magnitude[91] + magnitude[92] + magnitude[93]) * 2.0 / SAMPLE_SIZE);
		  mag_3k = ((magnitude[138] + magnitude[139] + magnitude[137]) * 2.0 / SAMPLE_SIZE);
		  mag_4k = ((magnitude[185] + magnitude[184] + magnitude[186]) * 2.0 / SAMPLE_SIZE);

		  char str_2k[50]; char str_3k[50]; char str_4k[50];
		  sprintf(str_2k, "%5.2f   2000 Hz", mag_2k);
		  sprintf(str_3k, "%5.2f   3000 Hz", mag_3k);
		  sprintf(str_4k, "%5.2f   4000 Hz", mag_4k);

		  lcd_clear(WHITE);
		  lcd_show_string(30, 90, 50, 16, 16, "2k: ", RED);
		  lcd_show_string(30, 110, 50, 16, 16, "3K: ", RED);
		  lcd_show_string(30, 130, 50, 16, 16, "4K: ", RED);
		  lcd_show_string(60, 90, 200, 16, 16, str_2k, BLUE);
		  lcd_show_string(60, 110, 200, 16, 16, str_3k, BLUE);
		  lcd_show_string(60, 130, 200, 16, 16, str_4k, BLUE);

		  HAL_Delay(100);

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
/*
@param: filter_in: 滤波器输入
@param: filter_out: 滤波器输出
@param: bolckSize: 滤波器单次处理的点数
@param: type: 使用的滤波器类型, 对于本项目 1 - filter_1k ; 2 - filter_2k ; 5 - filter_5k
*/
void filter_process(float32_t* filter_in, float32_t* filter_out, int blockSize, int type) {
	if(type == 1) {
		arm_biquad_cascade_df1_f32(&S_1k, filter_in, filter_out, blockSize);
		arm_scale_f32(filter_out, scale_1k, filter_out, blockSize);
	}
	else if(type == 2) {
		arm_biquad_cascade_df1_f32(&S_2k, filter_in, filter_out, blockSize);
		arm_scale_f32(filter_out, scale_2k, filter_out, blockSize);
	}
	else if(type == 5) {
		arm_biquad_cascade_df1_f32(&S_5k, filter_in, filter_out, blockSize);
		arm_scale_f32(filter_out, scale_5k, filter_out, blockSize);
	}
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
		input_signal[i] = (float32_t)((pwm1k + pwm2k + pwm5k) * 4096.f / 3.3f);
	}

	filter_process(input_signal, out_1k, SAMPLE_SIZE, 1);
	filter_process(input_signal, out_2k, SAMPLE_SIZE, 2);
	filter_process(input_signal, out_5k, SAMPLE_SIZE, 5);

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
