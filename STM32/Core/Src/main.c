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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#define NUM_SAMPLES 20

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

/* USER CODE BEGIN PV */
typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PID_Controller;

PID_Controller pid1 = {1.0f, 0.001f, 1.0f, 0, 0}; // Тюнинг подбирается экспериментально
PID_Controller pid2 = {5.0f, 0.001f, 1.0f, 0, 0};

volatile bool input_mode = false;
uint32_t adc_value_ch2 = 0;
volatile uint32_t adc_values[4] = {0,0,0,0};
float smooth_adc[4] = {0};
volatile uint32_t adc_raw_avg[4] = {0};  // усреднённые значения
volatile uint32_t last_button_press = 0;
float angle_pot1, angle_pot2;
float angle_enc1, angle_enc2;
volatile float sine_angle_2 = 90;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float GenerateSineValue(float freq_hz, float min_val, float max_val)
{
    static float time_ms = 0.0f;

    // Амплитуда и смещение
    float amplitude = (max_val - min_val) / 2.0f;
    float offset = (max_val + min_val) / 2.0f;

    // Вычисление значения
    float radians = 2.0f * M_PI * freq_hz * (time_ms / 1000.0f);
    float sine_value = offset + amplitude * sinf(radians);

    // Увеличение времени
    time_ms += 1.0f;
    if (time_ms >= 1000.0f / freq_hz) {
        time_ms -= 1000.0f / freq_hz;  // сброс по периоду
    }

    return sine_value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
    	sine_angle_2 = GenerateSineValue(9.0f, 10.0f, 330.0f);
    }
}

float PID_Compute(PID_Controller* pid, float setpoint, float measured)
{
    float error = setpoint - measured;

    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;



    return output;
}


// Мотор 1: PWM на PB13, направление на PA9
void SetDirection1(bool cw)
{
	cw = !cw;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Мотор 2: PWM на PB7, направление на PA10
void SetDirection2(bool cw)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Установка Ш�?М — ожидается значение от 0 до 4095
void PWM_Motor1(uint16_t pwm_value)
{
    if (pwm_value > 839) pwm_value = 839;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value); // таймер для PB13
}

void PWM_Motor2(uint16_t pwm_value)
{
    if (pwm_value > 839) pwm_value = 839;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_value); // таймер для PB7
}


void ControlMotor1(float target_angle)
{
    float pwm = PID_Compute(&pid1, target_angle, angle_enc2);

    if (pwm >= 0) {
        SetDirection1(true);  // CW
    } else {
        SetDirection1(false); // CCW
        pwm = -pwm;
    }

    if (pwm > 839.0f) pwm = 839.0f;
    PWM_Motor1((uint16_t)pwm);
}

void ControlMotor2(float target_angle)
{
    float pwm = PID_Compute(&pid2, target_angle, angle_enc1);

    if (pwm >= 0) {
        SetDirection2(true);  // CW
    } else {
        SetDirection2(false); // CCW
        pwm = -pwm;
    }

    if (pwm > 839.0f) pwm = 839.0f;
    PWM_Motor2((uint16_t)pwm);
}


void LowPassFilterADC(uint32_t adc_input[4])
{
    const float alpha = 0.1;  // от 0 до 1 (чем меньше — сильнее фильтр)
    for (int i = 0; i < 4; i++) {
        smooth_adc[i] = smooth_adc[i] * (1 - alpha) + adc_input[i] * alpha;
    }
}

void FilterADC(volatile uint32_t adc_input[4])
{
    static uint32_t adc_sum[4] = {0};
    static uint8_t sample_count = 0;

    for (int i = 0; i < 4; i++) {
        adc_sum[i] += adc_input[i];
    }

    sample_count++;

    if (sample_count >= NUM_SAMPLES) {
        for (int i = 0; i < 4; i++) {
            adc_raw_avg[i] = adc_sum[i] / NUM_SAMPLES;
            adc_sum[i] = 0; // сброс суммы
        }
        sample_count = 0;
    }
}


// Вход: adc_values[0] – [3]
// Выход: 4 угла в градусах
void ConvertADCToAngles(uint32_t adc_values[4], float* angle_pot1, float* angle_pot2,
                        float* angle_enc1, float* angle_enc2)
{
    // ——— Энкодер 1: диапазон 3885 шагов ≈ 337°
    const uint16_t ENC1_ZERO_POS = 2970-15;
    const uint16_t ENC1_STEPS = 3835+15;
    const float ENC1_MAX_DEGREES = 337.0f;

    // ——— Энкодер 2: диапазон 2088 шагов ≈ 183°
    const uint16_t ENC2_ZERO_POS = 4060-15;
    const uint16_t ENC2_STEPS = 2088+15;
    const float ENC2_MAX_DEGREES = 183.0f;

    // ——— Энкодер 1
    uint32_t raw_enc1 = adc_values[2];
    int32_t diff1 = (int32_t)raw_enc1 - ENC1_ZERO_POS;
    if (diff1 < 0) diff1 += 4096;
    if (diff1 > ENC1_STEPS) diff1 = ENC1_STEPS;  // ограничим
    *angle_enc1 = ((float)diff1 / ENC1_STEPS) * ENC1_MAX_DEGREES;

    // ——— Энкодер 2
    uint32_t raw_enc2 = adc_values[3];
    int32_t diff2 = (int32_t)raw_enc2 - ENC2_ZERO_POS;
    if (diff2 < 0) diff2 += 4096;
    if (diff2 > ENC2_STEPS) diff2 = ENC2_STEPS;
    *angle_enc2 = ((float)diff2 / ENC2_STEPS) * ENC2_MAX_DEGREES;

    // ——— Потенциометры по масштабу энкодеров
    float angle_pot1_tmp = adc_values[0] * (ENC1_MAX_DEGREES / 4095.0f);
	float angle_pot2_tmp = adc_values[1] * (ENC2_MAX_DEGREES / 4095.0f);


    if (angle_pot1_tmp < 10.0f){
    	*angle_pot1 = 10.0f;
    }
    else if (angle_pot1_tmp > 330.0f){
    	*angle_pot1 = 330.0f;
    }
    else *angle_pot1 = angle_pot1_tmp;


    if (angle_pot2_tmp < 8.0f){
        	*angle_pot2 = 8.0f;
    }
    else if (angle_pot2_tmp > 179.0f){
        	*angle_pot2 = 179.0f;
    }
    else *angle_pot2 = angle_pot2_tmp;


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_4)
	  {
	    uint32_t current_time = HAL_GetTick();

	    if (current_time - last_button_press > 200) // 100 мс защита от дребезга
	    {
	    	input_mode = !input_mode;
	      last_button_press = current_time;
	    }
	  }


}

void AutoTunePID(PID_Controller* pid, float (*readAngle)(void), void (*commandMotor)(float))
{
    float ku = 0.0f;         // Критический коэффициент усиления
    float tu = 0.0f;         // Критический период
    float target = 90.0f;    // Угол, на который будем подавать импульс
    float last_angle = 0.0f;
    float max_angle = -1000.0f;
    float min_angle = 1000.0f;

    uint32_t start_time = HAL_GetTick();
    uint32_t last_cross = start_time;
    int cross_count = 0;
    bool going_up = true;

    // Подадим импульс и будем смотреть на осцилляции
    float test_kp = 5.0f; // начнем с усиления
    pid->kp = test_kp;
    pid->ki = 0.0f;
    pid->kd = 0.0f;

    while (cross_count < 10 && HAL_GetTick() - start_time < 10000) // до 10 пересечений центра
    {
        float angle = readAngle();
        float pwm = PID_Compute(pid, target, angle);
        commandMotor(pwm);

        if ((going_up && angle < target) || (!going_up && angle > target)) {
            going_up = !going_up;
            cross_count++;

            uint32_t now = HAL_GetTick();
            if (cross_count == 6) {
                tu = (now - last_cross) / 1000.0f; // в секундах
                last_cross = now;
            }

            last_cross = now;
        }

        if (angle > max_angle) max_angle = angle;
        if (angle < min_angle) min_angle = angle;

        HAL_Delay(10);
    }

    ku = test_kp;

    // Применим настройки Ziegler-Nichols для "Pessen Integral Rule"
    pid->kp = 0.7f * ku;
    pid->ki = 2.5f * pid->kp / tu;
    pid->kd = 0.15f * pid->kp * tu;
}

float ReadAngleMotor2(void) {
    return angle_enc1;
}

void CommandMotor2(float pwm) {
    if (pwm >= 0) {
        SetDirection2(true);
    } else {
        SetDirection2(false);
        pwm = -pwm;
    }
    PWM_Motor2((uint16_t)pwm);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  HAL_TIM_Base_Start_IT(&htim3);

  //FilterADC(adc_values);
  //ConvertADCToAngles(adc_raw_avg, &angle_pot1, &angle_pot2, &angle_enc1, &angle_enc2);
  //AutoTunePID(&pid2, ReadAngleMotor2, CommandMotor2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  FilterADC(adc_values);
	  ConvertADCToAngles(adc_raw_avg, &angle_pot1, &angle_pot2, &angle_enc1, &angle_enc2);


	  if (input_mode) // Если true то используем потенциометры для управления
	      {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // Включить LED PB12
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // Выключить LED PB3
	        ControlMotor2(angle_pot1);
	        ControlMotor1(angle_pot2);
	        //PWM_Motor2(500);
	        //SetDirection2(false); //против часовой

	      }
	      else // Если false то используем заготовленный сигнал sin
	      {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Выключить LED PB12
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);    // Включить LED PB3
	        ControlMotor2(sine_angle_2);
	        //PWM_Motor2(500);
	        //SetDirection2(true);
	      }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
