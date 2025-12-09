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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

// Структура ПИ-регулятора с механизмом антивиндапа (anti-windup)
typedef struct
{
    float dt;       // период дискретизации системы управления
    float Kp;       // коэффициент пропорциональной составляющей
    float Ki;       // коэффициент интегральной составляющей
    float y;        // выходное значение регулятора (ограниченное)
    float y_unl;    // выходное значение регулятора (неограниченное)
    float Int;      // текущее значение интегратора
    float Int_pr;   // предыдущее значение интегратора
    float err;      // текущая ошибка регулирования
    float u;        // задающее воздействие (вход регулятора)
    float Kaw;      // коэффициент компенсации антивиндапа
    float ErrSat;   // ошибка насыщения для компенсации
    float Ymax;     // максимальное ограничение выхода регулятора
    float Ymin;     // минимальное ограничение выхода регулятора
    float counter;  // счетчик
} Regulator;

// Структура апериодического звена первого порядка
typedef struct
{
    float dt;       // период дискретизации объекта регулирования
    float K;        // коэффициент усиления
    float T;        // постоянная времени
    float k1;       // коэффициент для дискретной реализации
    float k2;       // коэффициент для дискретной реализации
    float y;        // выходное значение звена
    float y_pr;     // предыдущее выходное значение
    float Z;        // внешнее возмущение
} Object;

// Структура интегратора
typedef struct
{
    float dt;       // период дискретизации объекта регулирования
    float K;        // коэффициент усиления
    float T;        // постоянная времени
    float k1;       // коэффициент для дискретной реализации
    float y;        // выходное значение интегратора
    float y_pr;     // предыдущее выходное значение
} Integrator;

// Структура функции насыщения
typedef struct
{
    float Ymax;     // верхний предел насыщения
    float Ymin;     // нижний предел насыщения
} Saturation;

// Объявление глобальных переменных
Regulator piP; // регулятор ПИ для контура мощности
Regulator piT; // регулятор ПИ для контура температуры
Object f1;     // первый объект (преобразователь напряжения)
Object f2;     // второй объект (элементы Пельтье)
Integrator F;  // третий объект (модель тепловой инерции)
Saturation S2; // функция насыщения для второго объекта
Saturation S4; // функция насыщения для третьего объекта


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_str()
{
	// Параметры первого объекта (преобразователь напряжения)
	f1.dt = 0.0001;
	f1.K = 6.74;
	f1.T = 0.0013;
	f1.k1 = (f1.dt * f1.K) / (f1.dt + f1.T);
	f1.k2 = f1.T / (f1.dt + f1.T);
	f1.y = 0.0;
	f1.y_pr = 0.0;

	// Параметры второго объекта (элементы Пельтье)
	f2.dt = f1.dt;
	f2.K = 3.12;
	f2.T = 0.0165;
	f2.k1 = (f2.dt * f2.K) / (f2.dt + f2.T);
	f2.k2 = f2.T / (f2.dt + f2.T);
	f2.y = 0.0;
	f2.y_pr = 0.0;
	S2.Ymax = 250;
	S2.Ymin = -250;
	f2.Z = 0.0;

	// Параметры третьего объекта (тепловая инерция)
	F.dt = f1.dt;
	F.K = 1.0;
	F.T = 0.207;
	F.k1 = (F.dt * F.K) / F.T;
	F.y = 0.0;
	F.y_pr = 0.0;
	S4.Ymax = 90.0;
	S4.Ymin = -90.0;

	// Параметры ПИ-регулятора для контура мощности
	piP.dt = 0.001;
	piP.Kp = f2.T / (2 * f1.T * f1.K * f2.K);
	piP.Ki = 1 / f2.T;
	piP.y = 0.0;
	piP.y_unl = 0.0;
	piP.Int = 0.0;
	piP.Int_pr = 0.0;
	piP.err = 0.0;
	piP.Kaw = 10.0;
	piP.ErrSat = 0.0;
	piP.Ymax = 3;
	piP.Ymin = -3;

	// Параметры ПИ-регулятора для контура температуры
	piT.dt = 0.001;
	piT.Kp = F.T / (8 * f1.T);
	piT.Ki = 1.0 / (8 * f1.T);
	piT.y = 0.0;
	piT.Int = 0.0;
	piT.Int_pr = 0.0;
	piT.err = 0.0;
	piT.u = 0.0;
	piT.counter = 0.0;
}

// Функция насыщения
float sat_func(Saturation* S, float x)
{
	if (S->Ymax < x)
	{
		return S->Ymax;
	}
	else if (S->Ymin > x)
	{
		return S->Ymin;
	}
	else
	{
		return x;
	}
}

// Функция ПИ-регулятора без антивиндапа
float reg_func(Regulator* pi, float err)
{
	// Вычисление нового значения интегральной составляющей
	pi->Int = pi->Int_pr + pi->Kp * err * pi->Ki * pi->dt;
	pi->Int_pr = pi->Int;

	// Выход регулятора
	pi->y = pi->Kp * err + pi->Int;

	return pi->y;
}

float reg_func_sat(Regulator* pi, float err)
{
	// Вычисление интегральной составляющей с компенсацией антивиндапа
	pi->Int = pi->Int_pr + pi->Kp * err * pi->Ki * pi->dt + pi->ErrSat * pi->Kaw * pi->Ki * pi->dt;
	pi->Int_pr = pi->Int;

	// Вычисление неограниченного выхода регулятора
	pi->y_unl = pi->Kp * err + pi->Int;

	// Применение функции насыщения и вычисление ошибки насыщения
	if (pi->y_unl > pi->Ymax) {
		pi->y = pi->Ymax;
		pi->ErrSat = pi->y - pi->y_unl;
	} else if (pi->y_unl < pi->Ymin) {
		pi->y = pi->Ymin;
		pi->ErrSat = pi->y - pi->y_unl;
	} else {
		pi->y = pi->y_unl;
		pi->ErrSat = 0;
	}

	return pi->y;
}

// Функция апериодического звена первого порядка
float obj_func(Object* f, float x)
{
	f->y = f->k1 * x + f->k2 * f->y_pr;

	f->y_pr = f->y;

    return f->y;
}

// Функция интегратора
float obj_int(Integrator* f, float x)
{
	f->y = f->y_pr + f->k1 * x;

	f->y_pr = f->y;

    return f->y;
}

// Обратный вызов по прерыванию от таймера
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	 // Обработка прерывания от TIM8 (1 кГц)
	if (htim->Instance == TIM8) {
		// Вычисление ошибки регулирования температуры
		piT.err = piT.u - F.y;
		// Вычисление выходного значения регулятора температуры
		piT.y = reg_func(&piT, piT.err);
		// Вычисление ошибки регулирования мощности
		piP.err = piT.y - f2.y;
		// Вычисление выходного значения регулятора мощности с антивиндапом
		piP.y = reg_func_sat(&piP, piP.err);
	}

	// Обработка прерывания от TIM10 (10 кГц)
	if (htim->Instance == TIM10) {
		// Установка уставки температуры (линейный вход)
		piT.counter++;
		piT.u = piT.counter / 100.0;

		// Установка уставки температуры (константа)
		//piT.u = 30.0;

		// Установка внешнего возмущения
		f2.Z = 17.0;
		// Вычисление результата первого апериодического звена
		f1.y = obj_func(&f1, piP.y);
		// Вычисление результата второго апериодического звена с функцией насыщения
		f2.y = sat_func(&S2, obj_func(&f2, f1.y));
		// Вычисление результата интегрального звена с функцией насыщения
		F.y = sat_func(&S4, obj_int(&F, f2.y - f2.Z));
   	}
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
  init_str();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
