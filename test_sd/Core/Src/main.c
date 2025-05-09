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
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <math.h>   // Para sin() y M_PI
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t time, difftime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
FRESULT generate_sine_to_csv(const char* filename, double amplitude, double frequency, double duration, int num_points);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  time = TIM2->CNT;
  FATFS MiFatFs; // Objeto del sistema de archivos
     char MiSdPath[4]; // Ruta ej: "0:/"

     if (f_mount(&MiFatFs, MiSdPath, 1) == FR_OK) {
  	   myprintf("SD montada. Procediendo a generar CSV...\r\n");

         // Llamar a la función para generar el archivo
         FRESULT res = generate_sine_to_csv(
                             "0:/sin1.csv", // Nombre del archivo en la SD
                             5.0,                // Amplitud = 5.0
                             2.0,                // Frecuencia = 2 Hz (2 ciclos por segundo)
                             3.0,                // Duración = 3 segundos
                             150                 // Número de puntos = 150 (generará 150 puntos en 3 seg)
                         );

         if (res == FR_OK) {
      	   myprintf("Archivo CSV generado con éxito.\r\n");
      	   difftime = TIM2->CNT - time;
           myprintf("Tiempo de ejecución: %f ms\r\n", difftime/80000000.0);
         } else {
      	   myprintf("Fallo al generar el archivo CSV.\r\n");
         }

         // Opcional: desmontar la unidad si ya no se necesita
         f_mount(NULL, MiSdPath, 0);

     } else {
  	   myprintf("Error al montar la SD Card.\r\n");

     }


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Genera datos de una onda sinusoidal y los guarda en un archivo CSV.
 * @param filename Nombre del archivo CSV a crear (ej. "sine_data.csv" o "0:/sine_data.csv").
 * @param amplitude Amplitud de la onda sinusoidal.
 * @param frequency Frecuencia de la onda en Hz.
 * @param duration Duración total en segundos para generar los puntos.
 * @param num_points Número total de puntos a generar en esa duración.
 * @retval FR_OK si todo fue exitoso, otro código de error de FatFs en caso contrario.
 */
FRESULT generate_sine_to_csv(const char* filename, double amplitude, double frequency, double duration, int num_points) {
    FIL fil;         // Objeto de archivo FatFs
    FRESULT fr;      // Variable para códigos de retorno de FatFs
    UINT bytes_written; // Variable para almacenar bytes escritos por f_write
    char data_buffer[80]; // Buffer para formatear cada línea del CSV, ajusta tamaño si es necesario

    // --- 1. Abrir/Crear el archivo en modo escritura ---
    // FA_CREATE_ALWAYS: Crea un archivo nuevo. Si ya existe, lo sobrescribe.
    // FA_WRITE: Permiso de escritura.
    fr = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("Error: No se pudo abrir/crear el archivo '%s'. Codigo FatFs: %d\r\n", filename, fr);
        return fr;
    }

    // --- 2. Escribir la cabecera del CSV (opcional pero recomendado) ---
    // Usamos snprintf para seguridad contra desbordamiento de buffer
    snprintf(data_buffer, sizeof(data_buffer), "Tiempo (s),Valor\n");
    // Usamos f_write para escribir el buffer formateado
    fr = f_write(&fil, data_buffer, strlen(data_buffer), &bytes_written);
    if (fr != FR_OK || bytes_written < strlen(data_buffer)) {
        printf("Error: No se pudo escribir la cabecera en '%s'. Codigo FatFs: %d\r\n", filename, fr);
        f_close(&fil); // Intentar cerrar el archivo aunque haya error
        return fr;
    }

    // --- 3. Generar puntos y escribir en el archivo ---
    printf("Generando %d puntos sinusoidales...\r\n", num_points);
    for (int i = 0; i < num_points; ++i) {
        // Calcular el tiempo actual 't'
        // Asegura división flotante para obtener pasos de tiempo correctos
        // Se usa (num_points - 1) para que el último punto coincida exactamente con 'duration'
        double t = (double)i * duration / (double)(num_points > 1 ? num_points - 1 : 1);

        // Calcular el valor de la sinusoide y(t) = A * sin(2 * pi * f * t)
        double value = amplitude * sin(2.0 * M_PI * frequency * t);

        // Formatear la línea de datos como "tiempo,valor\n"
        // Ajusta "%.4f" para la precisión deseada
        int len = snprintf(data_buffer, sizeof(data_buffer), "%.4f,%.4f\n", t, value);

        // Verificar si snprintf truncó la salida (opcional pero buena práctica)
         if (len < 0 || len >= sizeof(data_buffer)) {
             printf("Advertencia: Buffer insuficiente al formatear punto %d\r\n", i);
             // Considerar cómo manejar esto: continuar, parar, etc.
         }

        // Escribir la línea formateada en el archivo
        fr = f_write(&fil, data_buffer, strlen(data_buffer), &bytes_written);
        if (fr != FR_OK || bytes_written < strlen(data_buffer)) {
            printf("Error: No se pudo escribir el punto %d en '%s'. Codigo FatFs: %d\r\n", i, filename, fr);
            f_close(&fil); // Intentar cerrar
            return fr;
        }
    }

    // --- 4. Cerrar el archivo ---
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("Error: No se pudo cerrar el archivo '%s'. Codigo FatFs: %d\r\n", filename, fr);
        return fr;
    }

    printf("Datos sinusoidales guardados exitosamente en '%s'\r\n", filename);
    return FR_OK; // Indicar éxito
}

int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
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
