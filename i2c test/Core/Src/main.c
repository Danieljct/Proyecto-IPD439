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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsl.h"
//#include "b_l475e_iot01a1_bus.h"
#include "stm32l4xx_nucleo_bus.h"
#include <stdio.h>
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
LSM6DSL_Object_t MotionSensor;
volatile uint32_t dataRdyIntReceived;
uint32_t time, difftime;
int timerflag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  dataRdyIntReceived = 0;
  MEMS_Init();
  printf("AccX AccY AccZ Time");
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // En tu bucle principal (ej. dentro de while(1))
	  LSM6DSL_AxesRaw_t axes_acc;
	  uint8_t drdy_status = 0;
	  int32_t ret_status; // Variable para guardar el código de retorno

	  // Esperar a que los datos del acelerómetro estén listos
	  // En el bucle principal, después de detectar DRDY
	  if(timerflag){
	      LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes_acc);
	      printf("%d %d %d %d\r\n", axes_acc.x, axes_acc.y, axes_acc.z, difftime);
	      timerflag = 0;
	  }
	  // Añadir delay para controlar la frecuencia de lectura general

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

int32_t SPI_Send(void *ignored_handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
    // Tamaño total de la transmisión SPI (1 byte de dirección + Length bytes de datos)
    uint16_t transfer_size = 1 + Length;
    int32_t ret = BSP_ERROR_NONE; // 0

    // Buffer local para la transmisión completa
    uint8_t tx_buffer[transfer_size];

    // Preparar comando de escritura (MSB=0 para escribir)
    tx_buffer[0] = 0x7F & Reg; // Pone a 0 el bit 7

    // Copiar los datos a escribir después del comando/dirección
    memcpy(&tx_buffer[1], pData, Length);
    printf("BSP_SPI3_Send: Escribiendo Reg=0x%02X, tx_buffer[0]=0x%02X\n", Reg, tx_buffer[0]);
    // Llamar a HAL para transmitir el buffer completo
    // Asumiendo que hspi3 es tu handle SPI global o accesible

    GPIOA->BSRR = GPIO_PIN_1 << 16; // Set pin to low
    if (HAL_SPI_Transmit(&hspi3, tx_buffer, transfer_size, BUS_SPI3_POLL_TIMEOUT*10) != HAL_OK)
    {

        // Error en la transmisión HAL
         printf("BSP_SPI3_Send: HAL_SPI_Transmit falló!\r\n");
         ret = BSP_ERROR_UNKNOWN_FAILURE; // O un código de error más específico
    }
    GPIOA->BSRR = GPIO_PIN_1; // Set pin to high
    // No necesitas recibir nada en una escritura pura (a menos que quieras verificar MISO)

    return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Pointer to data buffer to receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
// Asumiendo que BSP_SPI3_Recv recibe (..., uint8_t reg, uint8_t *data, uint16_t len)
int32_t SPI_Recv(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    // Tamaño total de la transferencia SPI
    uint16_t transfer_size = 1 + len;

    // Buffers locales (pueden ser estáticos si len tiene un máximo conocido y pequeño)
    uint8_t tx_buffer[transfer_size];
    uint8_t rx_buffer[transfer_size];

    // Preparar comando de lectura (Ej: MSB=1 para leer)
    tx_buffer[0] = 0x80 | reg;

    // Rellenar resto de Tx con dummy bytes (0x00 o 0xFF)
    for (uint16_t i = 1; i < transfer_size; i++) {
        tx_buffer[i] = 0x00; // O 0xFF
    }

    // Llamar a HAL con buffers válidos
    // Asumiendo que hspi3 es tu handle SPI global o accesible
    time = TIM2->CNT;
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIOA->BSRR = GPIO_PIN_1<< 16; // Set pin to low
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi3, tx_buffer, rx_buffer, transfer_size, HAL_MAX_DELAY); // Usa un timeout adecuado
    GPIOA->BSRR = GPIO_PIN_1 ; // Set pin to high
   //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
   difftime = TIM2->CNT - time;
    if (status == HAL_OK) {
        // Copiar los datos recibidos relevantes al buffer de destino
        memcpy(data, &rx_buffer[1], len);
        return 0; // Éxito (LSM6DSL_OK)
    } else {
        // Error en la comunicación HAL
        printf("BSP_SPI3_Recv: HAL_SPI_TransmitReceive falló con estado: %d\r\n", status);
        // Devolver un código de error. Podrías mapear HAL_ERROR, HAL_TIMEOUT etc. a errores específicos.
        if (status == HAL_TIMEOUT) {
             return -2; // Ejemplo para timeout
        } else {
             return -1; // Error genérico (LSM6DSL_ERROR)
        }
    }
}



static void MEMS_Init(void)
{
	LSM6DSL_IO_t io_ctx;
	uint8_t id;
	LSM6DSL_AxesRaw_t axes;


    /* Link SPI functions */
    io_ctx.BusType = LSM6DSL_SPI_4WIRES_BUS;
    io_ctx.Init = BSP_SPI3_Init;
    io_ctx.DeInit = BSP_SPI3_DeInit;
    io_ctx.ReadReg = SPI_Recv;
    io_ctx.WriteReg = SPI_Send;
    io_ctx.GetTick = BSP_GetTick;

	LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);
	/* Read the LSM6DSL WHO_AM_I register */
	LSM6DSL_ReadID(&MotionSensor, &id);
	if (id != LSM6DSL_ID) {
		Error_Handler();
	}
	/* Initialize the LSM6DSL sensor */
	LSM6DSL_Init(&MotionSensor);

	// En MEMS_Init, después de LSM6DSL_Init y ANTES de llamar a Enable
	uint8_t test_val_write = 0x38; // Valor de prueba (Ej: 26Hz, 4g)
	uint8_t test_val_read = 0;
	printf("Intentando escritura directa a CTRL1_XL (0x10) con valor 0x%02X\r\n", test_val_write);

	// Llama a tu función WriteReg (que usa BSP_SPI3_Send internamente)
	if (LSM6DSL_Write_Reg(&MotionSensor, LSM6DSL_CTRL1_XL, test_val_write) == LSM6DSL_OK) {
	    printf("LSM6DSL_Write_Reg reportó éxito.\r\n");
	    HAL_Delay(5); // Pequeña pausa

	    // Lee de vuelta usando tu función ReadReg (que usa BSP_SPI3_Recv)
	    if (LSM6DSL_Read_Reg(&MotionSensor, LSM6DSL_CTRL1_XL, &test_val_read) == LSM6DSL_OK) {
	         printf("Leído de vuelta CTRL1_XL = 0x%02X\r\n", test_val_read);
	         if (test_val_read == test_val_write) {
	             printf("¡ÉXITO en Escritura/Lectura Directa!\r\n");
	         } else {
	             printf("¡FALLO en Escritura/Lectura Directa! Valor no coincide.\r\n"); // <-- Probablemente aquí está el problema
	         }
	    } else {
	        printf("¡Error leyendo de vuelta CTRL1_XL!\r\n");
	    }
	} else {
	     printf("¡LSM6DSL_Write_Reg FALLÓ!\r\n"); // <-- O aquí si la escritura ya da error
	}
	// Detener aquí o continuar con el resto de la inicialización
	// while(1); // Puedes detener aquí para analizar

	/* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
	LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 6660); /* 26 Hz */
	LSM6DSL_ACC_SetFullScale(&MotionSensor, 4); /* [-4000mg; +4000mg] */
	LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE); /* Enable DRDY */
	LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes); /* Clear DRDY */
	/* Start the LSM6DSL accelerometer */
	LSM6DSL_ACC_Enable(&MotionSensor);



#if 0
    // Asume que MotionSensor es una variable global o estática de tipo LSM6DSL_Object_t
    if (LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx) != LSM6DSL_OK) {
        Error_Handler(); // Añade chequeos de error
    }

    /* Read WHO_AM_I */
    if (LSM6DSL_ReadID(&MotionSensor, &id) != LSM6DSL_OK) {
         Error_Handler();
    }
    //if (id != LSM6DSL_ID) {
    //    Error_Handler(); // Error si el ID no coincide
    //}

    /* Initialize the sensor (deja los sensores apagados) */
    if (LSM6DSL_Init(&MotionSensor) != LSM6DSL_OK) {
         Error_Handler();
    }


    /* Configure Gyroscope (¡Añadir si lo vas a usar!) */
    // if (LSM6DSL_GYRO_SetOutputDataRate(&MotionSensor, 104.0f) != LSM6DSL_OK) { Error_Handler(); }
    // if (LSM6DSL_GYRO_SetFullScale(&MotionSensor, 500) != LSM6DSL_OK) { Error_Handler(); }


    /* Enable Accelerometer (Ahora escribe el ODR guardado y lo enciende) */
    int32_t enable_ret = LSM6DSL_ACC_Enable(&MotionSensor);
    if (enable_ret != LSM6DSL_OK) {
        printf("¡¡ERROR: LSM6DSL_ACC_Enable falló con código %ld!!\r\n", enable_ret);
        Error_Handler();
    } else {
        printf("LSM6DSL_ACC_Enable reportó éxito.\r\n"); // Confirmar que se cree exitoso
    }

    // En MEMS_Init, justo después de verificar LSM6DSL_ACC_Enable
    uint8_t ctrl1_xl_val = 0;
    HAL_Delay(1); // Pequeña pausa
    if (LSM6DSL_Read_Reg(&MotionSensor, LSM6DSL_CTRL1_XL, &ctrl1_xl_val) == LSM6DSL_OK) {
        printf("MEMS_Init: CTRL1_XL leído DESPUÉS de Enable = 0x%02X\r\n", ctrl1_xl_val);
        // Para 26Hz (LSM6DSL_XL_ODR_26Hz = 0x03), los bits 7-4 deberían ser 0011.
        // Así que esperarías ver algo como 0x3? (ej: 0x30, 0x34, etc.)
    } else {
        printf("MEMS_Init: ¡Error leyendo CTRL1_XL después de Enable!\r\n");
    }
    HAL_Delay(50); // Pausa antes de entrar al bucle principal
    /* Enable Gyroscope (¡Añadir si lo configuraste!) */
    // if (LSM6DSL_GYRO_Enable(&MotionSensor) != LSM6DSL_OK) { Error_Handler(); }


    if ( LSM6DSL_ACC_Enable_6D_Orientation(&MotionSensor, 0) != LSM6DSL_OK) {
         Error_Handler();
    }

#endif
    // HAL_Delay(20); // Opcional: Pequeña pausa para asegurar la primera muestra
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) {
    dataRdyIntReceived++;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3){
		timerflag = 1;
	}

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
  while(1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    printf("no se conectó");
    HAL_Delay(50); /* wait 50 ms */
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
