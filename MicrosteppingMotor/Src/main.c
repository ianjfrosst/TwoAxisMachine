/**
 * @file main.c
 * @author STMicroelectronics
 * @brief Main program code
 *
 */

#include <stdlib.h>
#include <stdbool.h>
#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"

/* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);
uint16_t Read_ADC(void);

/* Limit switch pin defines: Axis 0/1, Limit 0/1 */
#define AX0_L0_PIN    GPIO_PIN_8
#define AX0_L0_PORT   GPIOA
#define AX0_L1_PIN    GPIO_PIN_6
#define AX0_L1_PORT   GPIOB
#define AX1_L0_PIN    GPIO_PIN_7
#define AX1_L0_PORT   GPIOC
#define AX1_L1_PIN    GPIO_PIN_9
#define AX1_L1_PORT   GPIOA

#define X_STEP_PER_CM (26)
#define Y_STEP_PER_CM (945)

#define MAX_SPEED (26840)

#define SPEED_SCALE (67)

bool updated = true;
eL6470_DirId_t ax0_dir = L6470_DIR_FWD_ID;
eL6470_DirId_t ax1_dir = L6470_DIR_FWD_ID;

/**
 * @brief EXTI9_5 IRQ Handler
 *
 */
void EXTI9_5_IRQHandler(void) {
  uint32_t flags =
      __HAL_GPIO_EXTI_GET_IT(AX0_L0_PIN | AX0_L1_PIN | AX1_L0_PIN | AX1_L1_PIN);

  if (flags != 0) {
    if (HAL_GPIO_ReadPin(AX0_L0_PORT, AX0_L0_PIN)) {
      // BSP_L6470_HardStop(0, 0);
      ax0_dir = L6470_DIR_REV_ID;
    }

    if (HAL_GPIO_ReadPin(AX0_L1_PORT, AX0_L1_PIN)) {
      // BSP_L6470_HardStop(0, 0);
      ax0_dir = L6470_DIR_FWD_ID;
    }

    if (HAL_GPIO_ReadPin(AX1_L0_PORT, AX1_L0_PIN)) {
      // BSP_L6470_HardStop(0, 1);
      ax1_dir = L6470_DIR_REV_ID;
    }

    if (HAL_GPIO_ReadPin(AX1_L1_PORT, AX1_L1_PIN)) {
      // BSP_L6470_HardStop(0, 1);
      ax1_dir = L6470_DIR_FWD_ID;
    }

    updated = true;

    __HAL_GPIO_EXTI_CLEAR_IT(flags);
  }
}

/* Initialize the GPIO for the TwoAxis limit switches */
void TwoAxis_Init(void) {
  GPIO_InitTypeDef gpio = {0};

  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLUP;

  gpio.Pin = AX0_L0_PIN;
  HAL_GPIO_Init(AX0_L0_PORT, &gpio);

  gpio.Pin = AX0_L1_PIN;
  HAL_GPIO_Init(AX0_L1_PORT, &gpio);

  gpio.Pin = AX1_L0_PIN;
  HAL_GPIO_Init(AX1_L0_PORT, &gpio);

  gpio.Pin = AX1_L1_PIN;
  HAL_GPIO_Init(AX1_L1_PORT, &gpio);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// #define RUN_USART_APP

/**
 * @brief The FW main module
 */
int main(void) {
  /* NUCLEO board initialization */
  /* Init for UART, ADC, GPIO and SPI */
  NUCLEO_Board_Init();

  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();

  TwoAxis_Init();

  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();

  /*Initialize the motor parameters */
  Motor_Param_Reg_Init();

  // BSP_L6470_Run(0, 0, ax0_dir, MAX_SPEED / 3);
  // BSP_L6470_Run(0, 1, ax1_dir, MAX_SPEED);

  /* Infinite loop */
  while (1) {
#ifdef RUN_USART_APP
    /* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
#else

    HAL_ADC_Start(&HADC);
    HAL_ADC_PollForConversion(&HADC, 100);
    uint16_t adc0 = HAL_ADC_GetValue(&HADC);
    HAL_ADC_PollForConversion(&HADC, 100);
    uint16_t adc1 = HAL_ADC_GetValue(&HADC);

    USART_Transmit(&huart2, " ADC Read 0: ");
    USART_Transmit(&huart2, num2hex(adc0, WORD_F));
    USART_Transmit(&huart2, " 1: ");
    USART_Transmit(&huart2, num2hex(adc1, WORD_F));
    USART_Transmit(&huart2, " \n\r");

    // if (updated) {
    //   BSP_L6470_Run(0, 0, ax0_dir, MAX_SPEED / 3);
    //   // BSP_L6470_Run(0, 1, ax1_dir, MAX_SPEED);
    //   updated = false;
    // }

#endif
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
 * @brief  This function return the ADC conversion result.
 * @retval The number into the range [0, 4095] as [0, 3.3]V.
 */
uint16_t Read_ADC(void) {
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);

  return HAL_ADC_GetValue(&HADC);
}
