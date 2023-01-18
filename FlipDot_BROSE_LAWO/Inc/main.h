/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define OnboardLED_Pin GPIO_PIN_13
#define OnboardLED_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_1
#define RS485_EN_GPIO_Port GPIOA
#define Lampe_Pin GPIO_PIN_2
#define Lampe_GPIO_Port GPIOA
#define Zeilen_SET_RESET_Pin GPIO_PIN_3
#define Zeilen_SET_RESET_GPIO_Port GPIOA
#define Zeilen_OE_Pin GPIO_PIN_4
#define Zeilen_OE_GPIO_Port GPIOA
#define Zeilen_CLK_Pin GPIO_PIN_5
#define Zeilen_CLK_GPIO_Port GPIOA
#define Zeilen_LATCH_Pin GPIO_PIN_6
#define Zeilen_LATCH_GPIO_Port GPIOA
#define Zeilen_D_Pin GPIO_PIN_7
#define Zeilen_D_GPIO_Port GPIOA
#define Test_Lampe_Pin GPIO_PIN_0
#define Test_Lampe_GPIO_Port GPIOB
#define Test_Schwarz_Pin GPIO_PIN_1
#define Test_Schwarz_GPIO_Port GPIOB
#define Test_Gelb_Pin GPIO_PIN_10
#define Test_Gelb_GPIO_Port GPIOB
#define Reset_Pin GPIO_PIN_11
#define Reset_GPIO_Port GPIOB
#define Spalten_OE_Pin GPIO_PIN_12
#define Spalten_OE_GPIO_Port GPIOB
#define Spalten_CLK_Pin GPIO_PIN_13
#define Spalten_CLK_GPIO_Port GPIOB
#define Spalten_LATCH_Pin GPIO_PIN_14
#define Spalten_LATCH_GPIO_Port GPIOB
#define Spalten_D_Pin GPIO_PIN_15
#define Spalten_D_GPIO_Port GPIOB
#define Spalten_RESET_Pin GPIO_PIN_8
#define Spalten_RESET_GPIO_Port GPIOA
#define SW_4_Pin GPIO_PIN_3
#define SW_4_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_4
#define SW_3_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_5
#define SW_2_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_6
#define SW_1_GPIO_Port GPIOB
#define LED_RX_Pin GPIO_PIN_7
#define LED_RX_GPIO_Port GPIOB
#define LED_Status_Pin GPIO_PIN_8
#define LED_Status_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
