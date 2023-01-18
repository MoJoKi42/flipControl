
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "delay_own.h"
#include "ringbuffer.h"
#include "fnt8x8.h"
#include "6x8_vertikal_LSB_1.h"

//    ______ _ _        _____            _             _ 
//   |  ____| (_)      / ____|          | |           | |
//   | |__  | |_ _ __ | |     ___  _ __ | |_ _ __ ___ | |
//   |  __| | | | '_ \| |    / _ \| '_ \| __| '__/ _ \| |
//   | |    | | | |_) | |___| (_) | | | | |_| | | (_) | |
//   |_|    |_|_| .__/ \_____\___/|_| |_|\__|_|  \___/|_|
//              | |                                      
//              |_|   written by Moritz Kimmig
//                    Version 1.2 (August 2022)
//
//////////////////////////////////////////////////////////
// Changelog:
//
//    V1.0
//      - Initaler Code
//
//    V1.1
//      - Zweite (schmalere) Schriftart implmentiert.
//      - Befehlssatz umgebaut, statt fixer Befehlslänge schließt
//        nun ein Return (0x0A) den jeweiligen Befehl ab
//      - Fehlender PullUp bei SW1-4 aktiviert
//      - "fast_refresh" eingebaut. Wenn dies aktiviert ist, dann
//        wird ein Dot nur noch gesetzt, wenn sich der Zustand vom
//        letzten mal davor geändert hat. Diese Funktion liegt nun
//        auf Schalter SW3.
//      - paar unbenutze Sachen rausgeworfen
//
//    V1.2
//      - Ringpuffer eingebaut und die Befehlsverarbeitung aus dem
//        UART Interrupt entfernt. Desweiteren den Befehlssatz auf
//        lesbare und sinnvolle Befehle umgebaut.
//        Alle Befehle müssen weiterhin mit einem \n abgeschlossen werden.
//      - Textoption für left/center/right, wie auch manuellen Offset eingebaut
//      - Bugfix beim UART Reset
//      - Option für manuelles Refreshen eingefügt (Siehe Schalter 4)
//
//
//    To Do:
//      - Overflow Bug lösen?
//      - erstes/letztes byte von schrift beinhaltet manchmal keine pixel -< beheben!
//


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//////////////////////////////////////////////////////
// Grundlegende Einstellungen
//////////////////////////////////////////////////////

// BROSE oder LAWO Panel (Änderbar über Onboard Schalter)
// 0 = LAWO
// 1 = BROSE
uint8_t Paneltyp;

// Anzahl Panels (Änderbar über Onboard Schalter)
uint8_t Panelanzahl;





//////////////////////////////////////////////////////
// Variablen
//////////////////////////////////////////////////////

// Pixelspeicher
uint32_t* dots;
uint32_t* dots_old;

// SPI Handler
SPI_HandleTypeDef hspi1;                // SPI_1 (Zeilen ULN2803, UDN2981)
SPI_HandleTypeDef hspi2;                // SPI_2 (Modulauswahl, Daten + Adressleitungen FP2800)
uint8_t hspi2_data[2] = {0x00, 0x00};   // SPI_2 Zwischenspeicher

// UART
uint8_t  uart_puffer = 0;                     // UART Puffer für ein Byte
uint32_t uart_last_time = 0;                  // Zeitmerker, wann letztes Byte kam
#define cmd_buffer_size 100                   // max. Befehlslänge
unsigned char cmd_buffer[cmd_buffer_size];    // Befehlszwischenspeicher

// Text
#define max_text_size 200                     // Puffergröße für Texterzeugung (Pixeldaten)
#define max_cmd_args 10                       // maximale Anzahl an Argumenten pro Befehl
uint8_t textbuffer_8px_UP[max_text_size];     // Zwischenspeicher Pixeldaten
uint8_t textbuffer_8px_DOWN[max_text_size];   // Zwischenspeicher Pixeldaten



// Optionen und Merker
uint8_t enable_fast_refresh = 0;        // Setze nur Dots, welche sich ändern -> schneller! (über Schalter 3 einstellbar!)
uint8_t enable_manually_refresh = 0;    // Nur nach Refresh-Befehl Dots schreiben (Ausnahme: Gelb/Schwarztest) (über Schalter 4 einstellbar!)
uint8_t yellow_test_or_center = 0;
uint8_t yellow_test_or_UP_DOWN = 0;
uint8_t start_refresh = 0;              // Vormerker: Pixeldaten ausgeben
uint8_t start_refresh_center = 0;       // Vormerker: Center Text ausgeben
uint8_t start_refresh_UP = 0;           // Vormerker: Top Text ausgeben
uint8_t start_refresh_DOWN = 0;         // Vormerker: Bottom Text ausgeben
uint8_t start_black = 0;                // Vormerker: Schwarzttest
uint8_t start_yellow = 0;               // Vormerker: Gelbtest


// ENUM für Befehle
enum {
  CMD_ERROR, // error
  CMD_LIGHT,
  CMD_SET_ALL,
  CMD_SET_BUFF,
  CMD_REFRESH,
  CMD_SET_DOT,
  CMD_TEXT_BOTTOM,
  CMD_TEXT_TOP,
  CMD_TEXT_CENTER
};



/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Hardware Funktionen
void Zeile_aktivieren(uint8_t Zeile, uint8_t SET_RESET);
void Spalte_aktivieren(uint8_t Spalte, uint8_t SET_RESET);
void Modul_aktivieren(uint8_t Modul, uint8_t ON_OFF);
void Write_Dots(uint8_t from_x, uint8_t to_x, uint8_t from_y, uint8_t to_y);

// Software Funktionen
int generate_text(uint8_t* text_in, uint8_t* text_out, int start, int end, uint8_t text_selector);
uint8_t bit_not_changed(uint8_t x, uint8_t y);
void save_written_dots(void);
int Run_CMD(unsigned char* command);
int isBusy();


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  //////////////////////////////////////////////////////
  // Basiseinstellungen abfragen
  //////////////////////////////////////////////////////

  // Switch 1
  // ---------
  #define LAWO 0
  #define BROSE 1
  if(!HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin)) {
    Paneltyp = BROSE; // ON
  } else {
    Paneltyp = LAWO; // OFF
  }

  // Switch 2
  // ---------
  // 0 = 3 Panels (84px  Breite)
  // 1 = 4 Panels (112px Breite)
  if(!HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin)) {
    Panelanzahl = 4; // ON (Brose)
  } else {
    Panelanzahl = 3; // OFF (Lawo)
  }

  // Switch 3
  // ---------
  // 0 = fast_refresh OFF
  // 1 = fast_refresh ON
  if(!HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin)) {
    enable_fast_refresh = 1;
  } else {
    enable_fast_refresh = 0;
  }

  // Switch 4
  // ---------
  // 0 = deaktiviere manuelles Refreshen
  // 1 =   aktiviere manuelles Refreshen
  if(!HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin)) {
    enable_manually_refresh = 1;
  } else {
    enable_manually_refresh = 0;
  }



  //////////////////////////////////////////////////////
  // Speicher initalisieren
  //////////////////////////////////////////////////////

  // Initalzustände aktivieren
  HAL_GPIO_WritePin(Spalten_RESET_GPIO_Port, Spalten_RESET_Pin, 1);
  HAL_GPIO_WritePin(Spalten_OE_GPIO_Port, Spalten_OE_Pin, 0);

  // Pixelspeicher
  dots = (uint32_t*)malloc(Panelanzahl * 28 * sizeof(uint32_t));
  if (dots == NULL) {
    while (1) {

      // Bei Speicherfehler -> blinken
      HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, 1);
      HAL_Delay(500);
      HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, 0);
      HAL_Delay(500);
    }
  }

  // Pixelspeicher (letzter Zustand)
  dots_old = (uint32_t*)malloc(Panelanzahl * 28 * sizeof(uint32_t));
  if (dots_old == NULL) {
    while (1) {

      // Bei Speicherfehler -> blinken
      HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, 1);
      HAL_Delay(500);
      HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, 0);
      HAL_Delay(500);
    }
  }

  // Pixelspeicher initalisieren
  for (uint8_t i=0; i < Panelanzahl * 28; i++) {
    dots[i] = 0;
    dots_old[i] = 0;
  }

  // Ringbuffer initalisieren
  ringbuffer_init();

  




  //////////////////////////////////////////////////////
  // Interrupts initalisieren
  //////////////////////////////////////////////////////

  // UART1 Interrupt aktivieren
  HAL_UART_Receive_IT(&huart1, &uart_puffer, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Timer2 Interrupt aktivieren (aktuell nicht in Verwendung!!)
  // HAL_TIM_Base_Start_IT(&htim2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  //////////////////////////////////////////////////////
  // Taster u. Variablen Abfragen durchführen
  //////////////////////////////////////////////////////

  // Taster Abfrage: Gelb-Test
  if (!HAL_GPIO_ReadPin(Test_Gelb_GPIO_Port, Test_Gelb_Pin) || start_yellow) {
    uint8_t Zeilen = 0;
    uint8_t enable_fast_refresh_current = enable_fast_refresh;
    enable_fast_refresh = 0; // disable fast_refresh for bl/yl test
    for (uint8_t i=0; i < Panelanzahl * 28; i++)
        dots[i] = 0xFFFFFFFF;
    if (Paneltyp == LAWO)  { Zeilen = 16; }
    if (Paneltyp == BROSE) { Zeilen = 19; }
    Write_Dots(0, (Panelanzahl * 28) - 1, 0, Zeilen - 1);
    enable_fast_refresh = enable_fast_refresh_current;
    yellow_test_or_center = 1;
    yellow_test_or_UP_DOWN = 1;
    start_yellow = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }

  // Taster Abfrage: Schwarz-Test
  if (!HAL_GPIO_ReadPin(Test_Schwarz_GPIO_Port, Test_Schwarz_Pin) || start_black) {
    uint8_t Zeilen = 0;
    uint8_t enable_fast_refresh_current = enable_fast_refresh;
    enable_fast_refresh = 0; // disable fast_refresh for bl/yl test
    for (uint8_t i=0; i < Panelanzahl * 28; i++)
        dots[i] = 0x00000000;
    if (Paneltyp == LAWO)  { Zeilen = 16; }
    if (Paneltyp == BROSE) { Zeilen = 19; }
    Write_Dots(0, (Panelanzahl * 28) - 1, 0, Zeilen - 1);
    enable_fast_refresh = enable_fast_refresh_current;
    yellow_test_or_center = 0;
    yellow_test_or_UP_DOWN = 0;
    start_black = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }

  // Taster Abfrage: Lampen-Test
  if (!HAL_GPIO_ReadPin(Test_Lampe_GPIO_Port, Test_Lampe_Pin)) {
    static uint32_t blocked_until = 0;
    if (HAL_GetTick() > blocked_until) {
      if (HAL_GPIO_ReadPin(Lampe_GPIO_Port, Lampe_Pin)) {
        HAL_GPIO_WritePin(Lampe_GPIO_Port, Lampe_Pin, 0);
      } else {
        HAL_GPIO_WritePin(Lampe_GPIO_Port, Lampe_Pin, 1);
      }
      blocked_until = HAL_GetTick() + 400;
    }
  }


  //////////////////////////////////////////////////////
  // Anzeigeninhalt aktualisieren
  //////////////////////////////////////////////////////
  if (start_refresh) {
    uint8_t anzahl_spalten = 20;
    if (Paneltyp == LAWO)  { anzahl_spalten=16; }
    if (Paneltyp == BROSE) { anzahl_spalten=19; }
    Write_Dots(0, (Panelanzahl * 28) - 1, 0, anzahl_spalten - 1);
    start_refresh = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }




  ////////////////////////////////////////////////////////////
  // Anzeigeninhalt aktualisieren (Text)
  ////////////////////////////////////////////////////////////
  // Text mittig
  if (start_refresh_center) {
    Write_Dots(0, (Panelanzahl * 28) - 1, 4, 11);
    start_refresh_center = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }

  // Text oben
  if (start_refresh_UP) {
    uint8_t offset = 0;
    if (Paneltyp == LAWO)  { offset=0; }
    if (Paneltyp == BROSE) { offset=0; }
    Write_Dots(0, (Panelanzahl * 28) - 1, 0, 7+offset);
    start_refresh_UP = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }

  // Text unten
  if (start_refresh_DOWN) {
    uint8_t anzahl_spalten = 20;
    if (Paneltyp == LAWO)  { anzahl_spalten=16; }
    if (Paneltyp == BROSE) { anzahl_spalten=19; }
    Write_Dots(0, (Panelanzahl * 28) - 1, anzahl_spalten - 8, anzahl_spalten - 1);
    start_refresh_DOWN = 0;

    // Speichere aktuellen Zustand
    save_written_dots();
  }


  //////////////////////////////////////////////////////
  // UART Buffer auslesen und Befehle starten
  //////////////////////////////////////////////////////
  unsigned char _temp_char = ringbuffer_get();
  static uint32_t cmd_buffer_pos = 0;
  while (_temp_char) {

    // Aktuelles Zeichen in Befehlspuffer laden
    cmd_buffer[cmd_buffer_pos] = _temp_char;
    cmd_buffer_pos++;
    if (cmd_buffer_pos >= cmd_buffer_size) {
      cmd_buffer_pos = 0;
      break;
    }

    // Befehl vollständig ausgelesen ?
    if (_temp_char == '\n') {
      cmd_buffer[cmd_buffer_pos] = '\0';
      cmd_buffer_pos = 0;

      // Befehl ausführen
      Run_CMD(cmd_buffer);
      break;
    }

    // Neues Zeichen aus UART/Ringpuffer laden
    _temp_char = ringbuffer_get();
  }


  //////////////////////////////////////////////////////
  // UART nach 250ms zurücksetzen
  //////////////////////////////////////////////////////
  if (HAL_GetTick() > (uart_last_time + 250) && uart_last_time != 0 && !isBusy()) {
    cmd_buffer_pos = 0;
    ringbuffer_init();
    uart_last_time = HAL_GetTick();
  }
  if (HAL_GetTick() > (uart_last_time + 50)) {
    HAL_GPIO_WritePin(LED_RX_GPIO_Port, LED_RX_Pin,  0);
    uart_last_time = HAL_GetTick();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//////////////////////////////////////////////////////
// FlipDot Schreibprozess aktiv?
//////////////////////////////////////////////////////
int isBusy() {
  return start_refresh | start_refresh_center | start_refresh_DOWN | start_refresh_UP | start_yellow | start_black;
}


//////////////////////////////////////////////////////
// UART Interrupts verarbeiten
//////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  // UART 1
  if (huart->Instance == USART1) {

    // RX LED einschalten
    HAL_GPIO_WritePin(LED_RX_GPIO_Port, LED_RX_Pin,  1);

    //////////////////////////////////////////////////////
    // Daten wegschreiben
    //////////////////////////////////////////////////////
    ringbuffer_pushback((unsigned char)uart_puffer);

    // RX LED wieder ausschalten
    // HAL_GPIO_WritePin(LED_RX_GPIO_Port, LED_RX_Pin,  0);
    // Wird im Hauptprogramm nach einem gewissen Delay wieder ausgeschaltet!

    // Aktuelle Zeit speichern
    uart_last_time = HAL_GetTick();
    
    // Interrupt erneut starten
    HAL_UART_Receive_IT(&huart1, &uart_puffer, 1);
  }
}



//////////////////////////////////////////////////////
// Timer Interrupts verarbeiten
//////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  // Timer 2 (Multiplexing)
  if (htim->Instance == TIM2) {

    // TIMER 2 ist deaktiviert!!

    // Auf nächsten Schritt umschalten und berechnete Zeit warten!
    //__HAL_TIM_SET_COUNTER(&htim2, 0);
    //__HAL_TIM_SET_AUTORELOAD(&htim2, (on_time)+4);
  }
}



//////////////////////////////////////////////////////
// Befehl verarbeiten
//////////////////////////////////////////////////////
int Run_CMD(unsigned char* command) {


  //////////////////////////////////////////////////////
  // 1. Befehl aufsplitten
  //////////////////////////////////////////////////////
  
  // count and replace " with \0
  uint32_t str_len = strlen((const char*)command);
  uint8_t quotation_marks_cnt = 0;
  char* arg_text = NULL;
  for (uint32_t i=0; i<str_len; i++) {
    if (command[i] == '\"') {
      command[i] = '\0';
      quotation_marks_cnt++;

      // set text arg pointer
      if (!arg_text) {
        arg_text = (char*)(command + i + 1);
      }
    }
    if (command[i] == '\n') { // delete newline
      command[i] = '\0';
    }
  }
  if (quotation_marks_cnt != 0 && quotation_marks_cnt != 2) {
    return -1; // invalid command
  }

  // separate command into args
  char* args[max_cmd_args];
  args[0] = strtok((char*)command, " ");
  int f = 0;
  while (args[f]) {
    f++;
    if (f >= max_cmd_args) { break; }
    args[f] = strtok(NULL, " ");
  }
  if (f < max_cmd_args && !arg_text) {
    args[f] = NULL;
  }

  // append the text-string to args
  if (arg_text) {
    uint8_t pos_text = f;
    if (!(pos_text >= max_cmd_args)) {
      args[pos_text] = arg_text;
    }
    if (pos_text+1 < max_cmd_args) {
      args[pos_text+1] = NULL;
    }
  }
 
  // count args
  uint8_t arg_cnt = 0;
  while (args[arg_cnt] && arg_cnt < max_cmd_args) { arg_cnt++; }
  if (!arg_cnt) {
    return -1;
  }

  // Befehl bestimmen
  uint8_t current_cmd = CMD_ERROR;
  if        (arg_cnt == 4 && !strcmp(args[0], "SET_DOT")) {   current_cmd = CMD_SET_DOT;
  } else if (arg_cnt == 2 && !strcmp(args[0], "LIGHT")) {     current_cmd = CMD_LIGHT; 
  } else if (arg_cnt == 2 && !strcmp(args[0], "SET_ALL")) {   current_cmd = CMD_SET_ALL;
  } else if (arg_cnt == 2 && !strcmp(args[0], "SET_BUFF")) {  current_cmd = CMD_SET_BUFF;
  } else if (arg_cnt == 1 && !strcmp(args[0], "REFRESH")) {   current_cmd = CMD_REFRESH; 
  } else if (arg_cnt == 4 && !strcmp(args[0], "TEXT_TOP")) {  current_cmd = CMD_TEXT_TOP; 
  } else if (arg_cnt == 4 && !strcmp(args[0], "TEXT_CENTER")) {   current_cmd = CMD_TEXT_CENTER;
  } else if (arg_cnt == 4 && !strcmp(args[0], "TEXT_BOTTOM")) {   current_cmd = CMD_TEXT_BOTTOM;
  }
  if (current_cmd == CMD_ERROR) {
    return -1;
  }



  //////////////////////////////////////////////////////
  // 2. Befehl ausführen
  //////////////////////////////////////////////////////
  switch (current_cmd) {

    // Lampe ein/aus
    // usage: LIGHT <1/0>
    case CMD_LIGHT:
      if (atoi(args[1])) {
        HAL_GPIO_WritePin(Lampe_GPIO_Port, Lampe_Pin, 1);
      } else {
        HAL_GPIO_WritePin(Lampe_GPIO_Port, Lampe_Pin, 0);
      }
      break;

    // Alle Dots setzen/löschen
    // usage: SET_ALL <1/0> (1=yellow, 0=black)
    case CMD_SET_ALL:
      if (atoi(args[1])) {
        start_yellow = 1;
      } else {
        start_black = 1;
      }
      break;
    
    // Alle Dots setzen/löschen (Zwischenspeicher)
    // usage: CMD_SET_BUFF <1/0> (1=yellow, 0=black)
    case CMD_SET_BUFF:
      if (atoi(args[1])) {
        for (int i=0; i<(28 * Panelanzahl); i++)
          dots[i] |= 0xFFFFFFFF;
      } else {
        for (int i=0; i<(28 * Panelanzahl); i++)
          dots[i] |= 0;
      }
      break;

    // Dot setzen/löschen (Zwischenspeicher)
    // usage: SET_DOT <x> <y> <1/0>
    case CMD_SET_DOT: {
      uint32_t x = atoi(args[1]);
      uint32_t y = atoi(args[2]);
      if (x > (Panelanzahl * 28) - 1) { break; }
      if (y > (20 - 1)) { break; }
      if (atoi(args[3])) {

        // Setzen
        dots[x] |= ((uint32_t)0x00000001 << y);
      
      } else {

        // Löschen
        dots[x] &= ~((uint32_t)0x00000001 << y);
      }
      break; }
    
    // Pixeldaten ausgeben (Refresh)
    // usage: REFRESH
    case CMD_REFRESH:
      start_refresh = 1;
      break;

    // Text (mittig)
    // usage: TEXT_CENTER <Font_ID> <Position> "<Text>"
    // (Position: L=left, C=center, R=right, 0..83/111=offset von links)
    case CMD_TEXT_CENTER: {

      // Textlänge festlegen
      int text_laenge = strlen(args[3]);
      if (text_laenge <= 0) { break; }

      // Checke auf gültige Schriftart und Position
      int font = atoi(args[1]);
      if (font != 1 && font != 0)
        break;
      int offset_mode = -1;
      unsigned char position = atoi(args[2]);
      if (args[2][0] == 'L' || args[2][0] == 'C' || args[2][0] == 'R') {
        offset_mode = 0;
        position = args[2][0];
      } else {
        if (position < (28 * Panelanzahl))
          offset_mode = 1;
      }
      if (offset_mode < 0)
        break; // error
      
      // Text generieren
      for (int i=0; i<max_text_size; i++) { textbuffer_8px_UP[i] = 0; }
      int text_breite = generate_text((unsigned char*)args[3], textbuffer_8px_UP, 0, text_laenge, font);
      if (!text_breite) { break; }

      // Mittlere Textreihe im Dot-Buffer löschen
      for (int i=0; i<(28 * Panelanzahl); i++)
        dots[i] = dots[i] & 0xFFFFF00F;

      // evtl. alles löschen
      if (yellow_test_or_UP_DOWN) {
        for (int i=0; i<(28 * Panelanzahl); i++) {
          dots[i] = 0; }
      }

      // Text in Dot-Buffer schreiben
      uint8_t offset = 0;
      if (text_breite < (28 * Panelanzahl)) {
        offset = ((Panelanzahl * 28)/2) - (text_breite/2);
      }
      if (text_breite >= (28 * Panelanzahl))
        text_breite = (28 * Panelanzahl);
      if (position == 'L') // linksbündig
        offset = 0;
      if (position == 'R') { // rechtsbündig
        if (text_breite <= (28 * Panelanzahl)) {
          offset = (28 * Panelanzahl) - text_breite;
        } else { offset = 0; }
      }
      if (offset_mode) // 0..83/111 - offset manuell
        offset = position;
      for (int i=0; i<text_breite && i+offset<(28*Panelanzahl); i++) {
        dots[i+offset] |= (uint16_t)textbuffer_8px_UP[i] << 4;
      }

      // Schreibflags setzen
      if (!enable_manually_refresh) {
        if (yellow_test_or_UP_DOWN) {
          start_refresh = 1;
        } else {
          start_refresh_center = 1;
        }
      }
      yellow_test_or_center = 1;
      yellow_test_or_UP_DOWN = 0;
      break; }

    // Text (oben)
    // usage: TEXT_TOP <Font_ID> <Position 1/2/3> "<Text>"
    // (Position: 1=left, 2=center, 3=right)
    case CMD_TEXT_TOP: {

      // Textlänge festlegen
      int text_laenge = strlen(args[3]);
      if (text_laenge <= 0) { break; }

      // Checke auf gültige Schriftart und Position
      int font = atoi(args[1]);
      if (font != 1 && font != 0)
        break;
      int offset_mode = -1;
      unsigned char position = atoi(args[2]);
      if (args[2][0] == 'L' || args[2][0] == 'C' || args[2][0] == 'R') {
        offset_mode = 0;
        position = args[2][0];
      } else {
        if (position < (28 * Panelanzahl))
          offset_mode = 1;
      }
      if (offset_mode < 0)
        break; // error
      
      // Text generieren
      for (int i=0; i<max_text_size; i++) { textbuffer_8px_UP[i] = 0; }
      int text_breite = generate_text((unsigned char*)args[3], textbuffer_8px_UP, 0, text_laenge, font);
      if (!text_breite) { break; }

      // Obere Textreihe im Dot-Buffer löschen
      for (int i=0; i<(28 * Panelanzahl); i++)
        dots[i] = dots[i] & 0xFFFFFF00;
      
      // evtl. alles löschen
      if (yellow_test_or_center) {
        for (int i=0; i<(28 * Panelanzahl); i++) {
          dots[i] = 0; }
      }

      // Text in Dot-Buffer schreiben
      uint8_t offset = 0;
      if (text_breite < (28 * Panelanzahl)) {
        offset = ((Panelanzahl * 28)/2) - (text_breite/2);
      }
      if (text_breite >= (28 * Panelanzahl))
        text_breite = (28 * Panelanzahl);
      if (position == 'L') // linksbündig
        offset = 0;
      if (position == 'R') { // rechtsbündig
        if (text_breite <= (28 * Panelanzahl)) {
          offset = (28 * Panelanzahl) - text_breite;
        } else { offset = 0; }
      }
      if (offset_mode) // 0..83/111 - offset manuell
        offset = position;
      for (int i=0; i<text_breite && i+offset<(28*Panelanzahl); i++) {
        dots[i+offset] |= (uint16_t)textbuffer_8px_UP[i];
      }

      // Schreibflags setzen
      if (!enable_manually_refresh) {
        if (yellow_test_or_center) {
          start_refresh = 1;
        } else {
          start_refresh_UP = 1;
        }
      }
      yellow_test_or_center = 0;
      yellow_test_or_UP_DOWN = 1;
      break; }

    // Text (unten)
    // usage: TEXT_BOTTOM <Font_ID> <Position 1/2/3> "<Text>"
    // (Position: 1=left, 2=center, 3=right)
    case CMD_TEXT_BOTTOM: {

      // Textlänge festlegen
      int text_laenge = strlen(args[3]);
      if (text_laenge <= 0) { break; }

      // Checke auf gültige Schriftart und Position
      int font = atoi(args[1]);
      if (font != 1 && font != 0)
        break;
      int offset_mode = -1;
      unsigned char position = atoi(args[2]);
      if (args[2][0] == 'L' || args[2][0] == 'C' || args[2][0] == 'R') {
        offset_mode = 0;
        position = args[2][0];
      } else {
        if (position < (28 * Panelanzahl))
          offset_mode = 1;
      }
      if (offset_mode < 0)
        break; // error
      
      // Text generieren
      for (int i=0; i<max_text_size; i++) { textbuffer_8px_UP[i] = 0; }
      int text_breite = generate_text((unsigned char*)args[3], textbuffer_8px_UP, 0, text_laenge, font);
      if (!text_breite) { break; }

      // Unterste Textreihe im Dot-Buffer löschen
      for (int i=0; i<(28 * Panelanzahl); i++)
        dots[i] = dots[i] & 0xFFFF00FF;
      
      // evtl. alles löschen
      if (yellow_test_or_center) {
        for (int i=0; i<(28 * Panelanzahl); i++) {
          dots[i] = 0; }
      }

      // Text in Dot-Buffer schreiben
      uint8_t offset = 0;
      if (text_breite < (28 * Panelanzahl)) {
        offset = ((Panelanzahl * 28)/2) - (text_breite/2);
      }
      if (text_breite >= (28 * Panelanzahl))
        text_breite = (28 * Panelanzahl);
      if (position == 'L') // linksbündig
        offset = 0;
      if (position == 'R') { // rechtsbündig
        if (text_breite <= (28 * Panelanzahl)) {
          offset = (28 * Panelanzahl) - text_breite;
        } else { offset = 0; }
      }
      if (offset_mode) // 0..83/111 - offset manuell
        offset = position;
      for (int i=0; i<text_breite && i+offset<(28*Panelanzahl); i++) {
        dots[i+offset] |= (uint16_t)textbuffer_8px_UP[i] << 8;
      }

      // Schreibflags setzen
      if (!enable_manually_refresh) {
        if (yellow_test_or_center) {
          start_refresh = 1;
        } else {
          start_refresh_DOWN = 1;
        }
      }
      yellow_test_or_center = 0;
      yellow_test_or_UP_DOWN = 1;
      break; }

    default:
      // Default
      return -1;
      break;
  }

  return 0;
}



//////////////////////////////////////////////////////
// Zeile aktivieren
//////////////////////////////////////////////////////
void Zeile_aktivieren(uint8_t Zeile, uint8_t SET_RESET) {

  // Variabelcheck
  if (Zeile >= 24 || SET_RESET > 1) { return; }

  // Passendes Bit auf 1 setzen
  uint8_t array_pos = (Zeile / 8);
  uint8_t shift_data[3] = {0x00, 0x00, 0x00};
  uint8_t bit_pos = Zeile - (8 * array_pos);
  shift_data[2 - array_pos] = 0x01 << bit_pos;

  // Daten rausschieben
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&shift_data, 3, 100);

  // SET oder RESET (UDN2981 oder ULN2803 aktivieren)
  if (SET_RESET) {
    HAL_GPIO_WritePin(Zeilen_SET_RESET_GPIO_Port, Zeilen_SET_RESET_Pin, 1);
  } else {
    HAL_GPIO_WritePin(Zeilen_SET_RESET_GPIO_Port, Zeilen_SET_RESET_Pin, 0);
  }

  // Latch Toggeln
  // (Anmerkung: OE wird extern über Timer gesteuert!)
  HAL_GPIO_WritePin(Zeilen_LATCH_GPIO_Port, Zeilen_LATCH_Pin, 1);
  HAL_GPIO_WritePin(Zeilen_LATCH_GPIO_Port, Zeilen_LATCH_Pin, 0);
}



//////////////////////////////////////////////////////
// Spalte aktivieren (Daten + Adressleitungen von FP2800)
//////////////////////////////////////////////////////
void Spalte_aktivieren(uint8_t Spalte, uint8_t SET_RESET) {

  // Variabelcheck
  if (Spalte >= 28) { return; }

  // Spalte umwandeln nach FP2800 Wahrheitstabelle
  if (Paneltyp == 0) { Spalte = 27 - Spalte; }
  uint8_t Spalte_FP2800 = Spalte + 1;
  if (Spalte > 6) {
    Spalte_FP2800++;
    if (Spalte > 13) {
      Spalte_FP2800++;
      if (Spalte > 20) {
        Spalte_FP2800++;
      }
    }
  }

  // Adressbits in Zwischenspeicher schreiben
  hspi2_data[1] = hspi2_data[1] & 0xC0;
  Spalte_FP2800 = Spalte_FP2800 << 1;
  hspi2_data[1] = hspi2_data[1] | Spalte_FP2800;

  // Datenbit in Zwischenspeicher schreiben
  if (SET_RESET) {
    hspi2_data[1] = hspi2_data[1] | 0x01;
  }

  // Daten rausschieben
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&hspi2_data, 2, 100);  

  // Latch Toggeln
  HAL_GPIO_WritePin(Spalten_LATCH_GPIO_Port, Spalten_LATCH_Pin, 1);
  HAL_GPIO_WritePin(Spalten_LATCH_GPIO_Port, Spalten_LATCH_Pin, 0);
}



//////////////////////////////////////////////////////
// Modul aktivieren (Enable von FP2800)
//////////////////////////////////////////////////////
void Modul_aktivieren(uint8_t Modul, uint8_t ON_OFF) {

  // LAWO
  if (Paneltyp == 0) {

    // Variabelcheck
    if (Modul >= 4) { return; }

    // Alte Modulbits löschen
    hspi2_data[1] = hspi2_data[1] & 0x3F;
    hspi2_data[0] = hspi2_data[0] & 0xC0;

    // Modulbit setzen
    if (Paneltyp == 0) { Modul = (Panelanzahl - 1) - Modul; }
    if (ON_OFF) {
      switch (Modul) {
        case 0: hspi2_data[1] = hspi2_data[1] | 0x40; break;
        case 3: hspi2_data[1] = hspi2_data[1] | 0x80; break;
        case 2: hspi2_data[0] = hspi2_data[0] | 0x01; break;
        case 1: hspi2_data[0] = hspi2_data[0] | 0x02; break;
        default: break;
      }
    }
  }

  // BROSE
  if (Paneltyp == 1) {

    // Variabelcheck
    if (Modul >= 8) { return; }

    // Alle Bits auf 1 setzten
    hspi2_data[1] = hspi2_data[1] | 0xC0;
    hspi2_data[0] = hspi2_data[0] | 0x3F;

    // Modulbit löschen
    if (ON_OFF) {
      if (Modul == 0) { hspi2_data[1] = hspi2_data[1] & 0xBF; }
      if (Modul == 1) { hspi2_data[1] = hspi2_data[1] & 0x7F; }
      if (Modul >= 2) {
        Modul = Modul - 2;
        uint8_t bit = 0x01 << Modul;
        bit = ~bit;
        hspi2_data[0] = hspi2_data[0] & bit;
      }
    }
  }

  // Daten rausschieben
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&hspi2_data, 2, 100);  

  // Latch Toggeln
  HAL_GPIO_WritePin(Spalten_LATCH_GPIO_Port, Spalten_LATCH_Pin, 1);
  HAL_GPIO_WritePin(Spalten_LATCH_GPIO_Port, Spalten_LATCH_Pin, 0);
}



//////////////////////////////////////////////////////
// Dots schreiben
//////////////////////////////////////////////////////
void Write_Dots(uint8_t from_x, uint8_t to_x, uint8_t from_y, uint8_t to_y) {

  // Status LED ein
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin,  1);

  // Parameter bestimmen
  uint8_t x_max = Panelanzahl * 28;
  uint8_t y_max = 0;
  if (Paneltyp == LAWO)  { y_max = 16; }  // Lawo
  if (Paneltyp == BROSE) { y_max = 19; }  // Brose

  // Variabelcheck
  if (from_x >= x_max || to_x >= x_max || from_x > to_x) { return; }
  if (from_y >= y_max || to_y >= y_max || from_y > to_y) { return; }

  // Spalten und Zeilen abarbeiten
  for (uint8_t Spalte = from_x; Spalte <= to_x; Spalte++) {
    for (uint8_t Zeile = from_y; Zeile <= to_y; Zeile++) {

      // Bit maskieren
      uint8_t skip = 0;
      uint8_t dot = 0;
      uint32_t current_data = dots[Spalte] & (0x00000001 << Zeile);
      if (current_data) { dot = 1; }

      // Checken ob Dot überhaupt erneut gekippt werden muss
      if (enable_fast_refresh) {
        skip |= bit_not_changed(Spalte, Zeile);
      }
      
      // Flip Vorgang ausführen
      if (!skip) {

        // Spalten + Zeilentreiber einstellen
        Spalte_aktivieren(Spalte - ((Spalte/28)*28), !dot);
        Zeile_aktivieren(Zeile, dot);

        // Spalten + Zeilentreiber einschalten
        Modul_aktivieren(Spalte/28, 1);
        HAL_GPIO_WritePin(Zeilen_OE_GPIO_Port, Zeilen_OE_Pin, 1);

        // 1ms Delay
        delay_us(1000);

        // Spalten + Zeilentreiber ausschalten
        HAL_GPIO_WritePin(Zeilen_OE_GPIO_Port, Zeilen_OE_Pin, 1);
        Modul_aktivieren(Spalte/28, 0);
      }
    }
  }

  // Status LED aus
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin,  0);
}



//////////////////////////////////////////////////////
// Text generieren (8px Höhe)
//////////////////////////////////////////////////////
// text_selector:
// '0' = fnt8x8.h
// '1' = 6x8_vertikal_LSB_1.h
int generate_text(uint8_t* text_in, uint8_t* text_out, int start, int end, uint8_t text_selector) {

  // Zeichenanzahl überprüfen
  if ((end-start) <= 0) { return 0; }

  // Font Init
  int char_width = 0;
  uint8_t* font_data = NULL;
  switch (text_selector) {
    case 0: char_width = 8;   font_data = (uint8_t*)font8x8int;             break;
    case 1: char_width = 6;   font_data = (uint8_t*)font_6x8vertikal_LSB;   break;
    default: return 0; break;
  }
  if (font_data == NULL) { return 0; }

  // Text aus Zeichen zusammensetzen
  int pos = 0;
  int spaceing_counter = 0;
  for (int i=start; i<end; i++) {

    // Alle Spalten eines Zeichens rüberkopieren
    for (int spalte=0; spalte<char_width; spalte++) {

      // Vertikale Pixeldaten abfragen
      uint8_t vert_data = (font_data[(char_width * text_in[i]) + spalte]);

      // evtl. Zwischenräume auslassen
      if (vert_data == 0 && text_in[i] != 32) { spaceing_counter++; } else { spaceing_counter=0; }
      if (spaceing_counter <= 1) {

        // Pixeldaten schreiben
        text_out[pos] = vert_data;
        pos++;
      }
    }
  }

  // Geschriebe Länge zurückgeben
  return pos;
}



//////////////////////////////////////////////////////
// Hilfsfunktionen
//////////////////////////////////////////////////////

// Gibt zurück, ob sich ein Bit nicht geändert hat
uint8_t bit_not_changed(uint8_t x, uint8_t y) {
  uint32_t new = dots[x]     & (0x00000001 << y);
  uint32_t old = dots_old[x] & (0x00000001 << y);
  if (new != old) {
    return 0x00; // bit changed
  } else {
    return 0xFF; // bit not changed
  }
}

// Speichert ein Backup vom Pixelbuffer 
void save_written_dots(void) {
  if (enable_fast_refresh) {
      for (uint8_t i=0; i < Panelanzahl * 28; i++)
        dots_old[i] = dots[i];
  }
  return;
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
