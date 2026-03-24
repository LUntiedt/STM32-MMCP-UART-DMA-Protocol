#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "bsp.h"

/* ========================================================== */
/* 2. Private Funktions-Prototypen (nur für diese Datei)      */
/* ========================================================== */
static void MX_Taster_Init(void);
static void MX_LED_Init(void);
static void MX_LS_GPIO_Init(void);
static void MX_SV_GPIO_Init(void);

/**
 * @brief  Initialisiert den DMA (Direct Memory Access) Controller.
 * * WARUM BRAUCHEN WIR DAS?
 * Die WS2812-LEDs benötigen ein extrem striktes Timing (800 kHz).
 * Würde die CPU jedes Bit einzeln an den Timer senden, würde jeder
 * UART-Interrupt (vom MMCP) das Timing zerschießen -> die LEDs würden flackern.
 * * Der DMA agiert als "Laufbursche": Er schaufelt automatisch Daten aus
 * unserem RAM (dem Farbpuffer) in das Timer-Register (CCR), sobald der
 * Timer "Hier!" ruft. Die CPU kann währenddessen schlafen oder rechnen.
 */
void MX_DMA_Init(void)
{
  // Clocks aktivieren
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Interrupt für DMA1 Stream1 (TIM2_CH3) aktivieren
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief  Konfiguriert TIM2 für die WS2812-Ansteuerung via PWM.
 * * FUNKTIONSPRINZIP:
 * Wir nutzen die PWM nicht zum Dimmen, sondern zur Datencodierung.
 * Eine "0" und eine "1" unterscheiden sich nur durch die Dauer des HIGH-Pegels.
 * * MATHEMATIK (bei 84 MHz CPU-Takt):
 * - WS2812 Bit-Dauer: 1,25 µs (fix).
 * - Timer-Prescaler: 0 (Wir nutzen den vollen Takt).
 * - Timer-Period (ARR): 104 Takte.
 * Rechnung: (1 / 84 MHz) * 105 = 1,25 µs.
 * * Der Timer läuft also exakt alle 1,25 µs einmal komplett durch.
 * Der DMA ändert bei jedem Durchlauf den "Pulse"-Wert (CCR3), um
 * entweder eine kurze (0) oder lange (1) Flanke zu erzeugen.
 */
void MX_TIM2_Init(void)
{
  // Clocks
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Für PB10

  // 1. Pin PB10 Konfiguration (AF)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 2. Timer Basis
  //TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 104; // 1,25µs bei 84MHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

  // 3. PWM Konfig Channel 3
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

  // 4. DMA Link
  hdma_tim2_ch3.Instance = DMA1_Stream1;
  hdma_tim2_ch3.Init.Channel = DMA_CHANNEL_3;
  hdma_tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // 32 Bit (TIM2 ist 32 Bit)
  hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    // Wir senden Bytes aus dem Puffer (oder Word, je nach Array-Typ)
  hdma_tim2_ch3.Init.Mode = DMA_NORMAL;
  hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_HIGH;
  
  if (HAL_DMA_Init(&hdma_tim2_ch3) != HAL_OK) Error_Handler();

  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC3], hdma_tim2_ch3);
}

/**
 * @brief  Initialisiert die "Hände" des Boards (SV-Pins).
 * * Diese Pins dienen als AUSGÄNGE, um dem Nachbarn zu signalisieren:
 * "Ich habe dir gerade ein Paket geschickt, nimm es an!"
 * * Protokoll: Wir setzen den Pin kurz auf HIGH (ca. 1ms), dann wieder LOW.
 * Der Nachbar (dort an einem LS-Pin angeschlossen) erkennt die steigende Flanke.
 */
static void MX_SV_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // --- NEU: Neighbor Outputs (SV-Pins) ---
  // PA6 (S), PA7 (O)
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Pin Reset (sicherstellen dass sie LOW sind)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  // PB0 (N), PB9 (W)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief  Initialisiert die "Ohren" des Boards (LS-Pins).
 * * Diese Pins lauschen auf Signale von den Nachbarn.
 * Wenn ein Nachbar seinen SV-Pin auf High setzt, empfangen wir hier
 * eine steigende Flanke.
 * * MODUS: IT_RISING (Interrupt bei steigender Flanke).
 * WARUM? Damit wir das Signal nicht verpassen, auch wenn wir gerade
 * mit den LEDs oder UART beschäftigt sind. Ein Polling in der main-Loop
 * wäre zu langsam oder unzuverlässig.
 */
static void MX_LS_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // --- NEU: Neighbor Inputs (LS1-LS4) als Interrupt ---
  // PC10, PC11, PC12 (N, O, S)
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt bei steigender Flanke
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;       // Sicherer Pegel
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // PD2 (W)
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// Taster-Konfiguration
static void MX_Taster_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// LED-Konfiguration
static void MX_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Dieser Handler wird aufgerufen, wenn der DMA-Transfer für die LEDs fertig ist.
 * WICHTIG: Ohne diese Funktion stürzt der Controller ab, sobald LEDs gesendet werden!
 */
void DMA1_Stream1_IRQHandler(void)
{
    // Wir reichen das Ereignis an den HAL-Treiber weiter.
    // Der kümmert sich um Flags und ruft dann 'PulseFinishedCallback' auf.
    HAL_DMA_IRQHandler(&hdma_tim2_ch3);
}

/**
 * @brief  Gemeinsamer Interrupt-Handler für Pins 10 bis 15.
 * * Hier laufen zusammen:
 * - User Button (PC13)
 * - Nachbar Norden (PC10)
 * - Nachbar Osten (PC11)
 * - Nachbar Süden (PC12)
 * * Wir rufen für jeden Pin den HAL-Handler auf. Dieser prüft,
 * ob der Interrupt wirklich von DIESEM Pin kam (Flag gesetzt?),
 * löscht das Flag und ruft dann unseren Callback auf.
 */
void EXTI15_10_IRQHandler(void) // <--  Name ist fest mit Hardware verbunden
{
  /*
   * Diese HAL-Funktion prüft, von welchem Pin der Interrupt kam,
   * löscht das notwendige Flag und ruft am Ende deine Callback-Funktion auf.
   */
  // Diese Funktion in z.60 macht im Hintergrund im Wesentlichen:
  // 1. if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != 0x00u) { ... }
  // 2. __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);  <-- HIER WIRD DAS FLAG GELÖSCHT
  // 3. HAL_GPIO_EXTI_Callback(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);

  /* Erstmal ausschalten zum testen
  // 2. Die Nachbar-Eingänge (LS1, LS2, LS3)
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); // LS1 (Norden)
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11); // LS2 (Osten)
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); // LS3 (Süden)
  */
}

/**
 * @brief  Spezifischer Interrupt-Handler für EXTI Line 2.
 * * Da der Pin PD2 (Nachbar Westen) an der Interrupt-Linie 2 hängt,
 * hat er seinen eigenen Vektor in der Interrupt-Tabelle.
 * Er teilt sich diesen NICHT mit den anderen Pins.
 */
void EXTI2_IRQHandler(void)
{
  //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); erstmal ausschalten zum testen
}


/**
 * @brief Manuelle Konfiguration der GPIOs (ersetzt Klicks in CubeIDE)
 */
void MX_GPIO_Init(void)
{
  // Clocks für alle GPIO-Ports aktivieren
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // Initialisiere Taster und LED (war noch aus alter Version)
  MX_Taster_Init();
  MX_LED_Init();

  // Initialisiere die Nachbar-Eingänge und -Ausgänge
  MX_LS_GPIO_Init();
  MX_SV_GPIO_Init();


  // Den Interrupt im NVIC aktivieren (Nested Vectored Interrupt Controller)
  // EXTI15_10_IRQn ist der Name des Interrupt-Kanals für die Pins 10 bis 15
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0); // Erst konfigurieren dann aktivieren
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Interrupts noch für PD2 aktivieren, PC10, PC11, PC12 werden mit EXTI15_10_IRQn abgedeckt
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/**
 * @brief This function handles USART2 global interrupt.
 * Dieser Name ist ebenfalls fest mit der Hardware verbunden.
 */
void USART2_IRQHandler(void)
{
  // Übergibt die Verarbeitung an den HAL-Treiber.
  // Dieser löscht die Flags und ruft die passenden Callbacks auf.
  HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief Manuelle Konfiguration von USART2 (ersetzt Klicks in CubeIDE)
 */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Low-Level Initialisierung für die UART (wird von HAL_UART_Init aufgerufen)
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance == USART2)
  {
    // 1. Clock für USART2 und GPIOA aktivieren
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. Pins PA2 (TX) und PA3 (RX) konfigurieren
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // Kein Pull-Up oder Pull-Down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // AF7 für USART2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // In HAL_UART_MspInit (oder MX_USART2_UART_Init):
    // 3. Den Interrupt-Kanal für USART2 im NVIC aktivieren
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  else if (huart->Instance == USART1)
  {
    // Konfiguriere hier die Clocks und GPIO-Pins für USART1
    // z.B. PA9 (TX) und PA10 (RX)

    // 1. Clock für USART1 und GPIOA aktivieren
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. Pins PA9 (TX) und PA10 (RX) konfigurieren
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // Kein Pull-Up oder Pull-Down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // AF7 für USART1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // In HAL_UART_MspInit (oder MX_USART1_UART_Init):
    // 3. Den Interrupt-Kanal für USART1 im NVIC aktivieren
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

/*
 * 🛠️ WICHTIG: Konfiguriert das "Kraftwerk" des Mikrocontrollers.
 * Diese Funktion stellt den Haupt-Systemtakt auf die volle Geschwindigkeit
 * (für dieses Board 84 MHz) ein. Ohne diesen Aufruf läuft der Chip in einem
 * langsamen, instabilen Notlaufmodus.
 * Das ist die Grundlage für stabile Peripherie (korrekte UART-Baudrate)
 * und verhindert Abstürze bei komplexen Operationen wie Interrupts.
 */
// Standard-Systemkonfigurationen (können so belassen werden)
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  while (1)
    ;
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}