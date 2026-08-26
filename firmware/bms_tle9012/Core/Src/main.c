/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tle9012.h"
#include "tle9012_port.h"
#include "tle9012_regs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMS_NODE_ID        1u     /* unico TLE9012 da cadeia                  */
#define BMS_NUM_CELLS      12u
#define BMS_MEAS_PERIOD_MS 100u

/* Tempo de conversao do PCVM em 16 bits. Valor conservador para o bring-up:
 * o bit PCVM_START limpa sozinho ao terminar, entao o caminho correto e
 * fazer polling do MEAS_CTRL -- fica para quando a posicao do bit estiver
 * confirmada no manual (secao 4.23). */
#define BMS_CONV_WAIT_MS   5u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Snapshot de medicao -- volatile e global para o Live Expressions conseguir
 * ler com o alvo rodando. E tambem a estrutura que as camadas de protecao e
 * de CAN vao consumir nas proximas fases. */
volatile uint16_t cell_mv[BMS_NUM_CELLS];
volatile uint16_t cell_min_mv;
volatile uint16_t cell_max_mv;
volatile uint16_t cell_delta_mv;
volatile uint32_t meas_count;
volatile uint32_t meas_fail_count;
volatile uint8_t  last_status;      /* tle9012_status_t da ultima transacao */
volatile uint8_t  chain_ready;

/* Smoke test: resultado isolado do ciclo de medicao. Se smoke_ok == 1, o link
 * inteiro esta de pe -- baudrate, MSB first, CRC, eco e DMA. */
volatile uint8_t  smoke_ok;
volatile uint8_t  smoke_status;     /* tle9012_status_t do readback           */
volatile uint16_t smoke_config;     /* valor lido do CONFIG (0x36)            */
volatile uint32_t smoke_attempts;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Sobe a cadeia: atribui NODE_ID e configura as celulas ativas.
 * @return true se ambos os passos responderam com CRC valido.
 */
/**
 * @brief Menor teste possivel que prova o link inteiro.
 *
 * Atribui o NODE_ID e le o CONFIG de volta. Uma transacao de escrita e uma de
 * leitura, com valor esperado conhecido: se fecha, entao baudrate, MSB first,
 * CRC8, CRC3, eco e DMA estao todos corretos. Testar isto antes de tentar
 * medir tensao evita confundir erro de link com erro de medicao.
 */
static bool bms_smoke_test(void)
{
  smoke_attempts++;
  smoke_ok = 0u;

  /* Unico IC na cadeia, portanto ele e tambem o final node. Sem o bit FN
   * o broadcast nunca responde e tudo da timeout. */
  tle9012_status_t st = tle9012_assign_node_id(BMS_NODE_ID, true);

  if (st != TLE9012_OK)
  {
    smoke_status = (uint8_t)st;
    return false;
  }

  uint16_t config = 0u;
  st = tle9012_read_reg(BMS_NODE_ID, TLE9012_REG_CONFIG, &config);

  smoke_status = (uint8_t)st;
  smoke_config = config;

  if (st != TLE9012_OK)
  {
    return false;
  }

  /* O NODE_ID atribuido deve aparecer nos bits baixos do CONFIG. */
  if ((config & 0x3Fu) != BMS_NODE_ID)
  {
    return false;
  }

  smoke_ok = 1u;
  return true;
}

/**
 * @brief Sobe a cadeia: smoke test, desativa open load e configura as celulas.
 */
static bool bms_chain_init(void)
{
  if (!bms_smoke_test())
  {
    return false;
  }

  /* Ladder resistivo da placa de avaliacao dispara open load falso. */
  if (tle9012_disable_open_load(BMS_NODE_ID) != TLE9012_OK)
  {
    return false;
  }

  if (tle9012_set_cell_count(BMS_NODE_ID, BMS_NUM_CELLS) != TLE9012_OK)
  {
    return false;
  }

  /* Confirma lendo de volta -- o manual recomenda validar toda escrita de
   * configuracao relendo o registrador (secao 3.1.2). */
  uint16_t readback = 0u;

  if (tle9012_read_reg(BMS_NODE_ID, TLE9012_REG_PART_CONFIG, &readback) != TLE9012_OK)
  {
    return false;
  }

  return (readback == (uint16_t)((1u << BMS_NUM_CELLS) - 1u));
}

/** Um ciclo de medicao: dispara, espera a conversao e le as 12 celulas. */
static void bms_measure_cycle(void)
{
  uint16_t mv[BMS_NUM_CELLS];

  tle9012_status_t st = tle9012_start_measurement(BMS_NODE_ID);

  if (st == TLE9012_OK)
  {
    HAL_Delay(BMS_CONV_WAIT_MS);
    st = tle9012_read_cells_mv(BMS_NODE_ID, mv, BMS_NUM_CELLS);
  }

  last_status = (uint8_t)st;

  if (st != TLE9012_OK)
  {
    meas_fail_count++;
    return;
  }

  uint16_t vmin = 0xFFFFu;
  uint16_t vmax = 0u;

  for (uint8_t i = 0u; i < BMS_NUM_CELLS; i++)
  {
    cell_mv[i] = mv[i];

    if (mv[i] < vmin) { vmin = mv[i]; }
    if (mv[i] > vmax) { vmax = mv[i]; }
  }

  cell_min_mv   = vmin;
  cell_max_mv   = vmax;
  cell_delta_mv = (uint16_t)(vmax - vmin);
  meas_count++;
}

/**
 * @brief Rearma a recepcao apos erro de linha.
 * @note  Sem isto, um unico overrun derruba o DMA circular em definitivo.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    tle9012_port_recover();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  tle9012_port_init(&huart1);
  tle9012_bind(tle9012_port_transport());

  /* Ordem obrigatoria: o transceiver acorda por hardware (nSLEEP), o sensing
   * IC acorda por comando de leitura. Inverter a ordem nao funciona. */
  tle9012_port_wake_transceiver();
  tle9012_wakeup();

  chain_ready = bms_chain_init() ? 1u : 0u;
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (chain_ready == 0u)
    {
      /* Cadeia nao subiu: tenta de novo em vez de travar. Inspecionar
       * tp_dbg_avail_on_timeout primeiro, depois tle_dbg_rx / smoke_status. */
      HAL_Delay(500u);
      tle9012_wakeup();
      chain_ready = bms_chain_init() ? 1u : 0u;
      continue;
    }

    bms_measure_cycle();

    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(BMS_MEAS_PERIOD_MS);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_MSBFIRST_INIT;
  huart1.AdvancedInit.MSBFirst = UART_ADVFEATURE_MSBFIRST_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
