/**
 * @file    tle9012_port.c
 * @brief   Transporte iso UART para STM32G4 -- bare-metal com DMA.
 */

#include "tle9012_port.h"

#include <stddef.h>

/* Potencia de 2: permite usar mascara em vez de divisao no ring buffer.
 * 64 bytes cobrem com folga a maior transacao (9 bytes) mesmo com lixo
 * acumulado de uma transacao anterior malsucedida. */
#define RX_RING_SIZE   64u
#define RX_RING_MASK   (RX_RING_SIZE - 1u)

/* Timeout de uma transacao. A 2 Mbit/s, 9 bytes levam ~45 us; 10 ms e folga
 * generosa para bring-up. HAL_GetTick() tem resolucao de 1 ms, entao nao
 * adianta pedir menos que 2. */
#define TP_TIMEOUT_MS  10u

/* Guarda do TX: se o DMA nao terminar nisso, algo esta travado. */
#define TX_TIMEOUT_MS  5u

/* Pino nSleep do TLE9015. Inicializado aqui, e nao no CubeMX, para manter todo
 * o hardware do link num arquivo so -- trocar de pino e uma linha, e nao exige
 * regerar o .ioc. Ajustar conforme a fiacao. */
#define NSLEEP_PORT        GPIOB
#define NSLEEP_PIN         GPIO_PIN_0
#define NSLEEP_CLK_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE

static UART_HandleTypeDef *s_huart;
static volatile uint8_t    s_rx_ring[RX_RING_SIZE];
static uint16_t            s_rx_tail;

/* --- Diagnostico para Live Expressions ----------------------------------- */
/* tp_dbg_avail_on_timeout responde a pergunta que mais importa quando o link
 * nao sobe: chegou algum byte de volta?
 *   0        -> nada voltou. Suspeitar da fiacao: RX nao esta vendo nem o
 *               proprio eco do TX (resistor serie ausente ou pino errado).
 *   parcial  -> o eco chega mas o escravo nao responde. Suspeitar de
 *               alimentacao do TLE9012, nSLEEP ou baudrate.
 *   completo -> problema de parsing, nao de fiacao. */
volatile uint16_t tp_dbg_avail_on_timeout;
volatile uint32_t tp_dbg_error_callbacks;

/* ------------------------------------------------------------------------- */

/** Posicao onde o DMA vai escrever o proximo byte. */
static uint16_t rx_head(void)
{
  const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
  return (uint16_t)((RX_RING_SIZE - remaining) & RX_RING_MASK);
}

static uint16_t rx_available(void)
{
  return (uint16_t)((rx_head() - s_rx_tail) & RX_RING_MASK);
}

/* ------------------------------------------------------------------------- */

static void port_flush(void)
{
  /* Descarta tudo o que estiver pendente: o eco da transacao anterior e
   * qualquer resposta atrasada nao devem contaminar a proxima leitura. */
  s_rx_tail = rx_head();
}

static bool port_send(const uint8_t *buf, uint16_t len)
{
  if (HAL_UART_Transmit_DMA(s_huart, (uint8_t *)buf, len) != HAL_OK)
  {
    return false;
  }

  /* Bloqueia ate o DMA esvaziar o buffer: quem chamou usa uma variavel local
   * como origem, entao ela precisa continuar valida ate o fim da transferencia. */
  const uint32_t t0 = HAL_GetTick();

  while (s_huart->gState != HAL_UART_STATE_READY)
  {
    if ((HAL_GetTick() - t0) > TX_TIMEOUT_MS)
    {
      (void)HAL_UART_AbortTransmit(s_huart);
      return false;
    }
  }

  return true;
}

static bool port_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  if (len > RX_RING_MASK)
  {
    return false;  /* nao cabe sem ambiguidade entre head e tail */
  }

  const uint32_t t0 = HAL_GetTick();

  while (rx_available() < len)
  {
    if ((HAL_GetTick() - t0) > timeout_ms)
    {
      tp_dbg_avail_on_timeout = rx_available();
      return false;
    }
  }

  for (uint16_t i = 0u; i < len; i++)
  {
    buf[i]    = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & RX_RING_MASK);
  }

  return true;
}

static void port_delay_us(uint32_t us)
{
  const uint32_t start  = DWT->CYCCNT;
  const uint32_t cycles = us * (SystemCoreClock / 1000000u);

  while ((DWT->CYCCNT - start) < cycles)
  {
    /* espera ativa */
  }
}

/* ------------------------------------------------------------------------- */

static const tle9012_transport_t s_transport =
{
  .send       = port_send,
  .recv       = port_recv,
  .flush      = port_flush,
  .delay_us   = port_delay_us,
  .timeout_ms = TP_TIMEOUT_MS
};

const tle9012_transport_t *tle9012_port_transport(void)
{
  return &s_transport;
}

void tle9012_port_inhibit_sleep(void)
{
  GPIO_InitTypeDef gpio = {0};

  NSLEEP_CLK_ENABLE();

  /* Nivel alto ANTES de configurar como saida, para nunca gerar a borda de
   * descida que justamente manda o TLE9015 dormir. */
  HAL_GPIO_WritePin(NSLEEP_PORT, NSLEEP_PIN, GPIO_PIN_SET);

  gpio.Pin   = NSLEEP_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSLEEP_PORT, &gpio);
}

void tle9012_port_recover(void)
{
  tp_dbg_error_callbacks++;

  (void)HAL_UART_AbortReceive(s_huart);

  __HAL_UART_CLEAR_OREFLAG(s_huart);
  __HAL_UART_CLEAR_NEFLAG(s_huart);
  __HAL_UART_CLEAR_FEFLAG(s_huart);

  s_rx_tail = 0u;
  (void)HAL_UART_Receive_DMA(s_huart, (uint8_t *)s_rx_ring, RX_RING_SIZE);
}

void tle9012_port_init(UART_HandleTypeDef *huart)
{
  s_huart   = huart;
  s_rx_tail = 0u;

  /* DWT para o delay de microssegundos. Precisa do TRCENA ligado. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* DMA circular: nunca para, entao nao ha janela entre frames onde um byte
   * possa se perder enquanto o DMA seria rearmado. */
  (void)HAL_UART_Receive_DMA(s_huart, (uint8_t *)s_rx_ring, RX_RING_SIZE);
}
