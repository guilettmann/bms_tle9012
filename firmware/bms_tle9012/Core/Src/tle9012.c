/**
 * @file    tle9012.c
 * @brief   Driver do TLE9012DQU sobre iso UART.
 *
 * Formato dos frames (o byte SYNC entra no CRC -- validado contra os vetores
 * das Tabelas 7-20 do user manual):
 *
 *   Escrita (6 bytes):  1E | 80|ID | ADDR | DATA_H | DATA_L | CRC8
 *   Leitura (4 bytes):  1E | 00|ID | ADDR | CRC8
 *
 * Como a linha e compartilhada, o host recebe de volta o proprio eco:
 *
 *   Escrita: 6 de eco + 1 byte de reply frame (CRC3)          = 7 bytes
 *   Leitura: 4 de eco + 5 de resposta (SYNC ID DH DL CRC8)    = 9 bytes
 */

#include "tle9012.h"
#include "tle9012_regs.h"
#include "crc_j1850.h"

#include <stddef.h>   /* NULL */

#define TLE9012_SYNC        0x1Eu
#define TLE9012_WRITE_BIT   0x80u

#define FRAME_LEN_WRITE     6u
#define FRAME_LEN_READ      4u
#define RESP_LEN_WRITE      7u   /* 6 de eco + reply frame                    */
#define RESP_LEN_READ       9u   /* 4 de eco + 5 de resposta                  */

/* Indices dentro da resposta de leitura, depois do eco. */
#define RESP_RD_BASE        4u   /* inicio do frame de resposta               */
#define RESP_RD_DATA_H      6u
#define RESP_RD_DATA_L      7u
#define RESP_RD_CRC         8u
#define RESP_WR_REPLY       6u   /* reply frame da escrita                    */

static const tle9012_transport_t *s_tp;

/* --- Snapshot para Live Expressions -------------------------------------- */
/* volatile e global de proposito: e assim que o depurador consegue ler estes
 * valores com o alvo rodando. Na primeira subida do link, a pergunta util e
 * sempre "o que voltou no fio?" -- estes buffers respondem isso. */
volatile uint8_t  tle_dbg_tx[FRAME_LEN_WRITE];
volatile uint8_t  tle_dbg_rx[RESP_LEN_READ];
volatile uint16_t tle_dbg_rx_len;
volatile uint8_t  tle_dbg_status;
volatile uint32_t tle_dbg_ok_count;
volatile uint32_t tle_dbg_err_count;

static void dbg_capture(const uint8_t *tx, uint16_t tx_len,
                        const uint8_t *rx, uint16_t rx_len,
                        tle9012_status_t st)
{
  for (uint16_t i = 0u; i < tx_len && i < sizeof(tle_dbg_tx); i++)
  {
    tle_dbg_tx[i] = tx[i];
  }
  for (uint16_t i = 0u; i < rx_len && i < sizeof(tle_dbg_rx); i++)
  {
    tle_dbg_rx[i] = rx[i];
  }
  tle_dbg_rx_len = rx_len;
  tle_dbg_status = (uint8_t)st;

  if (st == TLE9012_OK) { tle_dbg_ok_count++; }
  else                  { tle_dbg_err_count++; }
}

void tle9012_bind(const tle9012_transport_t *transport)
{
  s_tp = transport;
}

/* ------------------------------------------------------------------------- */

tle9012_status_t tle9012_write_reg(uint8_t node_id, uint8_t addr, uint16_t data)
{
  if ((s_tp == NULL) || (s_tp->send == NULL) || (s_tp->recv == NULL))
  {
    return TLE9012_ERR_PARAM;
  }

  uint8_t tx[FRAME_LEN_WRITE];
  uint8_t rx[RESP_LEN_WRITE];

  tx[0] = TLE9012_SYNC;
  tx[1] = (uint8_t)(TLE9012_WRITE_BIT | (node_id & 0x3Fu));
  tx[2] = addr;
  tx[3] = (uint8_t)(data >> 8);
  tx[4] = (uint8_t)(data & 0xFFu);
  tx[5] = crc8_j1850(tx, 5u);

  s_tp->flush();

  if (!s_tp->send(tx, FRAME_LEN_WRITE))
  {
    dbg_capture(tx, FRAME_LEN_WRITE, rx, 0u, TLE9012_ERR_TRANSPORT);
    return TLE9012_ERR_TRANSPORT;
  }

  if (!s_tp->recv(rx, RESP_LEN_WRITE, s_tp->timeout_ms))
  {
    /* Preserva o parcial: sao esses bytes que dizem se o eco esta integro. */
    const uint16_t got = (s_tp->drain != NULL)
                       ? s_tp->drain(rx, RESP_LEN_WRITE) : 0u;

    dbg_capture(tx, FRAME_LEN_WRITE, rx, got, TLE9012_ERR_TIMEOUT);
    return TLE9012_ERR_TIMEOUT;
  }

  if (!crc3_reply_valid(rx[RESP_WR_REPLY]))
  {
    dbg_capture(tx, FRAME_LEN_WRITE, rx, RESP_LEN_WRITE, TLE9012_ERR_CRC);
    return TLE9012_ERR_CRC;
  }

  dbg_capture(tx, FRAME_LEN_WRITE, rx, RESP_LEN_WRITE, TLE9012_OK);
  return TLE9012_OK;
}

tle9012_status_t tle9012_read_reg(uint8_t node_id, uint8_t addr, uint16_t *out)
{
  if ((out == NULL) || (s_tp == NULL) || (s_tp->send == NULL) || (s_tp->recv == NULL))
  {
    return TLE9012_ERR_PARAM;
  }

  uint8_t tx[FRAME_LEN_READ];
  uint8_t rx[RESP_LEN_READ];

  tx[0] = TLE9012_SYNC;
  tx[1] = (uint8_t)(node_id & 0x3Fu);   /* MSB = 0 indica leitura */
  tx[2] = addr;
  tx[3] = crc8_j1850(tx, 3u);

  s_tp->flush();

  if (!s_tp->send(tx, FRAME_LEN_READ))
  {
    dbg_capture(tx, FRAME_LEN_READ, rx, 0u, TLE9012_ERR_TRANSPORT);
    return TLE9012_ERR_TRANSPORT;
  }

  if (!s_tp->recv(rx, RESP_LEN_READ, s_tp->timeout_ms))
  {
    /* Preserva o parcial: sao esses bytes que dizem se o eco esta integro. */
    const uint16_t got = (s_tp->drain != NULL)
                       ? s_tp->drain(rx, RESP_LEN_READ) : 0u;

    dbg_capture(tx, FRAME_LEN_READ, rx, got, TLE9012_ERR_TIMEOUT);
    return TLE9012_ERR_TIMEOUT;
  }

  /* CRC da resposta cobre os 4 bytes do frame de resposta, SYNC incluso. */
  const uint8_t crc = crc8_j1850(&rx[RESP_RD_BASE], 4u);

  if (crc != rx[RESP_RD_CRC])
  {
    dbg_capture(tx, FRAME_LEN_READ, rx, RESP_LEN_READ, TLE9012_ERR_CRC);
    return TLE9012_ERR_CRC;
  }

  *out = (uint16_t)(((uint16_t)rx[RESP_RD_DATA_H] << 8) | (uint16_t)rx[RESP_RD_DATA_L]);

  dbg_capture(tx, FRAME_LEN_READ, rx, RESP_LEN_READ, TLE9012_OK);
  return TLE9012_OK;
}

/* ------------------------------------------------------------------------- */

void tle9012_wakeup(void)
{
  uint16_t scratch = 0u;

  /* O proprio ato de enviar a leitura acorda o dispositivo; a resposta
   * normalmente nao vem e isso e esperado. Dois disparos porque o primeiro
   * pode ser inteiramente consumido pelo despertar. */
  (void)tle9012_read_reg(TLE9012_UNASSIGNED_ID, TLE9012_REG_CONFIG, &scratch);

  if (s_tp->delay_us != NULL)
  {
    s_tp->delay_us(1000u);
  }

  (void)tle9012_read_reg(TLE9012_UNASSIGNED_ID, TLE9012_REG_CONFIG, &scratch);
}

tle9012_status_t tle9012_disable_open_load(uint8_t node_id)
{
  const tle9012_status_t st = tle9012_write_reg(node_id, TLE9012_REG_OL_OV_THR,
                                                TLE9012_OL_THR_DISABLED);
  if (st != TLE9012_OK)
  {
    return st;
  }

  return tle9012_write_reg(node_id, TLE9012_REG_OL_UV_THR,
                           TLE9012_OL_THR_DISABLED);
}

tle9012_status_t tle9012_assign_node_id(uint8_t new_id, bool final_node)
{
  if ((new_id == 0u) || (new_id >= TLE9012_BROADCAST_ID))
  {
    return TLE9012_ERR_PARAM;
  }

  uint16_t data = (uint16_t)new_id;

  if (final_node)
  {
    data |= TLE9012_CONFIG_FINAL_NODE;
  }

  /* Enderecado ao ID 0: responde o primeiro IC ainda nao endercado da cadeia. */
  return tle9012_write_reg(TLE9012_UNASSIGNED_ID, TLE9012_REG_CONFIG, data);
}

tle9012_status_t tle9012_set_cell_count(uint8_t node_id, uint8_t n_cells)
{
  if ((n_cells == 0u) || (n_cells > TLE9012_MAX_CELLS))
  {
    return TLE9012_ERR_PARAM;
  }

  /* Um bit por celula, a partir da celula 0. */
  const uint16_t mask = (uint16_t)((1u << n_cells) - 1u);

  return tle9012_write_reg(node_id, TLE9012_REG_PART_CONFIG, mask);
}

tle9012_status_t tle9012_start_measurement(uint8_t node_id)
{
  return tle9012_write_reg(node_id, TLE9012_REG_MEAS_CTRL,
                           TLE9012_MEAS_CTRL_PCVM_16BIT);
}

uint16_t tle9012_raw_to_mv(uint16_t raw)
{
  /* V[mV] = (FSR / 2^16) * raw, em inteiro para evitar float. */
  return (uint16_t)(((uint32_t)raw * TLE9012_PCVM_FSR_MV) >> 16);
}

tle9012_status_t tle9012_read_cells_mv(uint8_t node_id, uint16_t *mv, uint8_t n_cells)
{
  if ((mv == NULL) || (n_cells == 0u) || (n_cells > TLE9012_MAX_CELLS))
  {
    return TLE9012_ERR_PARAM;
  }

  for (uint8_t i = 0u; i < n_cells; i++)
  {
    uint16_t raw = 0u;
    const uint8_t addr = (uint8_t)(TLE9012_REG_PCVM_0 + i);

    const tle9012_status_t st = tle9012_read_reg(node_id, addr, &raw);

    if (st != TLE9012_OK)
    {
      return st;
    }

    mv[i] = tle9012_raw_to_mv(raw);
  }

  return TLE9012_OK;
}
