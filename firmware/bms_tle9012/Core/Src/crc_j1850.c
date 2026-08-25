/**
 * @file    crc_j1850.c
 * @brief   Implementacao dos CRCs do protocolo iso UART do TLE9012.
 */

#include "crc_j1850.h"

/* Implementacao bit a bit em vez de lookup table: um frame tem no maximo 5 bytes,
 * ou seja 40 iteracoes (~0,3 us a 170 MHz) contra ~30 us de duracao do proprio
 * frame no fio a 2 Mbit/s. A tabela de 256 bytes nao se pagaria e o codigo
 * bit a bit e mais facil de auditar. */

uint8_t crc8_j1850(const uint8_t *data, uint16_t len)
{
  const uint8_t polynomial = 0x1Du;
  uint8_t crc = 0xFFu;

  for (uint16_t i = 0u; i < len; i++)
  {
    crc ^= data[i];

    for (uint8_t j = 0u; j < 8u; j++)
    {
      if ((crc & 0x80u) != 0u)
      {
        crc = (uint8_t)((crc << 1) ^ polynomial);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  return (uint8_t)(crc ^ 0xFFu);
}

bool crc3_reply_valid(uint8_t reply_frame)
{
  const uint8_t polynomial = 0xB0u;  /* x^3 + x + 1, alinhado a esquerda */
  uint8_t crc = reply_frame;

  /* 5 bits de dados no reply frame; os 3 restantes sao o proprio CRC. */
  for (uint8_t n = 0u; n < 5u; n++)
  {
    if ((crc & 0x80u) != 0u)
    {
      crc ^= polynomial;
    }
    crc = (uint8_t)(crc << 1);
  }

  return (crc == 0u);
}
