/**
 * @file    crc_j1850.h
 * @brief   CRCs do protocolo iso UART do TLE9012.
 *
 * O protocolo usa dois CRCs distintos:
 *   - CRC8 (SAE J1850) nos frames de dados, cobrindo inclusive o byte SYNC.
 *   - CRC3 no reply frame de 1 byte devolvido apos uma escrita.
 */

#ifndef CRC_J1850_H
#define CRC_J1850_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  CRC8 SAE J1850: poly 0x1D, init 0xFF, XOR final 0xFF.
 * @note   O byte SYNC (0x1E) faz parte do calculo. Validado contra os 10 vetores
 *         de exemplo do user manual da Infineon (Tabelas 7-20).
 */
uint8_t crc8_j1850(const uint8_t *data, uint16_t len);

/**
 * @brief  Valida o reply frame de 1 byte devolvido apos uma escrita.
 * @param  reply_frame  byte recebido.
 * @return true se o CRC3 fecha (resto zero).
 */
bool crc3_reply_valid(uint8_t reply_frame);

#endif /* CRC_J1850_H */
