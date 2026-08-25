/**
 * @file    tle9012_port.h
 * @brief   Transporte iso UART para STM32G4 -- bare-metal com DMA.
 *
 * Implementa a tle9012_transport_t sobre um USART em modo assincrono com
 * DMA circular no RX e DMA normal no TX. Trocar este arquivo (e so ele) e
 * o suficiente para levar o driver a outro MCU ou a um RTOS.
 */

#ifndef TLE9012_PORT_H
#define TLE9012_PORT_H

#include "stm32g4xx_hal.h"
#include "tle9012.h"

/**
 * @brief Inicializa o transporte e arma a recepcao circular por DMA.
 * @param huart  USART ja inicializado pelo CubeMX (2 Mbit/s, MSB first).
 */
void tle9012_port_init(UART_HandleTypeDef *huart);

/** Devolve a interface de transporte para passar ao tle9012_bind(). */
const tle9012_transport_t *tle9012_port_transport(void);

/**
 * @brief Rearma a recepcao apos erro de linha (overrun, framing, noise).
 * @note  Chamar de HAL_UART_ErrorCallback(). Sem isso, um unico erro de linha
 *        derruba o DMA circular e a comunicacao para em definitivo.
 */
void tle9012_port_recover(void);

#endif /* TLE9012_PORT_H */
