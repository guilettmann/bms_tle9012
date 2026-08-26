/**
 * @file    tle9012.h
 * @brief   Driver do TLE9012DQU sobre iso UART.
 *
 * Este modulo nao contem nenhuma chamada de HAL. Toda a dependencia de hardware
 * entra pela struct tle9012_transport_t, o que mantem o driver portavel entre
 * STM32, AURIX e um mock de teste em PC.
 */

#ifndef TLE9012_H
#define TLE9012_H

#include <stdint.h>
#include <stdbool.h>

#define TLE9012_MAX_CELLS      12u
#define TLE9012_BROADCAST_ID   0x3Fu
#define TLE9012_UNASSIGNED_ID  0x00u

typedef enum
{
  TLE9012_OK = 0,
  TLE9012_ERR_TIMEOUT,    /**< nao chegou o numero esperado de bytes          */
  TLE9012_ERR_CRC,        /**< CRC8 (leitura) ou CRC3 (escrita) nao fechou    */
  TLE9012_ERR_TRANSPORT,  /**< falha ao transmitir                            */
  TLE9012_ERR_PARAM       /**< argumento invalido                             */
} tle9012_status_t;

/**
 * @brief Interface de transporte injetada pela camada de porte.
 *
 * @note  recv() deve devolver exatamente @p len bytes ou falhar por timeout.
 *        flush() descarta o que estiver pendente antes de uma nova transacao --
 *        essencial porque a linha e compartilhada e devolve o eco do TX.
 */
typedef struct
{
  bool (*send)(const uint8_t *buf, uint16_t len);
  bool (*recv)(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
  void (*flush)(void);
  void (*delay_us)(uint32_t us);

  /** Copia ate @p max_len bytes ja recebidos, devolvendo quantos copiou.
   *  Usado no caminho de timeout para preservar o que chegou -- tipicamente
   *  o eco, que e o que diz se o problema e local ou do outro lado do fio.
   *  Opcional: pode ser NULL. */
  uint16_t (*drain)(uint8_t *buf, uint16_t max_len);

  uint32_t timeout_ms;
} tle9012_transport_t;

/** Associa o driver a uma implementacao de transporte. Chamar antes de tudo. */
void tle9012_bind(const tle9012_transport_t *transport);

/**
 * @brief Acorda a cadeia enviando um comando de leitura descartavel.
 *
 * O manual (secao 3.6.2, passo 2) estabelece que um comando de leitura acorda
 * o dispositivo. Esse primeiro comando e consumido pelo despertar e costuma
 * nao responder, portanto o resultado e deliberadamente ignorado.
 *
 * @note Nao acorda o TLE9015: o transceiver e acordado por hardware, pelo pino
 *       nSLEEP, antes desta chamada.
 */
void tle9012_wakeup(void);

/**
 * @brief Desativa o diagnostico de open load.
 * @note  Obrigatorio ao usar o ladder resistivo da placa de avaliacao, que
 *        dispara open load falso. Remover ao migrar para celulas reais.
 */
tle9012_status_t tle9012_disable_open_load(uint8_t node_id);

/* --- Acesso de baixo nivel ---------------------------------------------- */
tle9012_status_t tle9012_write_reg(uint8_t node_id, uint8_t addr, uint16_t data);
tle9012_status_t tle9012_read_reg (uint8_t node_id, uint8_t addr, uint16_t *out);

/* --- Alto nivel ---------------------------------------------------------- */

/**
 * @brief Atribui um NODE_ID ao proximo IC nao endercado da cadeia.
 * @param new_id      ID a atribuir (1..62).
 * @param final_node  true no ultimo IC da cadeia -- sem isso o broadcast
 *                    nunca responde e da timeout na iso UART.
 */
tle9012_status_t tle9012_assign_node_id(uint8_t new_id, bool final_node);

tle9012_status_t tle9012_set_cell_count(uint8_t node_id, uint8_t n_cells);

/**
 * @brief Realimenta o watchdog do TLE9012, recarregando WD_CNT no maximo.
 *
 * O contador decrementa sozinho; ao zerar, o dispositivo entra em sleep e o
 * NODE_ID volta a 0, derrubando a comunicacao sem aviso. Precisa ser chamado
 * periodicamente enquanto a cadeia estiver ativa -- o firmware de demo da
 * Infineon usa 500 ms, e o ciclo de medicao de 100 ms cobre isso com folga.
 */
tle9012_status_t tle9012_kick_watchdog(uint8_t node_id);

/** Dispara a medicao primaria (PCVM) em 16 bits. */
tle9012_status_t tle9012_start_measurement(uint8_t node_id);

/**
 * @brief Aguarda o fim da conversao fazendo polling do bit PCVM_START.
 *
 * O hardware limpa PCVM_START (MEAS_CTRL bit 15) quando termina, o que
 * dispensa esperar um tempo fixo arbitrario: a leitura sai assim que fica
 * pronta e um atraso maior que o esperado vira erro em vez de dado invalido.
 *
 * @param max_polls  numero maximo de leituras antes de desistir.
 */
tle9012_status_t tle9012_wait_measurement(uint8_t node_id, uint16_t max_polls);

/* --- Multiread ----------------------------------------------------------- */

/* 96 porque a primeira sonda saturou em 48. Hipoteses para o tamanho real:
 * 12 respostas de 5 bytes (SYNC ID DH DL CRC) + 4 de eco = 64, ou 12 de 4
 * bytes + eco = 52. Ambas cabem aqui. */
#define TLE9012_MULTIREAD_MAX_BYTES 96u

/** Resposta crua da ultima sonda de multiread, para inspecao no depurador. */
extern volatile uint8_t  tle_mr_raw[TLE9012_MULTIREAD_MAX_BYTES];
extern volatile uint16_t tle_mr_len;

/**
 * @brief Configura o que entra na rajada de multiread.
 * @note  O manual (secao 4.37) exige que este registrador seja escrito por
 *        comando de BROADCAST, nao enderecado a um no especifico.
 */
tle9012_status_t tle9012_multiread_configure(uint16_t cfg);

/**
 * @brief Dispara um multiread e captura a resposta crua em tle_mr_raw.
 *
 * O formato do frame de resposta do multiread NAO esta documentado no user
 * manual -- ele descreve os registradores mas nao o arranjo dos bytes que
 * voltam. Esta funcao existe para observar a rajada real em bancada e so
 * entao escrever o parser, em vez de adivinhar o layout.
 */
tle9012_status_t tle9012_multiread_probe(uint8_t node_id);

/**
 * @brief Le todas as tensoes de celula numa unica transacao (multiread).
 *
 * Substitui as 12 transacoes de tle9012_read_cells_mv() por uma so. Exige
 * que tle9012_multiread_configure() tenha sido chamado antes.
 *
 * Formato da rajada, determinado em bancada (nao documentado no manual):
 * 4 bytes de eco seguidos de @p n_cells frames de 5 bytes, cada um no mesmo
 * layout da resposta de leitura simples -- SYNC, ID, DATA_H, DATA_L, CRC8,
 * com o CRC cobrindo os 4 bytes anteriores. Total de 64 bytes para 12 celulas.
 *
 * @note A ORDEM das celulas na rajada segue o campo PCVM_SEL do manual, que
 *       descreve 0xC como "Result of Cell 11-0" -- ou seja, decrescente, da
 *       celula 11 para a 0. Isso NAO pode ser verificado com o ladder
 *       resistivo, onde todas as celulas leem igual. Confirmar com fonte
 *       desbalanceada ou celulas reais antes de confiar na ordem.
 */
tle9012_status_t tle9012_multiread_cells_mv(uint8_t node_id,
                                            uint16_t *mv, uint8_t n_cells);

/**
 * @brief Le as tensoes de celula ja convertidas para milivolts.
 * @param mv       vetor de saida com @p n_cells posicoes.
 * @note  Chamar tle9012_start_measurement() e aguardar a conversao antes.
 */
tle9012_status_t tle9012_read_cells_mv(uint8_t node_id, uint16_t *mv, uint8_t n_cells);

/** Converte o valor bruto de um registrador PCVM para milivolts. */
uint16_t tle9012_raw_to_mv(uint16_t raw);

#endif /* TLE9012_H */
