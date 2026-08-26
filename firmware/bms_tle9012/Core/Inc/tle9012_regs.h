/**
 * @file    tle9012_regs.h
 * @brief   Enderecos de registrador do TLE9012DQU.
 *
 * Enderecos conforme a Tabela 27 "Registers overview - REG (ascending offset
 * address)" do user manual da Infineon (Z8F80064984 Rev. 1.0), p. 61-63.
 */

#ifndef TLE9012_REGS_H
#define TLE9012_REGS_H

/* --- Configuracao ------------------------------------------------------- */
#define TLE9012_REG_PART_CONFIG   0x01u  /* celulas ativas na medicao         */
#define TLE9012_REG_OL_OV_THR     0x02u  /* threshold de sobretensao          */
#define TLE9012_REG_OL_UV_THR     0x03u  /* threshold de subtensao            */
#define TLE9012_REG_TEMP_CONF     0x04u  /* sensores NTC externos             */
#define TLE9012_REG_INT_OT_CONF   0x05u  /* temperatura interna               */
#define TLE9012_REG_RR_ERR_CNT    0x08u  /* contadores de erro do round robin */
#define TLE9012_REG_RR_CONFIG     0x09u  /* configuracao do round robin       */
#define TLE9012_REG_FAULT_MASK    0x0Au  /* mascara do pino ERR / EMM         */
#define TLE9012_REG_OP_MODE       0x14u  /* sleep / reset de registradores    */
#define TLE9012_REG_BAL_CURR_THR  0x15u  /* thresholds de corrente de balanc. */
#define TLE9012_REG_BAL_SETTINGS  0x16u  /* drivers de balanceamento          */
#define TLE9012_REG_AVM_CONFIG    0x17u  /* medicao auxiliar                  */
#define TLE9012_REG_MEAS_CTRL     0x18u  /* controle de medicao               */
#define TLE9012_REG_MULTI_READ    0x31u  /* comando multiread                 */
#define TLE9012_REG_MULTI_READ_CFG 0x32u /* configuracao do multiread         */
#define TLE9012_REG_CONFIG        0x36u  /* NODE_ID e final node              */
#define TLE9012_REG_WDOG_CNT      0x3Du  /* contador do watchdog              */

/* --- Diagnostico -------------------------------------------------------- */
#define TLE9012_REG_GEN_DIAG      0x0Bu  /* diagnostico geral                 */
#define TLE9012_REG_CELL_UV       0x0Cu  /* flags de subtensao                */
#define TLE9012_REG_CELL_OV       0x0Du  /* flags de sobretensao              */
#define TLE9012_REG_EXT_TEMP_DIAG 0x0Eu  /* flags de sobretemperatura         */
#define TLE9012_REG_DIAG_OL       0x10u  /* diagnostico de open load          */
#define TLE9012_REG_REG_CRC_ERR   0x11u  /* erro de CRC de registrador        */
#define TLE9012_REG_ICVID         0x39u  /* versao do IC e ID de fabricacao   */

/* --- Resultados --------------------------------------------------------- */
#define TLE9012_REG_PCVM_0        0x19u  /* PCVM_i = 0x19 + i, i de 0 a 11    */
#define TLE9012_REG_PCVM_11       0x24u
#define TLE9012_REG_SCVM_HIGH     0x25u  /* maior tensao de celula            */
#define TLE9012_REG_SCVM_LOW      0x26u  /* menor tensao de celula            */
#define TLE9012_REG_BVM           0x28u  /* tensao de bloco                   */
#define TLE9012_REG_EXT_TEMP_0    0x29u  /* EXT_TEMP_z = 0x29 + z, z de 0 a 4 */
#define TLE9012_REG_EXT_TEMP_4    0x2Du
#define TLE9012_REG_INT_TEMP      0x30u  /* temperatura do chip               */

/* --- Valores de campo --------------------------------------------------- */

/** PART_CONFIG para habilitar as 12 celulas (Tabela 12). */
#define TLE9012_PART_CONFIG_12CELLS   0x0FFFu

/** MEAS_CTRL: PCVM_START = 1, CVM_MODE = 110b (16 bits). Tabela 19. */
#define TLE9012_MEAS_CTRL_PCVM_16BIT  0xE021u

/** Bit FN (Final Node) no campo de dados do CONFIG. Tabela 10: 0x0804 = FN + ID 4. */
#define TLE9012_CONFIG_FINAL_NODE     0x0800u

/**
 * Valor de recarga do watchdog (WDOG_CNT, secao 4.48).
 *
 * O campo WD_CNT ocupa os bits 6:0 e decrementa sozinho. Ao chegar em 0x00 o
 * dispositivo ENTRA EM SLEEP e o NODE_ID volta a 0 -- a partir dai toda
 * transacao endereçada ao no antigo da timeout. 0x7F e o valor de reset, o
 * maximo. MAIN_CNT (bits 15:7) e somente leitura e deve ser escrito com zero.
 *
 * NUNCA escrever 0x0000 aqui: isso manda o dispositivo dormir imediatamente.
 */
#define TLE9012_WDOG_CNT_RELOAD       0x007Fu

/** Fundo de escala do ADC de celula, em milivolts (secao 3.4.1: FSR = 5,0 V). */
#define TLE9012_PCVM_FSR_MV           5000u

/**
 * Desativa o diagnostico de open load (secao 1.1.2, nota da Figura 2).
 *
 * Com o ladder resistivo da placa de avaliacao o open load e detectado
 * FALSAMENTE, porque a resistencia do divisor e muito maior que a resistencia
 * interna de uma celula real. O manual manda zerar OL_THR_MAX e OL_THR_MIN.
 * Ao migrar para celulas reais, remover isto e configurar os thresholds.
 */
#define TLE9012_OL_THR_DISABLED       0x0000u

#endif /* TLE9012_REGS_H */
