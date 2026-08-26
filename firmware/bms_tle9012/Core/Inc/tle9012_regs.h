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

/**
 * Bit PCVM_START do MEAS_CTRL (secao 4.23, bit 15, tipo rwh).
 * O hardware limpa este bit quando a conversao termina -- e por isso que da
 * para fazer polling em vez de esperar um tempo fixo chutado.
 */
#define TLE9012_MEAS_CTRL_PCVM_START  0x8000u

/**
 * MULTI_READ_CFG.PCVM_SEL = 0xC (RES_CELL_11_0): traz o resultado das 12
 * celulas, da 11 a 0, numa unica transacao (secao 4.37).
 * Valores 0xD a 0xF nao produzem resultado de PCVM.
 */
#define TLE9012_MULTIREAD_CFG_ALL_CELLS  0x000Cu

/* --- Temperatura --------------------------------------------------------- */

/** TEMP_CONF.NR_TEMP_SENSE, bits 14:12 (secao 4.6). 101b = TMP0 a TMP4. */
#define TLE9012_TEMP_CONF_NR_SHIFT    12u
#define TLE9012_TEMP_CONF_NR_ALL5     5u

/** TEMP_CONF.I_NTC, bits 11:10 -- fonte de corrente usada na medicao. */
#define TLE9012_TEMP_CONF_INTC_SHIFT  10u

/** TEMP_CONF.EXT_OT_THR, bits 9:0. Sobretemperatura e detectada quando o
 *  resultado fica ABAIXO do limiar, porque a resistencia do NTC CAI com o
 *  aumento de temperatura. Zero desativa. */
#define TLE9012_TEMP_CONF_OT_MASK     0x03FFu

/* Campos do EXT_TEMP_z (secao 4.29). */
#define TLE9012_TEMP_RESULT_MASK      0x03FFu  /* bits 9:0                    */
#define TLE9012_TEMP_INTC_SHIFT       10u      /* bits 11:10                  */
#define TLE9012_TEMP_INTC_MASK        0x0003u
#define TLE9012_TEMP_PULLDOWN_BIT     0x1000u  /* bit 12                      */
#define TLE9012_TEMP_VALID_BIT        0x2000u  /* bit 13, limpo ao ler        */
#define TLE9012_TEMP_PD_ERR_BIT       0x4000u  /* bit 14                      */

/**
 * RR_CONFIG com RR_SYNC = 1 (bit 7), preservando o restante do valor de reset
 * (0x8024). Faz o round robin disparar a cada escrita no WD_CNT -- ou seja, o
 * proprio kick do watchdog passa a agendar a medicao de temperatura.
 */
#define TLE9012_RR_CONFIG_SYNC_ON     0x80A4u

/* --- Thresholds de tensao ------------------------------------------------ */

/**
 * Layout de OL_OV_THR (0x02) e OL_UV_THR (0x03), Tabelas 13 a 16:
 *   bits [9:0]   OV_THR / UV_THR, em passos de V_OVUV_LSB
 *   bits [15:10] OL_THR_MAX / OL_THR_MIN, em passos de 20 mV
 *
 * V_OVUV_LSB = 5 V / 1024 = 4,8828 mV. Confere com o exemplo do manual:
 * 4,6 V / 4,8828 mV = 942 = 0x3AE, e 0xFFAE tem 0x3AE nos 10 bits baixos.
 */
#define TLE9012_OVUV_THR_MASK         0x03FFu
#define TLE9012_OL_THR_SHIFT          10u
#define TLE9012_OL_THR_MASK           0x003Fu
#define TLE9012_OVUV_FSR_MV           5000u   /* fundo de escala, 10 bits     */
#define TLE9012_OL_THR_LSB_MV         20u     /* passo do threshold de OL      */

/* --- Bits do GEN_DIAG (0x0B), secao 4.11 --------------------------------- */

#define TLE9012_DIAG_UART_WAKEUP      0x0001u  /* bit 0,  somente leitura     */
#define TLE9012_DIAG_MOT_MOB_N        0x0002u  /* bit 1,  0=PoB, 1=PoT        */
#define TLE9012_DIAG_BAL_ACTIVE       0x0004u  /* bit 2                       */
#define TLE9012_DIAG_LOCK_MEAS        0x0008u  /* bit 3,  medicao em curso    */
#define TLE9012_DIAG_RR_ACTIVE        0x0010u  /* bit 4                       */
#define TLE9012_DIAG_PS_ERR_SLEEP     0x0020u  /* bit 5                       */
#define TLE9012_DIAG_ADC_ERR          0x0040u  /* bit 6                       */
#define TLE9012_DIAG_OL_ERR           0x0080u  /* bit 7                       */
#define TLE9012_DIAG_INT_IC_ERR       0x0100u  /* bit 8                       */
#define TLE9012_DIAG_REG_CRC_ERR      0x0200u  /* bit 9                       */
#define TLE9012_DIAG_EXT_T_ERR        0x0400u  /* bit 10                      */
#define TLE9012_DIAG_INT_OT           0x0800u  /* bit 11                      */
#define TLE9012_DIAG_CELL_UV          0x1000u  /* bit 12                      */
#define TLE9012_DIAG_CELL_OV          0x2000u  /* bit 13                      */
#define TLE9012_DIAG_BAL_ERR_UC       0x4000u  /* bit 14                      */
#define TLE9012_DIAG_BAL_ERR_OC       0x8000u  /* bit 15                      */

/** Bits de falha do GEN_DIAG -- os bits 0 a 4 sao status, nao erro. */
#define TLE9012_DIAG_FAULT_MASK       0xFFE0u

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

/**
 * OP_MODE com SLEEP_REG_RESET = 1 (Tabela 26).
 *
 * Reseta TODOS os registradores -- inclusive os alimentados em sleep -- e
 * coloca o dispositivo para dormir. Ao acordar, ele volta com NODE_ID = 0.
 * E o unico jeito documentado de recuperar um IC que ficou com NODE_ID
 * antigo apos um ciclo de energia incompleto.
 */
#define TLE9012_OP_MODE_SLEEP_REG_RESET  0xC404u

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
