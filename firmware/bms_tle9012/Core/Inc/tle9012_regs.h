/**
 * @file    tle9012_regs.h
 * @brief   Enderecos de registrador do TLE9012DQU.
 *
 * Apenas enderecos com confirmacao direta no user manual da Infineon
 * (Z8F80064984 Rev. 1.0) estao listados aqui, com a referencia ao lado.
 * Para os demais, consultar a secao 4.2 "Registers overview" do manual --
 * nao adivinhar enderecos.
 */

#ifndef TLE9012_REGS_H
#define TLE9012_REGS_H

/* --- Configuracao ------------------------------------------------------- */
#define TLE9012_REG_PART_CONFIG   0x01u  /* Tabela 12  - celulas ativas       */
#define TLE9012_REG_OL_OV_THR     0x02u  /* Tabela 13  - threshold sobretensao*/
#define TLE9012_REG_OL_UV_THR     0x03u  /* Tabela 14  - threshold subtensao  */
#define TLE9012_REG_TEMP_CONF     0x04u  /* Tabela 17  - sensores NTC         */
#define TLE9012_REG_BAL_CURR_THR  0x15u  /* Tabela 18  - correntes de balanc. */
#define TLE9012_REG_AVM_CONFIG    0x17u  /* Tabela 22  - medida auxiliar      */
#define TLE9012_REG_MEAS_CTRL     0x18u  /* Tabela 19  - controle de medicao  */
#define TLE9012_REG_CONFIG        0x36u  /* Tabela 7   - NODE_ID              */

/* --- Resultados --------------------------------------------------------- */
#define TLE9012_REG_PCVM_0        0x19u  /* Secao 3.4.1 - PCVM_0..PCVM_11     */
#define TLE9012_REG_PCVM_11       0x24u  /*               ficam em 0x19..0x24 */
#define TLE9012_REG_BVM           0x28u  /* Secao 3.3.2 - tensao de bloco     */

/* --- Valores de campo --------------------------------------------------- */

/** PART_CONFIG para habilitar as 12 celulas (Tabela 12). */
#define TLE9012_PART_CONFIG_12CELLS   0x0FFFu

/** MEAS_CTRL: PCVM_START = 1, CVM_MODE = 110b (16 bits). Tabela 19. */
#define TLE9012_MEAS_CTRL_PCVM_16BIT  0xE021u

/** Bit FN (Final Node) no campo de dados do CONFIG. Tabela 10: 0x0804 = FN + ID 4. */
#define TLE9012_CONFIG_FINAL_NODE     0x0800u

/** Fundo de escala do ADC de celula, em milivolts (secao 3.4.1: FSR = 5,0 V). */
#define TLE9012_PCVM_FSR_MV           5000u

#endif /* TLE9012_REGS_H */
