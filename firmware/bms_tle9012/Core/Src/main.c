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

#include <math.h>   /* logf, apenas na conversao NTC para graus */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* As structs de estado sao definidas apos os enums e defines de que dependem;
 * ver a secao "Estado agrupado" logo antes das variaveis privadas. */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMS_NODE_ID        1u     /* unico TLE9012 da cadeia                  */
#define BMS_NUM_CELLS      12u
#define BMS_MEAS_PERIOD_MS 100u

/* Falhas consecutivas ate declarar a cadeia caida e reinicializar. Tres
 * ciclos = 300 ms sem leitura valida, tempo de sobra para distinguir um
 * glitch pontual de um link que caiu. */
#define BMS_MAX_CONSEC_FAIL 3u

/* --- Protecoes ----------------------------------------------------------- */

/* Limiares de tensao de celula, em mV.
 *
 * ATENCAO: estes sao valores de BANCADA. Com o ladder resistivo em 12 V cada
 * "celula" le ~1056 mV, entao limiares reais de Li-ion (2500/4200) fariam a
 * subtensao disparar continuamente.
 *
 * Ao migrar para celulas reais:  UV = 2500,  OV = 4200. */
#define BMS_UV_THR_MV       500u
#define BMS_OV_THR_MV       2000u

/**
 * Histerese das falhas de tensao, em mV.
 *
 * Os flags CELL_UV e CELL_OV do TLE9012 sao latching (tipo rocw): uma vez
 * setados, so saem quando alguem escreve zero. Ou seja, QUANDO limpar e
 * decisao do firmware -- e limpar assim que a tensao volta a cruzar o limiar
 * produziria oscilacao continua com a celula parada em cima dele.
 *
 * Com histerese:
 *   UV limpa so quando a celula sobe acima de  (UV_THR + 200 mV)
 *   OV limpa so quando a celula desce abaixo de (OV_THR - 200 mV)
 *
 * A falha SEMPRE seta no limiar exato -- a histerese atrasa apenas a
 * recuperacao, nunca a deteccao. Numa protecao, errar para o lado de
 * permanecer em falha e o lado seguro.
 */
#define BMS_FAULT_HYST_MV   200u

/* Open load desativado: obrigatorio com ladder resistivo, que dispara open
 * load falso. Ao usar celulas reais, calcular conforme a secao 3.2.3. */
#define BMS_OL_MIN_MV       0u
#define BMS_OL_MAX_MV       0u

/* --- Shutdown circuit ----------------------------------------------------- */

/**
 * GPIO que comanda o MOSFET do shutdown circuit.
 *
 * POLARIDADE OBRIGATORIA -- nivel ALTO fecha o circuito (veiculo liberado),
 * nivel BAIXO abre (estado seguro).
 *
 * O motivo e que o estado seguro precisa ser o estado NAO ENERGIZADO. Com
 * pull-down externo no gate, MCU desligado, em reset ou com o pino em alta
 * impedancia deixa o circuito ABERTO. A polaridade inversa produziria um
 * sistema em que MCU morto = veiculo liberado sem protecao nenhuma.
 *
 * O pull-down externo nao e opcional: sem ele, o pino flutuante durante o
 * reset pode fechar o MOSFET por acaso.
 */
#define BMS_SHUTDOWN_PORT       GPIOB
#define BMS_SHUTDOWN_PIN        GPIO_PIN_1
#define BMS_SHUTDOWN_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

/**
 * Modo de acionamento do shutdown.
 *
 * 0 = NIVEL ESTATICO. Simples, mas tem um furo: um MCU TRAVADO continua
 *     segurando o pino em alto, e o circuito permanece fechado. Travamento
 *     de firmware nao abre o shutdown.
 *
 * 1 = ONDA QUADRADA (kick). O pino alterna a cada ciclo enquanto tudo estiver
 *     ok. Exige um retificador/RC externo no gate: enquanto houver pulsos o
 *     capacitor se mantem carregado e o MOSFET conduz; se os pulsos pararem
 *     -- por travamento, reset ou perda de alimentacao -- o RC descarrega e o
 *     circuito abre sozinho.
 *
 * O modo 1 e o correto para um AMS, porque cobre a falha do proprio
 * supervisor. Mantido em 0 por enquanto para casar com o hardware descrito.
 */
#define BMS_SHUTDOWN_TOGGLE     0

/* --- Codigo de falha ------------------------------------------------------ */

/**
 * Falha ativa mais severa, em forma simbolica.
 *
 * Existe porque as mascaras de bit sao ilegiveis em depurador: o Live
 * Expressions do CubeIDE mostra o NOME do enumerado quando a variavel e de
 * tipo enum, entao "BMS_FAULT_CELL_OV" aparece no lugar de "0x2000".
 *
 * A ordem e de prioridade decrescente -- a primeira condicao verdadeira
 * ganha. Perda de comunicacao vem antes de tudo porque, sem dado, nenhuma
 * outra protecao significa nada. Sobretensao vem antes de subtensao porque
 * sobrecarga tem consequencia termica, e subtensao so degrada.
 */
typedef enum
{
  BMS_FAULT_NONE = 0,
  BMS_FAULT_CHAIN_DOWN,     /**< cadeia caida, sem comunicacao              */
  BMS_FAULT_DATA_STALE,     /**< medicao obsoleta                           */
  BMS_FAULT_CELL_OV,        /**< sobretensao de celula                      */
  BMS_FAULT_CELL_UV,        /**< subtensao de celula                        */
  BMS_FAULT_EXT_OVERTEMP,   /**< sobretemperatura externa (NTC)             */
  BMS_FAULT_INT_OVERTEMP,   /**< sobretemperatura interna do CI             */
  BMS_FAULT_UNDERTEMP,      /**< subtemperatura -- deteccao em firmware     */
  BMS_FAULT_OPEN_LOAD,      /**< fio de sensoriamento aberto                */
  BMS_FAULT_ADC,            /**< soma dos PCVM diverge do BVM               */
  BMS_FAULT_INTERNAL_IC,    /**< erro interno do TLE9012                    */
  BMS_FAULT_REG_CRC,        /**< CRC de registrador                         */
  BMS_FAULT_PS_SLEEP,       /**< sleep por erro de alimentacao              */
  BMS_FAULT_BALANCING       /**< corrente de balanceamento fora do previsto */
} bms_fault_code_t;

/* --- Simulacao de falhas (demonstracao) ---------------------------------- */

/**
 * Modos de injecao de falha, para demonstracao em inspecao tecnica.
 *
 * IMPORTANTE: nenhum destes modos falsifica flag. Todos criam uma CONDICAO
 * que o TLE9012 detecta de verdade -- o caminho de deteccao, os registradores
 * de diagnostico e o intertravamento que desliga o balanceamento sao os
 * mesmos de uma falha real. O que muda e a origem da condicao, nao a
 * deteccao. Forjar os flags mostraria o LED aceso sem provar nada.
 */
typedef enum
{
  BMS_SIM_NONE = 0,       /**< operacao normal                              */
  BMS_SIM_UNDERVOLTAGE,   /**< limiar de UV acima da tensao real            */
  BMS_SIM_OVERVOLTAGE,    /**< limiar de OV abaixo da tensao real           */
  BMS_SIM_OVERTEMP,       /**< limiar de OT acima da leitura real do NTC    */
  BMS_SIM_OPEN_LOAD,      /**< diagnostico de open load com ladder resistivo */
  BMS_SIM_COMM_LOSS,      /**< poe o TLE9012 em sleep: perda de comunicacao */
  BMS_SIM_UNDERTEMP,      /**< limiar de UT abaixo da resistencia real      */
  BMS_SIM_COUNT
} bms_sim_fault_t;

/* Margem entre a tensao medida e o limiar injetado. Precisa ser maior que a
 * dispersao entre celulas para a falha pegar todas de uma vez. */
#define BMS_SIM_MARGIN_MV   200u

/* Margem da injecao de sobretemperatura, em contagens de RESULT. 50 contagens
 * equivalem a uns 7 C na faixa de operacao -- o bastante para disparar com
 * folga e ainda mostrar um limiar plausivel em graus. */
#define BMS_SIM_OT_MARGIN   50u

/* --- Temperatura --------------------------------------------------------- */

#define BMS_NUM_TEMP        5u    /* tamanho dos vetores: maximo do CI        */

/**
 * Quantos canais de temperatura estao REALMENTE ligados.
 *
 * Precisa bater com o hardware. Configurar canais sem NTC faz o teste de
 * mux do TLE9012 falhar neles, o que seta PD_ERR no EXT_TEMP_z -- e PD_ERR
 * esta ligado ao EXT_T_ERR do GEN_DIAG. O sintoma e sobretemperatura
 * disparando na inicializacao, independentemente do limiar de OT.
 *
 * Aumentar conforme forem sendo plugados NTCs, sempre a partir do TMP0:
 * o campo NR_TEMP_SENSE ativa uma sequencia, nao canais avulsos.
 */
#define BMS_NUM_TEMP_USED   1u
/**
 * Fonte de corrente para a comparacao de sobretemperatura (TEMP_CONF.I_NTC).
 *
 * TEM QUE BATER com a fonte que a MEDICAO usa, lida em EXT_TEMP_z.INTC.
 * Sao campos distintos: I_NTC escolhe a fonte da comparacao de OT, INTC
 * informa a que foi usada na conversao. Se divergirem, o limiar e comparado
 * contra um valor com escala diferente -- e a escala muda por 4 a cada nivel.
 *
 * Medido em bancada: EXT_TEMP_0 = 0x2244 -> INTC = 0. Por isso 0 aqui.
 * Com I_NTC = 1 e INTC = 0, a comparacao usava um RESULT 4x menor que o
 * reportado e a sobretemperatura disparava com o NTC a 24 C.
 *
 * Se trocar NTC ou resistores e o INTC mudar, este valor muda junto.
 */
#define BMS_TEMP_INTC       0u
/**
 * Limiar de SOBRETEMPERATURA, em codigo bruto de 10 bits (EXT_OT_THR).
 *
 * Ao contrario da subtemperatura, esta protecao e do HARDWARE: o TLE9012
 * compara sozinho e seta EXT_T_ERR no GEN_DIAG, sem depender do firmware.
 *
 * Dispara quando o RESULT do ADC fica ABAIXO do limiar, porque a resistencia
 * do NTC cai conforme a temperatura sobe. Zero desativa.
 *
 * 292 = 60 C, para a cadeia atual, com INTC = 0 MEDIDO em bancada
 * (EXT_TEMP_0 = 0x2244: VALID=1, PD_ERR=0, INTC=0, RESULT=580):
 *   60 C -> R_NTC 2486 ohm
 *        -> em paralelo com os 5197 da placa = 1682 ohm
 *        -> mais R_TMP de 100 ohm            = 1782 ohm
 *        -> RESULT = 1782 * 4096 / (25000 * 4^0) = 292
 *
 * Referencia rapida:  40 C = 446 | 50 C = 364 | 60 C = 292
 *                     70 C = 232 | 80 C = 184
 *
 * ATENCAO: o RESULT escala por 4 a cada nivel de INTC. Uma versao anterior
 * usava 73, calculado assumindo INTC = 1 sem verificar -- erro de fator 4.
 * TEMP_CONF.I_NTC seleciona a fonte usada para a FALHA de OT, enquanto
 * EXT_TEMP_z.INTC informa a que foi usada na MEDICAO; nao sao a mesma coisa.
 * Se trocar o NTC, o resistor da placa ou o numero de canais, RELER
 * temp.intc antes de recalcular.
 *
 * DEPENDE tambem de B = 3950, ainda nao confirmado no componente.
 */
#define BMS_TEMP_OT_THR     292u

/**
 * Limiar de RETORNO da sobretemperatura, com 5 C de histerese.
 *
 * 327 = 55 C. Como o NTC tem coeficiente negativo, esfriar AUMENTA o RESULT:
 * a falha seta quando RESULT cai abaixo de 292 e so sai quando sobe acima
 * de 327 -- uma margem de 35 contagens.
 *
 * A deteccao acontece no hardware, mas o flag e latching (rocw), entao QUANDO
 * limpar e decisao do firmware -- mesma logica da histerese de tensao. Sem
 * ela, um NTC parado em 60 C faria a falha piscar continuamente.
 *
 * Referencia:  45 C = 404 | 50 C = 364 | 55 C = 327
 *              60 C = 292 | 65 C = 260 | 70 C = 232
 */
#define BMS_TEMP_OT_CLEAR   327u

/* Le temperatura a cada 10 ciclos de tensao (1 s). O round robin faz no
 * maximo duas medicoes de temperatura por ciclo, entao nao adianta ler mais
 * rapido -- e temperatura de celula nao muda em 100 ms. */
#define BMS_TEMP_EVERY_N    10u

/* Resistor de referencia em serie com os NTCs (R_TMP no esquematico).
 * O manual usa 100 ohm no exemplo da secao 3.4.2. CONFERIR NA PLACA. */
#define BMS_TEMP_R_SERIES   100u

/* Resistencia populada na placa em paralelo com o NTC externo (posicoes
 * NTC0..NTC4 da Figura 18). Medida em bancada: ~5,2 kohm nos cinco canais.
 * Zero desativa a compensacao.
 *
 * Isto e um PALIATIVO. Dessoldar o componente da placa da leitura limpa; a
 * compensacao amplifica erro em temperaturas baixas, onde R_medido se
 * aproxima deste valor. */
#define BMS_TEMP_R_PARALLEL 5197u

/* Parametros do NTC (MF52A), usados APENAS na conversao para graus, que serve
 * para conferencia visual. As protecoes comparam em ohms e nao dependem disto.
 *
 * MF52A e nome de familia, nao de valor: existem variantes de 1k a 100k e
 * varios B. A mais comum e 10 kohm com B = 3950 -- e o que esta aqui.
 * CONFERIR no encapsulamento ou medindo com multimetro a temperatura conhecida.
 *
 * B errado nao desloca a leitura a 25 C (os dois ancoram em R25), mas afasta
 * progressivamente conforme sai dessa temperatura -- justamente na faixa em
 * que um pack opera. Vira parametro da tabela em flash na fase 3. */
#define BMS_NTC_R25_OHM     10000.0f
#define BMS_NTC_BETA        3950.0f

/**
 * Limiar de SUBTEMPERATURA, em ohms do NTC.
 *
 * O TLE9012 NAO detecta subtemperatura -- ele so tem EXT_OT_THR, para o lado
 * quente. Nao existe registrador de limiar frio. Portanto esta protecao e
 * feita em FIRMWARE, comparando a resistencia medida.
 *
 * Como o NTC tem coeficiente negativo, mais frio significa MAIOR resistencia:
 * a falha e temp.ohms > limiar.
 *
 * 25925 ohm = 5 C para um MF52A de 10 kohm com B = 3950. Recalcular se trocar
 * de NTC -- este numero depende inteiramente de R25 e B.
 *
 * Por que 5 C importa: carregar celula de litio perto de 0 C causa
 * lithium plating, que degrada de forma permanente e cria risco de curto
 * interno. O limiar de carga costuma ser mais restritivo que o de descarga.
 */
#define BMS_UT_THR_OHM      25925uL

/**
 * Limite de confianca da compensacao de paralelo, em porcentagem de
 * R_PARALLEL.
 *
 * A compensacao divide por (R_paralelo - R_medido). Quando os dois se
 * aproximam, o denominador tende a zero e ruido de poucos ohms vira centenas
 * de kohm. Acima deste percentual o resultado nao e confiavel e o canal e
 * marcado invalido, em vez de reportar um numero inventado.
 *
 * 90% de 5197 = 4677 ohm, que corresponde a cerca de -5 C. Abaixo disso a
 * medicao com resistor em paralelo nao serve mesmo -- a solucao e dessoldar
 * o componente da placa, nao afrouxar este limite.
 */
#define BMS_TEMP_TRUST_PCT  90u

/* Tentativas de polling do PCVM_START antes de desistir da conversao. Cada
 * tentativa custa uma leitura (~50 us) mais 100 us de espera, entao 100
 * tentativas dao ~15 ms de teto -- folga larga sobre o tempo real. */
#define BMS_CONV_MAX_POLLS 100u
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
/* ==========================================================================
 * Estado agrupado
 * ========================================================================== */

/** Estado da cadeia e do smoke test. */
typedef struct
{
  uint8_t  ready;           /**< 1 = cadeia inicializada e respondendo       */
  uint8_t  smoke_ok;        /**< 1 = link validado de ponta a ponta          */
  uint8_t  smoke_status;    /**< tle9012_status_t do readback                */
  uint16_t smoke_config;    /**< valor lido do CONFIG (0x36)                 */
  uint32_t smoke_attempts;  /**< tentativas de subir a cadeia                */
  uint8_t  diag_status;     /**< tle9012_status_t da leitura do GEN_DIAG     */
} bms_link_t;

/** Tensoes de celula. */
typedef struct
{
  uint16_t cell_mv[BMS_NUM_CELLS];  /**< tensao de cada celula, em mV        */
  uint16_t min_mv;
  uint16_t max_mv;
  uint16_t delta_mv;                /**< desbalanco: max - min               */
  uint8_t  valid;                   /**< 0 = cell_mv e historico, nao medicao */
  uint32_t age_ms;                  /**< ms desde a ultima medicao boa       */
  uint32_t count;                   /**< ciclos bem-sucedidos                */
  uint32_t fail_count;
  uint8_t  status;                  /**< tle9012_status_t da ultima transacao */
} bms_meas_t;

/** Temperaturas dos NTCs externos. */
typedef struct
{
  uint32_t ohms[BMS_NUM_TEMP];      /**< NTC isolado -- use este nas protecoes */
  uint32_t ohms_raw[BMS_NUM_TEMP];  /**< o que o ADC ve, com o paralelo da placa */
  float    celsius[BMS_NUM_TEMP];   /**< so para leitura humana              */
  uint8_t  valid[BMS_NUM_TEMP];

  /* Cru do registrador. Necessario para calcular o limiar de OT, que e
   * comparado contra o RESULT e nao contra ohms -- e o RESULT depende da
   * fonte de corrente que o dispositivo escolheu, que muda a escala por 4. */
  uint16_t reg[BMS_NUM_TEMP];       /**< EXT_TEMP_z inteiro, sem interpretar */
  uint16_t result[BMS_NUM_TEMP];    /**< RESULT, bits 9:0                    */
  uint8_t  intc[BMS_NUM_TEMP];      /**< INTC, fonte de corrente usada       */
  uint16_t conf_readback;           /**< TEMP_CONF relido -- a config pegou? */
  uint16_t ext_diag;                /**< EXT_TEMP_DIAG (0x0E)                */
  uint32_t count;
  uint32_t fail_count;
  uint8_t  status;
} bms_temp_t;

/**
 * Falhas, limiares em vigor e historico.
 *
 * Os limiares moram aqui de proposito: numa demonstracao, mostrar o limiar ao
 * lado da medicao e do veredito -- tudo numa expansao so -- e o que torna a
 * prova convincente. Separa-los obrigaria a abrir tres expressoes.
 */
typedef struct
{
  /* --- Resumo --- */
  bms_fault_code_t code;        /**< falha mais severa ativa, em texto       */
  uint8_t          node;        /**< qual escravo da cadeia                  */
  uint8_t          cell;        /**< primeira celula afetada, 0xFF se n/a    */
  uint8_t          cell_count;  /**< quantas ao mesmo tempo                  */

  /* --- Mascaras apos histerese: e o que vale --- */
  uint16_t uv;                  /**< bit i = celula i em subtensao           */
  uint16_t ov;
  uint8_t  ut;                  /**< bit z = canal z frio demais (firmware)  */
  uint8_t  ot;                  /**< bit z = canal z quente demais           */

  /* Diagnostico do fio do NTC, do EXT_TEMP_DIAG (0x0E). */
  uint8_t  temp_open;           /**< bit z = fio aberto no canal z           */
  uint8_t  temp_short;          /**< bit z = curto no canal z                */

  /* --- Cru do CI, latching --- */
  uint16_t gen_diag;
  uint16_t uv_raw;
  uint16_t ov_raw;

  /* --- Limiares em vigor --- */
  uint16_t uv_thr_mv;
  uint16_t ov_thr_mv;
  uint16_t hyst_mv;
  uint32_t ut_thr_ohm;
  uint8_t  ut_enabled;          /**< 0 = protecao de frio desligada          */
  uint16_t ot_thr;              /**< dispara: RESULT abaixo disto            */
  uint16_t ot_clear_thr;        /**< retorna: RESULT acima disto (5 C acima) */

  /* Os mesmos limiares em graus, para leitura humana. Recalculados a cada
   * ciclo a partir dos valores brutos em vigor -- entao acompanham a injecao
   * de falha, que e justamente o que se quer mostrar numa demonstracao. */
  float    ot_thr_c;
  float    ot_clear_c;
  float    ut_thr_c;

  /* --- Historico --- */
  uint16_t latched;             /**< acumula desde o ultimo clear            */
  uint32_t count;               /**< ciclos com alguma falha                 */
  uint8_t  read_status;

  /**
   * Escrever 1 pelo depurador limpa as falhas latcheadas no dispositivo.
   *
   * Existe porque bms_chain_init() so roda quando a cadeia esta caida: uma
   * falha que trave DEPOIS da inicializacao fica latcheada para sempre, sem
   * caminho de limpeza. Varias falhas desativam o balanceamento, e algumas
   * podem afetar o round robin -- sair desse estado precisa ser possivel sem
   * reiniciar o firmware.
   */
  uint8_t clear_request;
} bms_fault_t;

/** Valores iniciais dos campos que nao comecam em zero. */
#define BMS_FAULT_INIT { .node       = BMS_NODE_ID,     \
                         .cell       = 0xFFu,           \
                         .uv_thr_mv  = BMS_UV_THR_MV,   \
                         .ov_thr_mv  = BMS_OV_THR_MV,   \
                         .hyst_mv    = BMS_FAULT_HYST_MV, \
                         .ut_thr_ohm = BMS_UT_THR_OHM,  \
                         .ut_enabled = 0u,              \
                         .ot_thr       = BMS_TEMP_OT_THR, \
                         .ot_clear_thr = BMS_TEMP_OT_CLEAR }

/**
 * Estado do shutdown circuit.
 *
 * O travamento (latch) e deliberado: uma vez aberto por falha, o circuito
 * NAO fecha sozinho quando a condicao some. Recuperacao automatica permitiria
 * o veiculo se reenergizar sozinho apos um transiente -- inaceitavel. Sair do
 * estado exige acao explicita, que no veiculo deve ser um botao fisico.
 */
typedef struct
{
  uint8_t  closed;         /**< 1 = circuito fechado, veiculo liberado       */
  uint8_t  latched_open;   /**< 1 = travado aberto por falha                 */
  uint8_t  reset_request;  /**< escrever 1 para destravar (so sem falha)     */
  bms_fault_code_t cause;  /**< qual falha abriu, preservada apos o evento   */
  uint32_t open_count;     /**< quantas vezes abriu desde o boot             */
} bms_shutdown_t;

/** Injecao de falha para demonstracao. */
typedef struct
{
  bms_sim_fault_t request;      /**< o que se quer injetar                   */
  bms_sim_fault_t active;       /**< o que esta injetado                     */
  uint8_t         apply_status;
} bms_sim_t;

/* Estado do BMS, agrupado em quatro structs.
 *
 * volatile e global de proposito: e assim que o Live Expressions le com o
 * alvo rodando. Agrupar em struct significa uma expressao por assunto no
 * depurador, em vez de vinte variaveis soltas -- e a expansao mostra tudo
 * junto, o que importa quando alguem esta olhando por cima do ombro.
 *
 *   link   estado da cadeia e do smoke test
 *   meas   tensoes de celula
 *   temp   temperaturas
 *   fault  falhas, limiares em vigor e historico
 *   sim    injecao de falha para demonstracao
 */
volatile bms_link_t  link;
volatile bms_meas_t  meas;
volatile bms_temp_t  temp;
volatile bms_fault_t fault = BMS_FAULT_INIT;
volatile bms_sim_t   sim;
volatile bms_shutdown_t shutdown;

static uint32_t s_last_good_tick;
static uint8_t  s_consec_fail;

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
  link.smoke_attempts++;
  link.smoke_ok = 0u;

  /* Devolve o IC a NODE_ID = 0 antes de tentar atribuir. Sem isto, um IC que
   * manteve o ID de uma sessao anterior nunca mais e alcancado: a atribuicao
   * escreve no no 0, onde nao ha ninguem. E o que impedia o sistema de voltar
   * sozinho depois de desligar e religar. Inofensivo quando o IC ja esta em 0. */
  tle9012_force_reset(BMS_NODE_ID);

  /* Unico IC na cadeia, portanto ele e tambem o final node. Sem o bit FN
   * o broadcast nunca responde e tudo da timeout. */
  tle9012_status_t st = tle9012_assign_node_id(BMS_NODE_ID, true);

  if (st != TLE9012_OK)
  {
    link.smoke_status = (uint8_t)st;
    return false;
  }

  uint16_t config = 0u;
  st = tle9012_read_reg(BMS_NODE_ID, TLE9012_REG_CONFIG, &config);

  link.smoke_status = (uint8_t)st;
  link.smoke_config = config;

  if (st != TLE9012_OK)
  {
    return false;
  }

  /* Le o diagnostico geral. Nao bloqueia a inicializacao -- serve para
   * distinguir falha latcheada no dispositivo de problema de link. */
  {
    uint16_t diag = 0u;
    link.diag_status = (uint8_t)tle9012_read_reg(BMS_NODE_ID,
                                                TLE9012_REG_GEN_DIAG, &diag);
    if (link.diag_status == (uint8_t)TLE9012_OK)
    {
      fault.gen_diag = diag;

      /* O manual (secao 3.6.2) exige limpar manualmente o PS_ERR_SLEEP, que
       * pode ficar setado apos o reset de registradores por sleep. */
      if (diag != 0u)
      {
        (void)tle9012_write_reg(BMS_NODE_ID, TLE9012_REG_GEN_DIAG, 0x0000u);
      }
    }
  }

  /* O NODE_ID atribuido deve aparecer nos bits baixos do CONFIG. */
  if ((config & 0x3Fu) != BMS_NODE_ID)
  {
    return false;
  }

  link.smoke_ok = 1u;
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

  /* Recarrega o watchdog logo apos atribuir o NODE_ID: o manual avisa que a
   * propria inicializacao pode demorar mais que o intervalo do watchdog, e
   * nesse caso o ID recem-atribuido seria perdido. */
  if (tle9012_kick_watchdog(BMS_NODE_ID) != TLE9012_OK)
  {
    return false;
  }

  /* Limiares de tensao e de open load moram nos mesmos dois registradores,
   * entao sao escritos juntos. Open load em zero desativa o diagnostico, que
   * e obrigatorio com o ladder resistivo. */
  if (tle9012_set_thresholds(BMS_NODE_ID, BMS_UV_THR_MV, BMS_OV_THR_MV,
                             BMS_OL_MIN_MV, BMS_OL_MAX_MV) != TLE9012_OK)
  {
    return false;
  }

  if (tle9012_set_cell_count(BMS_NODE_ID, BMS_NUM_CELLS) != TLE9012_OK)
  {
    return false;
  }

  /* Rajada de multiread com as 12 celulas. Broadcast e exigido pelo manual
   * para este registrador (secao 4.37). */
  if (tle9012_multiread_configure(TLE9012_MULTIREAD_CFG_ALL_CELLS) != TLE9012_OK)
  {
    return false;
  }

  if (tle9012_config_temp(BMS_NODE_ID, BMS_NUM_TEMP_USED,
                          BMS_TEMP_INTC, BMS_TEMP_OT_THR) != TLE9012_OK)
  {
    return false;
  }

  /* Amarra o round robin ao kick do watchdog, que ja roda a cada ciclo. */
  if (tle9012_sync_round_robin(BMS_NODE_ID) != TLE9012_OK)
  {
    return false;
  }

  /* Todas as falhas disparam EMM, que sobe pela iso UART ate o TLE9015 e
   * aciona ERRQ e ERR_loc_out -- caminho de protecao em hardware, que
   * continua valendo se o firmware travar. */
  if (tle9012_write_reg(BMS_NODE_ID, TLE9012_REG_FAULT_MASK,
                        TLE9012_FAULT_MASK_ALL) != TLE9012_OK)
  {
    return false;
  }

  /* Deixa o round robin completar alguns ciclos e SO ENTAO limpa as falhas.
   *
   * As primeiras conversoes apos habilitar a temperatura acontecem antes das
   * fontes de corrente e dos filtros RC assentarem, e sinalizam erro que nao
   * corresponde a condicao real nenhuma. Limpar aqui separa "falha de
   * partida" de "falha de verdade" -- sem isto, o EXT_T_ERR fica latcheado
   * desde o boot e contamina todo o diagnostico daí em diante.
   *
   * O watchdog e realimentado durante a espera porque ele nao para de contar. */
  for (uint8_t i = 0u; i < 5u; i++)
  {
    HAL_Delay(100u);
    (void)tle9012_kick_watchdog(BMS_NODE_ID);
  }

  (void)tle9012_clear_faults(BMS_NODE_ID);

  fault.latched = 0u;
  fault.uv      = 0u;
  fault.ov      = 0u;
  fault.ut      = 0u;
  fault.ot      = 0u;
  fault.temp_open  = 0u;
  fault.temp_short = 0u;

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

  /* Realimenta o watchdog ANTES de medir. Se ele zerar, o TLE9012 dorme e o
   * NODE_ID volta a 0 -- a comunicacao cai sem nenhum sintoma alem de
   * timeouts, que e facil confundir com problema de fiacao. */
  tle9012_status_t st = tle9012_kick_watchdog(BMS_NODE_ID);

  if (st == TLE9012_OK)
  {
    st = tle9012_start_measurement(BMS_NODE_ID);
  }

  /* Espera o hardware limpar PCVM_START, em vez de um atraso fixo chutado:
   * a leitura sai assim que fica pronta, e uma conversao que nao conclui
   * vira erro explicito em vez de dado lido cedo demais. */
  if (st == TLE9012_OK)
  {
    st = tle9012_wait_measurement(BMS_NODE_ID, BMS_CONV_MAX_POLLS);
  }

  /* Multiread: uma transacao de 64 bytes no lugar de doze de 9. */
  if (st == TLE9012_OK)
  {
    st = tle9012_multiread_cells_mv(BMS_NODE_ID, mv, BMS_NUM_CELLS);
  }

  meas.status = (uint8_t)st;

  if (st != TLE9012_OK)
  {
    meas.fail_count++;
    meas.valid = 0u;   /* meas.cell_mv passa a ser historico, nao medicao */

    if (s_consec_fail < 0xFFu)
    {
      s_consec_fail++;
    }

    /* Link caido: volta ao smoke test em vez de insistir para sempre num
     * canal morto. Cobre o caso de a CSC perder alimentacao ou o watchdog
     * do TLE9012 resetar o NODE_ID. */
    if (s_consec_fail >= BMS_MAX_CONSEC_FAIL)
    {
      link.ready = 0u;
    }

    return;
  }

  s_consec_fail = 0u;

  uint16_t vmin = 0xFFFFu;
  uint16_t vmax = 0u;

  for (uint8_t i = 0u; i < BMS_NUM_CELLS; i++)
  {
    meas.cell_mv[i] = mv[i];

    if (mv[i] < vmin) { vmin = mv[i]; }
    if (mv[i] > vmax) { vmax = mv[i]; }
  }

  meas.min_mv   = vmin;
  meas.max_mv   = vmax;
  meas.delta_mv = (uint16_t)(vmax - vmin);

  s_last_good_tick = HAL_GetTick();
  meas.age_ms = 0u;
  meas.valid  = 1u;
  meas.count++;
}

/**
 * @brief Converte resistencia de NTC em graus Celsius (equacao de Beta).
 * @return -999.0f se o valor nao fizer sentido.
 *
 * Apenas para leitura humana: as protecoes comparam em ohms e em RESULT.
 */
static float ohms_to_celsius(uint32_t ohms)
{
  if (ohms == 0u)
  {
    return -999.0f;
  }

  const float t = 1.0f / ((1.0f / 298.15f)
                          + (logf((float)ohms / BMS_NTC_R25_OHM) / BMS_NTC_BETA));

  return t - 273.15f;
}

/**
 * @brief Converte um codigo RESULT do ADC de temperatura em graus.
 *
 * Desfaz toda a cadeia: RESULT -> resistencia medida -> remove o paralelo da
 * placa -> equacao de Beta. Usado para exibir os limiares de sobretemperatura
 * em graus, ja que o hardware os expressa em contagens de ADC.
 */
static float result_to_celsius(uint32_t result, uint8_t intc)
{
  const uint32_t gain   = 1uL << (2u * intc);
  const uint32_t scaled = (result * gain * 25000uL) >> 12;

  if (scaled <= BMS_TEMP_R_SERIES)
  {
    return -999.0f;
  }

  const uint32_t measured = scaled - BMS_TEMP_R_SERIES;
  const uint32_t ohms = tle9012_compensate_parallel(measured,
                                                    BMS_TEMP_R_PARALLEL);

  return ohms_to_celsius(ohms);
}

/** Indice do bit menos significativo setado, ou 0xFF se a mascara for zero. */
static uint8_t first_bit(uint16_t mask)
{
  for (uint8_t i = 0u; i < 16u; i++)
  {
    if ((mask & (uint16_t)(1u << i)) != 0u)
    {
      return i;
    }
  }

  return 0xFFu;
}

/** Quantos bits setados -- quantas celulas em falha ao mesmo tempo. */
static uint8_t count_bits(uint16_t mask)
{
  uint8_t n = 0u;

  while (mask != 0u)
  {
    mask &= (uint16_t)(mask - 1u);   /* limpa o bit menos significativo */
    n++;
  }

  return n;
}

/**
 * @brief Resolve a falha ativa mais severa em codigo simbolico.
 *
 * Ordem de prioridade, nao de ocorrencia: retorna a mais grave que estiver
 * ativa neste instante.
 */
static void bms_resolve_fault_code(void)
{
  const uint16_t d = fault.gen_diag;

  fault.cell       = 0xFFu;
  fault.cell_count = 0u;

  if (link.ready == 0u)          { fault.code = BMS_FAULT_CHAIN_DOWN;  return; }
  if (meas.valid == 0u)      { fault.code = BMS_FAULT_DATA_STALE;  return; }

  if (fault.ov != 0u)
  {
    fault.cell       = first_bit(fault.ov);
    fault.cell_count = count_bits(fault.ov);
    fault.code       = BMS_FAULT_CELL_OV;
    return;
  }

  if (fault.uv != 0u)
  {
    fault.cell       = first_bit(fault.uv);
    fault.cell_count = count_bits(fault.uv);
    fault.code       = BMS_FAULT_CELL_UV;
    return;
  }

  /* Usa a mascara com histerese, nao o bit-resumo do GEN_DIAG: e ela que
   * define quando a sobretemperatura realmente sai. */
  if (fault.ot != 0u)
  {
    fault.cell       = first_bit((uint16_t)fault.ot);
    fault.cell_count = count_bits((uint16_t)fault.ot);
    fault.code       = BMS_FAULT_EXT_OVERTEMP;
    return;
  }

  if ((fault.temp_open != 0u) || (fault.temp_short != 0u))
  {
    fault.cell = first_bit((uint16_t)(fault.temp_open | fault.temp_short));
    fault.code = BMS_FAULT_OPEN_LOAD;
    return;
  }
  if ((d & TLE9012_DIAG_INT_OT) != 0u)    { fault.code = BMS_FAULT_INT_OVERTEMP; return; }

  if (fault.ut != 0u)
  {
    fault.cell       = first_bit((uint16_t)fault.ut);
    fault.cell_count = count_bits((uint16_t)fault.ut);
    fault.code       = BMS_FAULT_UNDERTEMP;
    return;
  }

  if ((d & TLE9012_DIAG_OL_ERR) != 0u)      { fault.code = BMS_FAULT_OPEN_LOAD;   return; }
  if ((d & TLE9012_DIAG_ADC_ERR) != 0u)     { fault.code = BMS_FAULT_ADC;         return; }
  if ((d & TLE9012_DIAG_INT_IC_ERR) != 0u)  { fault.code = BMS_FAULT_INTERNAL_IC; return; }
  if ((d & TLE9012_DIAG_REG_CRC_ERR) != 0u) { fault.code = BMS_FAULT_REG_CRC;     return; }
  if ((d & TLE9012_DIAG_PS_ERR_SLEEP) != 0u){ fault.code = BMS_FAULT_PS_SLEEP;    return; }

  if ((d & (TLE9012_DIAG_BAL_ERR_UC | TLE9012_DIAG_BAL_ERR_OC)) != 0u)
  {
    fault.code = BMS_FAULT_BALANCING;
    return;
  }

  fault.code = BMS_FAULT_NONE;
}

/**
 * @brief Ha alguma condicao de falha ativa?
 *
 * Cobre tanto falha reportada pelo CI quanto perda de comunicacao e dado
 * obsoleto -- nao saber a tensao e, para um AMS, tao grave quanto saber que
 * ela esta errada.
 */
static bool bms_any_fault(void)
{
  if (link.ready == 0u)      { return true; }  /* cadeia caida             */
  if (meas.valid == 0u)  { return true; }  /* medicao obsoleta         */
  /* Usa a versao com histerese: e ela que define quando a falha realmente
   * sai, nao o flag cru do CI. */
  if (fault.uv != 0u)     { return true; }
  if (fault.ov != 0u)     { return true; }
  if (fault.ut != 0u)         { return true; }  /* subtemperatura, firmware */
  if (fault.ot != 0u)         { return true; }  /* sobretemperatura         */
  if (fault.temp_open != 0u)  { return true; }
  if (fault.temp_short != 0u) { return true; }

  return ((fault.gen_diag & TLE9012_DIAG_FAULT_MASK) != 0u);
}

/**
 * @brief Aplica (ou remove) uma injecao de falha.
 *
 * Cada modo cria uma condicao fisica ou de configuracao que o TLE9012 detecta
 * pelo caminho normal. Nada aqui escreve em flag de diagnostico.
 */
static void bms_apply_simulation(bms_sim_fault_t which)
{
  tle9012_status_t st = TLE9012_OK;

  switch (which)
  {
    case BMS_SIM_UNDERVOLTAGE:
      /* Limiar de UV acima da tensao medida: as celulas passam a estar
       * legitimamente abaixo do limite configurado. */
      fault.uv_thr_mv = (uint16_t)(meas.max_mv + BMS_SIM_MARGIN_MV);
      fault.ov_thr_mv = BMS_OV_THR_MV;

      st = tle9012_set_thresholds(BMS_NODE_ID, fault.uv_thr_mv, fault.ov_thr_mv,
                                  BMS_OL_MIN_MV, BMS_OL_MAX_MV);
      break;

    case BMS_SIM_OVERVOLTAGE:
      /* Limiar de OV abaixo da tensao medida. Guarda contra underflow. */
      if (meas.min_mv > (BMS_SIM_MARGIN_MV + BMS_UV_THR_MV))
      {
        fault.uv_thr_mv = BMS_UV_THR_MV;
        fault.ov_thr_mv = (uint16_t)(meas.min_mv - BMS_SIM_MARGIN_MV);

        st = tle9012_set_thresholds(BMS_NODE_ID, fault.uv_thr_mv, fault.ov_thr_mv,
                                    BMS_OL_MIN_MV, BMS_OL_MAX_MV);
      }
      else
      {
        st = TLE9012_ERR_PARAM;   /* tensao baixa demais para injetar OV */
      }
      break;

    case BMS_SIM_OVERTEMP:
      /* Move o limiar para logo ACIMA do RESULT medido -- o que em graus
       * significa logo ABAIXO da temperatura atual, ja que RESULT maior
       * corresponde a mais frio.
       *
       * Deslocar em vez de saturar no maximo e proposital: o limiar
       * continua sendo um numero plausivel em graus, entao o depurador
       * mostra "limiar 20 C, medindo 24 C, disparou" -- que se explica
       * sozinho. Saturado em 1023 apareceria como -30 C e nao comunicaria
       * nada a quem esta olhando.
       */
      if ((temp.valid[0] != 0u) && (temp.result[0] > 0u))
      {
        uint32_t thr = (uint32_t)temp.result[0] + BMS_SIM_OT_MARGIN;

        if (thr > TLE9012_TEMP_CONF_OT_MASK)
        {
          thr = TLE9012_TEMP_CONF_OT_MASK;
        }

        fault.ot_thr       = (uint16_t)thr;
        fault.ot_clear_thr = (uint16_t)(thr + BMS_SIM_OT_MARGIN);

        st = tle9012_config_temp(BMS_NODE_ID, BMS_NUM_TEMP_USED,
                                 BMS_TEMP_INTC, (uint16_t)thr);
      }
      else
      {
        st = TLE9012_ERR_PARAM;   /* sem leitura valida para injetar */
      }
      break;

    case BMS_SIM_OPEN_LOAD:
      /* Reabilita o diagnostico de open load. Com o ladder resistivo ele
       * dispara de verdade, porque a resistencia do divisor e muito maior
       * que a interna de uma celula -- exatamente o que o manual descreve. */
      st = tle9012_set_thresholds(BMS_NODE_ID, BMS_UV_THR_MV, BMS_OV_THR_MV,
                                  60u, 220u);
      break;

    case BMS_SIM_UNDERTEMP:
      /* Limiar de UT abaixo da resistencia medida: o NTC passa a estar
       * legitimamente "mais frio" que o limite configurado. A comparacao e
       * a mesma da protecao real -- so o limiar mudou.
       *
       * Nota honesta: diferente dos outros modos, este exercita apenas a
       * deteccao em FIRMWARE, porque o TLE9012 nao tem protecao de
       * subtemperatura. Nao ha caminho de hardware para demonstrar aqui. */
      if ((temp.valid[0] != 0u) && (temp.ohms[0] > 100u))
      {
        fault.ut_enabled = 1u;
        fault.ut_thr_ohm = temp.ohms[0] / 2u;
      }
      else
      {
        /* Sem termistor conectado nao ha o que injetar: o canal esta
         * invalido e a protecao nao tem entrada. */
        st = TLE9012_ERR_PARAM;
      }
      break;

    case BMS_SIM_COMM_LOSS:
      /* Manda o CI dormir. A comunicacao cai de fato, o NODE_ID reseta, e a
       * recuperacao automatica e exercitada junto. */
      st = tle9012_write_reg(BMS_NODE_ID, TLE9012_REG_OP_MODE, 0xC401u);
      break;

    case BMS_SIM_NONE:
    default:
      /* Restaura a configuracao normal. */
      fault.uv_thr_mv = BMS_UV_THR_MV;
      fault.ov_thr_mv = BMS_OV_THR_MV;

      st = tle9012_set_thresholds(BMS_NODE_ID, fault.uv_thr_mv, fault.ov_thr_mv,
                                  BMS_OL_MIN_MV, BMS_OL_MAX_MV);
      if (st == TLE9012_OK)
      {
        st = tle9012_config_temp(BMS_NODE_ID, BMS_NUM_TEMP_USED,
                                 BMS_TEMP_INTC, BMS_TEMP_OT_THR);
      }
      if (st == TLE9012_OK)
      {
        st = tle9012_clear_faults(BMS_NODE_ID);
        fault.latched = 0u;
      }
      fault.ut_thr_ohm   = BMS_UT_THR_OHM;   /* desfaz injecao de subtemperatura */
      fault.ut_enabled   = 0u;               /* volta desligada: sem termistor   */
      fault.ot_thr       = BMS_TEMP_OT_THR;  /* desfaz injecao de sobretemp.     */
      fault.ot_clear_thr = BMS_TEMP_OT_CLEAR;
      fault.ut     = 0u;
      fault.ot     = 0u;
      fault.temp_open  = 0u;
      fault.temp_short = 0u;
      fault.uv = 0u;               /* sai do estado latcheado          */
      fault.ov = 0u;
      break;
  }

  sim.apply_status = (uint8_t)st;
  sim.active       = which;
}

/**
 * @brief Configura o GPIO do shutdown, ja no estado seguro.
 *
 * A ordem importa: escreve nivel BAIXO antes de configurar como saida, para
 * que o pino nunca passe por um instante em alto durante a inicializacao.
 */
static void bms_shutdown_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  BMS_SHUTDOWN_CLK_ENABLE();

  HAL_GPIO_WritePin(BMS_SHUTDOWN_PORT, BMS_SHUTDOWN_PIN, GPIO_PIN_RESET);

  gpio.Pin   = BMS_SHUTDOWN_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;   /* pull-down e EXTERNO, no gate do MOSFET */
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_SHUTDOWN_PORT, &gpio);

  shutdown.closed       = 0u;
  shutdown.latched_open = 0u;
}

/**
 * @brief Atualiza o shutdown circuit conforme o estado de falha.
 *
 * Fecha somente quando TUDO esta em ordem: cadeia de pe, dado valido, nenhuma
 * falha e nenhum travamento pendente. Na duvida, abre.
 */
static void bms_shutdown_update(bool any_fault)
{
  if (any_fault)
  {
    if (shutdown.latched_open == 0u)
    {
      shutdown.latched_open = 1u;
      shutdown.cause        = fault.code;   /* preserva o que abriu */
      shutdown.open_count++;
    }
  }
  else if (shutdown.reset_request != 0u)
  {
    /* Destrava so na ausencia de falha: pedido de reset com falha ativa e
     * ignorado, nunca enfileirado. */
    shutdown.latched_open  = 0u;
    shutdown.reset_request = 0u;
  }
  else
  {
    /* sem falha e sem pedido: mantem o estado atual */
  }

  const bool allow = (!any_fault) && (shutdown.latched_open == 0u);

#if BMS_SHUTDOWN_TOGGLE
  /* Onda quadrada: so mantem o RC externo carregado enquanto o loop roda. */
  if (allow)
  {
    HAL_GPIO_TogglePin(BMS_SHUTDOWN_PORT, BMS_SHUTDOWN_PIN);
  }
  else
  {
    HAL_GPIO_WritePin(BMS_SHUTDOWN_PORT, BMS_SHUTDOWN_PIN, GPIO_PIN_RESET);
  }
#else
  HAL_GPIO_WritePin(BMS_SHUTDOWN_PORT, BMS_SHUTDOWN_PIN,
                    allow ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

  shutdown.closed = allow ? 1u : 0u;
}

/** Botao azul da Nucleo: avanca para o proximo modo de simulacao. */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    sim.request = (bms_sim_fault_t)(((uint8_t)sim.request + 1u)
                                    % (uint8_t)BMS_SIM_COUNT);
  }
}

/**
 * @brief Le o estado de falha do dispositivo.
 *
 * Nao limpa nada: limpar falha e decisao de quem trata, nao de quem le. O
 * acumulador fault.latched existe porque uma falha momentanea pode sumir do
 * GEN_DIAG antes de alguem olhar -- perder isso num BMS e inaceitavel.
 */
static void bms_fault_cycle(void)
{
  tle9012_faults_t f;

  const tle9012_status_t st = tle9012_read_faults(BMS_NODE_ID, &f);
  fault.read_status = (uint8_t)st;

  if (st != TLE9012_OK)
  {
    return;
  }

  fault.gen_diag = f.gen_diag;
  fault.uv_raw   = f.cell_uv;
  fault.ov_raw   = f.cell_ov;

  /* Seta imediatamente: deteccao nunca e atrasada pela histerese. */
  fault.uv |= f.cell_uv;
  fault.ov |= f.cell_ov;

  /* Limpa so com margem, e so com medicao valida. Dado obsoleto nunca pode
   * justificar sair de um estado de falha. */
  if (meas.valid != 0u)
  {
    uint16_t clear_uv = 0u;
    uint16_t clear_ov = 0u;

    for (uint8_t i = 0u; i < BMS_NUM_CELLS; i++)
    {
      const uint16_t bit = (uint16_t)(1u << i);
      const uint16_t mv  = meas.cell_mv[i];

      if (((fault.uv & bit) != 0u) &&
          (mv > (uint16_t)(fault.uv_thr_mv + BMS_FAULT_HYST_MV)))
      {
        clear_uv |= bit;
      }

      if (((fault.ov & bit) != 0u) &&
          (fault.ov_thr_mv > BMS_FAULT_HYST_MV) &&
          (mv < (uint16_t)(fault.ov_thr_mv - BMS_FAULT_HYST_MV)))
      {
        clear_ov |= bit;
      }
    }

    if ((clear_uv != 0u) || (clear_ov != 0u))
    {
      /* Limpa tambem no CI: o flag e latching, entao sem isto ele
       * reapareceria na proxima leitura e a histerese nao teria efeito. */
      if (tle9012_clear_cell_flags(BMS_NODE_ID, clear_uv, clear_ov)
          == TLE9012_OK)
      {
        fault.uv &= (uint16_t)~clear_uv;
        fault.ov &= (uint16_t)~clear_ov;
      }
    }

    /* CELL_UV e CELL_OV do GEN_DIAG sao bits-RESUMO, em registrador
     * separado dos flags por celula. Limpar os detalhados nao limpa o
     * resumo -- sem isto o GEN_DIAG fica preso na falha antiga.
     *
     * Sao do tipo rocwl: escrever '0' limpa o bit e reseta o registrador
     * detalhado associado, que a esta altura ja esta vazio. */
    uint16_t diag_clear = 0u;

    if ((fault.uv == 0u) && ((fault.gen_diag & TLE9012_DIAG_CELL_UV) != 0u))
    {
      diag_clear |= TLE9012_DIAG_CELL_UV;
    }

    if ((fault.ov == 0u) && ((fault.gen_diag & TLE9012_DIAG_CELL_OV) != 0u))
    {
      diag_clear |= TLE9012_DIAG_CELL_OV;
    }

    if (diag_clear != 0u)
    {
      /* '1' preserva os demais bits; '0' so nas posicoes a limpar. */
      (void)tle9012_write_reg(BMS_NODE_ID, TLE9012_REG_GEN_DIAG,
                              (uint16_t)~diag_clear);
    }
  }

  /* Bits 0 a 4 do GEN_DIAG sao status de operacao, nao erro -- nao entram
   * no acumulador para nao poluir com "round robin ativo" e afins. */
  fault.latched |= (uint16_t)(f.gen_diag & TLE9012_DIAG_FAULT_MASK);

  if (((f.gen_diag & TLE9012_DIAG_FAULT_MASK) != 0u) ||
      (f.cell_uv != 0u) || (f.cell_ov != 0u))
  {
    fault.count++;
  }
}

/**
 * @brief Le os NTCs externos e converte para resistencia.
 *
 * Nao afeta link.ready: falha de temperatura nao derruba a cadeia, so
 * invalida os canais. A tensao continua sendo o dado critico.
 */
static void bms_temp_cycle(void)
{
  tle9012_temp_raw_t raw[BMS_NUM_TEMP];

  const tle9012_status_t st = tle9012_read_temp_raw(BMS_NODE_ID, raw,
                                                    BMS_NUM_TEMP_USED);
  temp.status = (uint8_t)st;

  if (st != TLE9012_OK)
  {
    temp.fail_count++;

    for (uint8_t z = 0u; z < BMS_NUM_TEMP_USED; z++)
    {
      temp.valid[z] = 0u;
    }

    return;
  }

  /* Contexto de diagnostico: confirma que a configuracao pegou e mostra os
   * flags detalhados de temperatura. Sem isto, "tudo zero" nao distingue
   * "nao configurado" de "configurado mas sem medir". */
  {
    uint16_t v = 0u;

    if (tle9012_read_reg(BMS_NODE_ID, TLE9012_REG_TEMP_CONF, &v) == TLE9012_OK)
    {
      temp.conf_readback = v;
    }

    if (tle9012_read_reg(BMS_NODE_ID, TLE9012_REG_EXT_TEMP_DIAG, &v)
        == TLE9012_OK)
    {
      temp.ext_diag = v;

      /* Layout do EXT_TEMP_DIAG: tres bits por canal, na ordem
       * OPEN, SHORT, OT -- canal z ocupa os bits 3z, 3z+1 e 3z+2. */
      uint8_t ot_now = 0u;
      uint8_t op_now = 0u;
      uint8_t sh_now = 0u;

      for (uint8_t z = 0u; z < BMS_NUM_TEMP_USED; z++)
      {
        const uint8_t base = (uint8_t)(3u * z);

        if ((v & (uint16_t)(1u << base))        != 0u) { op_now |= (uint8_t)(1u << z); }
        if ((v & (uint16_t)(1u << (base + 1u))) != 0u) { sh_now |= (uint8_t)(1u << z); }
        if ((v & (uint16_t)(1u << (base + 2u))) != 0u) { ot_now |= (uint8_t)(1u << z); }
      }

      /* Aberto e curto nao tem histerese: sao falha de fiacao, nao grandeza
       * analogica oscilando em torno de um limiar. */
      fault.temp_open  = op_now;
      fault.temp_short = sh_now;

      /* Sobretemperatura seta na hora e so sai com margem. */
      fault.ot |= ot_now;
    }
  }

  for (uint8_t z = 0u; z < BMS_NUM_TEMP_USED; z++)
  {
    /* Captura o cru ANTES de qualquer guarda: quando nada funciona, e o
     * conteudo do registrador que diz por que. Deixar isto depois do teste
     * de VALID esconde justamente o caso que precisa ser diagnosticado. */
    temp.reg[z]    = raw[z].reg;
    temp.result[z] = raw[z].result;
    temp.intc[z]   = raw[z].intc;

    /* Histerese da sobretemperatura. RESULT maior significa mais frio, entao
     * a falha so sai quando ele SOBE acima do limiar de retorno.
     *
     * Feito aqui, e nao ao ler o EXT_TEMP_DIAG, porque depende do RESULT
     * deste canal -- e o EXT_TEMP_DIAG so diz que passou, nao quanto. */
    if (((fault.ot & (uint8_t)(1u << z)) != 0u) &&
        (raw[z].valid) &&
        (raw[z].result > fault.ot_clear_thr))
    {
      fault.ot &= (uint8_t)~(1u << z);

      /* Limpa tambem no CI: o flag e latching (rocw), e sem limpar ele
       * reaparece na proxima leitura e a histerese nao teria efeito.
       * '0' na posicao limpa, '1' preserva as demais. */
      const uint16_t ot_bit = (uint16_t)(1u << ((3u * z) + 2u));
      (void)tle9012_write_reg(BMS_NODE_ID, TLE9012_REG_EXT_TEMP_DIAG,
                              (uint16_t)~ot_bit);
    }

    /* O bit VALID e limpo ao ler: se estiver zero, o round robin ainda nao
     * produziu resultado novo para este canal desde a leitura anterior. */
    if (!raw[z].valid)
    {
      temp.valid[z] = 0u;
      continue;
    }

    const uint32_t measured = tle9012_temp_raw_to_ohms(&raw[z],
                                                       BMS_TEMP_R_SERIES);

    /* O que o ADC ve: NTC externo em paralelo com o componente da placa. */
    temp.ohms_raw[z] = measured;

    /* Guarda de confiabilidade: perto de R_PARALLEL o denominador da
     * compensacao tende a zero e o resultado deixa de ter significado. */
    const uint32_t trust_limit =
        ((uint32_t)BMS_TEMP_R_PARALLEL * BMS_TEMP_TRUST_PCT) / 100uL;

    if (measured >= trust_limit)
    {
      /* Tipicamente: nenhum NTC externo conectado neste canal. */
      temp.valid[z] = 0u;
      fault.ut &= (uint8_t)~(1u << z);
      continue;
    }

    const uint32_t ohms = tle9012_compensate_parallel(measured,
                                                      BMS_TEMP_R_PARALLEL);

    if (ohms == 0u)
    {
      temp.valid[z] = 0u;
      fault.ut &= (uint8_t)~(1u << z);
      continue;
    }

    temp.ohms[z]  = ohms;
    temp.valid[z] = 1u;

    /* Subtemperatura em firmware: NTC tem coeficiente negativo, entao mais
     * frio = maior resistencia. */
    if ((fault.ut_enabled != 0u) && (ohms > fault.ut_thr_ohm))
    {
      fault.ut |= (uint8_t)(1u << z);
    }
    else
    {
      fault.ut &= (uint8_t)~(1u << z);
    }

    /* Equacao de Beta, so para leitura humana. Protecao deve comparar em
     * ohms, sem passar por logaritmo. */
    if (ohms > 0u)
    {
      const float t_kelvin =
          1.0f / ((1.0f / 298.15f)
                  + (logf((float)ohms / BMS_NTC_R25_OHM) / BMS_NTC_BETA));

      temp.celsius[z] = t_kelvin - 273.15f;
    }
  }

  /* Espelha os limiares em graus, usando a fonte de corrente realmente em
   * uso. Fica ao lado da medicao no depurador, o que e o que torna a
   * demonstracao legivel: limiar e leitura na mesma unidade. */
  {
    const uint8_t intc = temp.intc[0];

    fault.ot_thr_c   = result_to_celsius(fault.ot_thr, intc);
    fault.ot_clear_c = result_to_celsius(fault.ot_clear_thr, intc);
    fault.ut_thr_c   = ohms_to_celsius(fault.ut_thr_ohm);
  }

  temp.count++;
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
  /* Shutdown no estado seguro ANTES de qualquer outra coisa: o circuito so
   * fecha depois que a cadeia estiver de pe e sem falha. */
  bms_shutdown_init();

  tle9012_port_init(&huart1);
  tle9012_bind(tle9012_port_transport());

  /* O TLE9015 liga acordado (nSleep tem pull-up interno e so manda dormir na
   * borda de descida), portanto aqui apenas fixamos o nivel. Ja o TLE9012
   * precisa mesmo ser acordado, e isso se faz com um comando de leitura.
   * Se o nSleep nao estiver fiado, a primeira chamada pode ser removida. */
  tle9012_port_inhibit_sleep();
  tle9012_wakeup();

  link.ready = bms_chain_init() ? 1u : 0u;
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
    /* Idade do snapshot, atualizada sempre -- inclusive enquanto a cadeia
     * esta caida, que e justamente quando ela importa. */
    meas.age_ms = HAL_GetTick() - s_last_good_tick;

    if (link.ready == 0u)
    {
      /* Cadeia caida ou ainda nao inicializada: tenta de novo em vez de
       * travar. Inspecionar tp_dbg_avail_on_timeout, depois tle_dbg_rx. */
      meas.valid = 0u;
      BSP_LED_On(LED_GREEN);        /* cadeia caida tambem e falha */
      bms_shutdown_update(true);    /* e tem que abrir o shutdown  */

      HAL_Delay(500u);
      tle9012_wakeup();

      if (bms_chain_init())
      {
        link.ready   = 1u;
        s_consec_fail = 0u;
      }

      continue;
    }

    bms_measure_cycle();

    /* Falhas a cada ciclo: e o dado de seguranca, nao pode ficar atrasado. */
    bms_fault_cycle();

    /* Temperatura em cadencia mais lenta que a tensao. */
    if ((meas.count % BMS_TEMP_EVERY_N) == 0u)
    {
      bms_temp_cycle();
    }

    /* Limpeza de falhas latcheadas, pedida pelo depurador. */
    if (fault.clear_request != 0u)
    {
      fault.clear_request = 0u;

      if (tle9012_clear_faults(BMS_NODE_ID) == TLE9012_OK)
      {
        fault.latched = 0u;
        fault.uv      = 0u;
        fault.ov      = 0u;
        fault.ut      = 0u;
        fault.ot      = 0u;
        fault.temp_open  = 0u;
        fault.temp_short = 0u;
      }
    }

    /* Injecao de falha pedida pelo botao ou pelo depurador. */
    if (sim.request != sim.active)
    {
      bms_apply_simulation(sim.request);
    }

    bms_resolve_fault_code();

    const bool faulted = bms_any_fault();

    /* Shutdown antes do LED: se algo aqui bloquear, o circuito ja abriu. */
    bms_shutdown_update(faulted);

    /* LD2 aceso FIXO = falha. Piscando = vivo e saudavel.
     *
     * Piscar como sinal de saude e proposital: LED apagado passa a significar
     * firmware travado, e nao "tudo bem". Um AMS nao pode ter estado de falha
     * indistinguivel de estado morto. */
    if (faulted)
    {
      BSP_LED_On(LED_GREEN);
    }
    else
    {
      BSP_LED_Toggle(LED_GREEN);
    }

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
