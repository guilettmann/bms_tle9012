# BMS TLE9012

Battery Management System baseado em **TLE9012DQU** (sensing e balanceamento) e
**TLE9015DQU** (transceiver iso UART), com **STM32G491RET6** como controlador.

O objetivo de longo prazo é um firmware **parametrizável**: mudanças de configuração
(número de células, thresholds, IDs CAN) feitas alterando uma tabela de parâmetros em
flash, sem recompilar, e atualização de firmware por **bootloader CAN**.

---

## Estado atual

| Fase | Escopo | Endereço do app | Status |
|:---:|---|---|---|
| 1 | Driver TLE9012 — ler as 12 tensões | `0x0800 0000` | 🔨 em bring-up |
| 2 | Proteções, temperatura, balanceamento | `0x0800 0000` | ⬜ |
| 3 | Tabela de parâmetros em flash + CAN | `0x0800 0000` | ⬜ |
| 4 | Bootloader CAN | `0x0800 4000` | ⬜ |

Até a fase 3, gravação e depuração são feitas normalmente pelo ST-LINK da Nucleo.
O bootloader é a última etapa, não a primeira.

---

## Hardware

| Item | Componente |
|---|---|
| MCU | NUCLEO-G491RE — Cortex-M4F @170 MHz, 512 KB flash, 3× FDCAN, ST-LINK integrado |
| Transceiver | AURIX TLE9015 Adapter Board V2.0 |
| Sensing | BMS Balancing and Sensing Board V6 (TLE9012DQU, 12 canais) |

A adapter board traz `UART_HS`, `UART_LS`, `nSLEEP`, `ERRQ`, `3.3V` e `GND` em test
points e no header COM6 — apesar do nome, **não** exige um AURIX.

### Ligação

**Nucleo → transceiver (TLE9015):**

| Adapter board | NUCLEO-G491RE | Função |
|---|---|---|
| `TXD_LS` | `PC4` (USART1_TX) | Lado do host, antes do `R_UART` |
| `UART_LS` | `PC5` (USART1_RX) | Nó compartilhado, ligado ao pino do chip |
| `VS` (test point `VS1`) | `5V` | **Alimentação — faixa 4,75 V a 45 V** |
| `VIO` | `3V3` | Define o nível lógico (faixa 3 V a 5,5 V) |
| `GND` | `GND` | Referência comum |
| `nSleep` | `PB0` | Opcional — ver nota abaixo |

> **`VS` não funciona com 3,3 V.** O mínimo funcional é 4,75 V (datasheet TLE9015DQU,
> Tabela 2). Use o pino `5V` da Nucleo — o consumo é ~5 mA em idle e ~15 mA durante
> comunicação. Alimentar só com 3V3 deixa o chip morto, com sintoma idêntico ao de
> fiação errada.

> **`nSleep` manda dormir, não acorda.** É ativo em baixo, disparado por borda de
> descida, e tem pull-up interno — o chip liga acordado. O fio é opcional no bring-up;
> serve para fixar o nível e permitir comandar sleep depois.

> **`TXD_LS` e `UART_LS` são os dois extremos do `R_UART`** (1,5 kΩ, já populado na
> placa de avaliação — medido e confirmado). Se na sua placa a medição entre eles der
> 0 Ω, é nó único: ligue o RX direto e o TX através de um resistor de 1,5 kΩ.

**Transceiver → sensing (TLE9012):**

| TLE9015 | TLE9012 |
|---|---|
| conector `LS`, pino `P` | `CON6` → `H P` |
| conector `LS`, pino `N` | `CON6` → `H N` |

O **low side do transceiver vai no high side do sensing** — nunca LS↔LS. Use o cabo
par trançado do kit. O conector `L P`/`L N` do sensing fica livre: é a última placa da
cadeia, e por isso o firmware passa `final_node = true`.

A sensing board é alimentada à parte, com **5 V a 60 V** nos terminais `VS`/`GND` da
borda esquerda. **Não ligue o GND dela ao GND da Nucleo** — os dois domínios são
galvanicamente isolados de propósito, e o link iso UART é a barreira.

---

## Protocolo iso UART — o essencial

| Parâmetro | Valor |
|---|---|
| Baudrate | **2 Mbit/s** (oficial) — **mínimo 1 Mbit/s** |
| Formato | 8N1, **MSB first** |
| Topologia | Single-wire half-duplex, daisy-chain |

Três armadilhas que custam tempo:

1. **Os 115200 baud que aparecem no manual da Infineon são do terminal de debug
   PC↔AURIX, não do link iso UART.** Abaixo de 1 Mbit/s a comunicação não estabelece.
2. **O protocolo é MSB-first.** No STM32 isso é hardware (`CR2.MSBFIRST`), já
   habilitado no `.ioc`. Não precisa inverter bits por software.
3. **Os bytes de um frame não podem ter atraso entre si** — o transceiver dá timeout.
   Por isso TX e RX usam DMA, não polling.

### Frames

```
Escrita (6 bytes):  1E | 80|ID | ADDR | DATA_H | DATA_L | CRC8
Leitura (4 bytes):  1E | 00|ID | ADDR | CRC8
```

O byte SYNC (`0x1E`) **entra** no cálculo do CRC — validado contra os 10 vetores de
exemplo das Tabelas 7–20 do user manual.

Como a linha é compartilhada, o host recebe de volta o próprio eco:

| Operação | Envia | Recebe | Composição |
|---|---|---|---|
| Read | 4 B | 9 B | 4 de eco + 5 de resposta |
| Write | 6 B | 7 B | 6 de eco + 1 reply frame |

Dois CRCs distintos: **CRC8** (SAE J1850, poly `0x1D`, init `0xFF`, xor `0xFF`) nos
frames de dados, e **CRC3** (poly `0xB0`, válido se o resto der zero) no reply frame.

---

## Estrutura

```
bms_tle9012/
├── docs/
│   ├── relatorio_estrutura_software_bms.html   fonte do relatório
│   └── relatorio_estrutura_software_bms.pdf    documento de projeto
└── firmware/bms_tle9012/
    ├── bms_tle9012.ioc                         configuração do CubeMX
    └── Core/
        ├── Inc/  crc_j1850.h  tle9012.h  tle9012_regs.h  tle9012_port.h
        └── Src/  crc_j1850.c  tle9012.c  tle9012_port.c  main.c
```

**Regra de arquitetura:** `tle9012.c` não contém nenhuma chamada de HAL. Toda a
dependência de hardware entra pela struct `tle9012_transport_t` (`send` / `recv` /
`flush` / `delay_us`). Portar para AURIX ou para um RTOS é reimplementar
`tle9012_port.c`, sem tocar no driver.

---

## Compilando

Abrir `firmware/bms_tle9012` no **STM32CubeIDE 1.18+** e compilar. Nenhuma alteração
de linker é necessária na fase 1 — o app usa o endereço default `0x08000000`.

Ao clonar, dê **F5** no projeto para o CubeIDE indexar os arquivos.

---

## Depuração — Live Expressions

O firmware não usa `printf`. O acompanhamento é por **Live Expressions** do CubeIDE,
que lê memória pelo SWD com o alvo rodando. Todas as variáveis são globais e
`volatile` para permitir isso.

**Operação normal:**

| Variável | Conteúdo |
|---|---|
| `cell_mv[12]` | Tensão de cada célula, em mV |
| `cell_min_mv` / `cell_max_mv` / `cell_delta_mv` | Extremos e desbalanço |
| `meas_count` / `meas_fail_count` | Ciclos bem-sucedidos e falhos |

**Quando o link não sobe**, a variável que resolve é `tp_dbg_avail_on_timeout`:

| Valor | Diagnóstico |
|---|---|
| `0` | RX não vê nem o próprio eco → fiação, resistor série ausente ou pino errado |
| parcial (4 ou 6) | Eco chega mas o escravo não responde → alimentação, `nSLEEP` ou baudrate |
| completo | Chegou tudo → problema de parsing, não de hardware |

Complementam o diagnóstico `tle_dbg_tx`, `tle_dbg_rx`, `tle_dbg_status` (bytes crus da
última transação) e `tp_dbg_error_callbacks` (erros de linha).

---

## Pendências conhecidas

- **Tempo de conversão do PCVM** está como delay fixo de 5 ms. O bit `PCVM_START` se
  limpa sozinho ao terminar, mas a posição dele (bit 15) foi *deduzida* comparando
  `0xE021` do PCVM com `0x0E21` do BVM — confirmar na seção 4.23 do manual antes de
  trocar por polling.
- **Multiread não implementado.** É o mecanismo mais eficiente para ler as 12 tensões
  num ciclo só; hoje são 12 transações. Os endereços dos registradores `MULTI_READ` /
  `MULTI_READ_CFG` precisam ser confirmados na seção 4.2 — não estão em
  `tle9012_regs.h` justamente para não propagar palpite.
- **Watchdog do TLE9012** ainda não é alimentado. Ele reseta o `NODE_ID` se não for
  servido dentro do intervalo de `WDOG_CNT.WD_CNT`.
- **Sem transceiver CAN** — a Nucleo não tem um on-board; será necessário externo
  (TJA1051 ou SN65HVD230) nas fases 3 e 4.

---

## Referências

- Infineon, *TLE9012DQU / TLE9015DQU User Manual*, Rev. 1.0 (Z8F80064984) —
  [infineon.com](https://www.infineon.com/battery-management-systems).
  **Não versionado neste repositório** (copyright de terceiros, ~17 MB); baixar e
  colocar em `docs/`.
- STMicroelectronics, *RM0440* — Reference Manual da série STM32G4.
- [MaxMax-embedded/TLE9012_Arduino_Lib](https://github.com/MaxMax-embedded/TLE9012_Arduino_Lib) —
  implementação de referência, licença MIT. Fonte do mapa de registradores e da
  confirmação de CRC, MSB-first e contagem de bytes.

---

## Aviso

Este é um projeto **experimental**, em desenvolvimento e ainda não validado em bancada.
Não é um produto certificado e não passou por nenhum processo de segurança funcional.

Baterias de íon-lítio apresentam risco real de incêndio quando operadas fora dos limites
de tensão, corrente ou temperatura. Um BMS com defeito pode falhar em detectar essas
condições. Quem usar este código assume integralmente o risco — valide o comportamento
por conta própria antes de conectar a qualquer pack real.

## Licença

[MIT](LICENSE) — Guilherme Lettmann, equipe AMPERA.

Partes do mapa de registradores e a validação do protocolo derivam de
[TLE9012_Arduino_Lib](https://github.com/MaxMax-embedded/TLE9012_Arduino_Lib),
também sob licença MIT.
