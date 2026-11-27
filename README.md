# BMS TLE9012

Battery management system for the **AMPERA** Formula SAE electric car (AMP 227).

## What it is

- **Slave board** (`bms-amp-227/`): KiCad, 6 layers, 110 × 85 mm, 28S. Four
  **TLE9012DQU** cell-supervisor ICs (7 cells each) measure cell voltages and
  temperatures (16 NTCs) and drive external passive balancing (PMOS + resistors).
  The four ICs are daisy-chained over capacitively isolated **iso UART**, with a
  transformer at the cable input. Each IC has its own floating ground. The board
  has a 4 mm isolation barrier.
- **Firmware** (`firmware/`): STM32G491 (developed on a NUCLEO-G491RE). It talks to
  the chain through a **TLE9015DQU** iso UART transceiver. The goal is a
  parameterizable firmware with a CAN bootloader.
- **Simulations** (`sim/`): LTspice models (balancing).
- **Docs** (`docs/`): project reports.

## License

MIT. See [LICENSE](LICENSE).
