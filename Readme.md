# LR2021 lora-phy Trait implementation

[![Crates.io](https://img.shields.io/crates/v/lr2021-loraphy.svg)](https://crates.io/crates/lr2021-loraphy)
[![Documentation](https://docs.rs/lr2021-loraphy/badge.svg)](https://docs.rs/lr2021-loraphy)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/TheClams/lr2021-loraphy)

An async, no_std Rust driver for the Semtech LR2021 dual-band transceiver, supporting many different radio protocols including LoRa, BLE, ZigBee, Z-Wave, and more.

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
lr2021-loraphy = "0.1.0"
```

Basic usage:

```rust
use lr2021_loraphy::*;

let mut radio = Lr2021LoraPhy::new(nreset, busy, spi, nss, irq, DioNum::Dio7);
let modulation = lr2021.create_modulation_params(
        SpreadingFactor::_5,
        Bandwidth::_500KHz,
        CodingRate::_4_5,
        901_000_000,
    ).expect("Creating Modulation Params");

let pkt_params = lr2021.create_packet_params(8, false, PLD_SIZE, true, false, &modulation)
    .expect("Creating Modulation Params");

lr2021.set_channel(modulation.frequency_in_hz).await.expect("set_channel");

lr2021.set_modulation_params(&modulation).await.expect("set_modulation_params");
lr2021.set_packet_params(&pkt_params).await.expect("set_packet_params");
lr2021.set_irq_params(Some(RadioMode::Standby)).await.expect("set_irq_params");
lr2021.do_rx(RxMode::Continuous).await.expect("SetRx");
```

## Hardware Requirements

- Semtech LR2021 transceiver module
- SPI-capable microcontroller
- 4 GPIO pins: Reset (output), Busy & IRQ (input), NSS/CS (output) (not counting SPI SCK/MISO/MOSI)
- Embassy-compatible async runtime

## Documentation & Examples

- **[API Documentation](https://docs.rs/lr2021-loraphy)** - Complete API reference
- **[Example Applications](https://github.com/TheClams/lr2021-apps)** - Real-world usage examples on Nucleo boards
