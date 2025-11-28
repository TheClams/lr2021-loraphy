#![no_std]

use lr2021::{BusyAsync, BusyPin, Lr2021, lora::{ExitMode, HeaderType, Ldro, LoraBw, LoraCr, LoraModulationParams, LoraPacketParams, Sf}, radio::{PacketType, RampTime}, status::Intr, system::{ChipMode, DioNum}};
use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal_async::{digital::Wait, spi::SpiBus};
use embassy_time::Duration;

pub use lora_phy::{mod_traits::*, mod_params::*, RxMode};

/// Wrapper around the Lr2021 Driver to implement the LoRaPhy traits
/// This allows integration in lora-rs which provide a LoRaWAN stack implementation
pub struct Lr2021LoraPhy<O, SPI, IRQ, M:BusyPin> {
    pub driver: Lr2021<O,SPI,M>,
    irq: IRQ,
    dio_irq: DioNum
}

// Create driver with busy pin implementing wait
impl<I,O,SPI> Lr2021LoraPhy<O,SPI, I, BusyAsync<I>> where
    I: InputPin + Wait, O: OutputPin, SPI: SpiBus<u8>
{
    /// Create a LR2021 Device with async busy pin
    pub fn new(nreset: O, busy: I, spi: SPI, nss: O, irq: I, dio_irq: DioNum) -> Self {
        Self {
            driver: Lr2021::new(nreset, busy, spi, nss),
            irq, dio_irq
        }
    }
}

impl<O, SPI, IRQ, M:BusyPin> RadioKind for Lr2021LoraPhy<O,SPI,IRQ,M>
    where O: OutputPin, SPI: SpiBus<u8>, IRQ: InputPin + Wait, M:BusyPin
{

    // LoRa Init: Run Calibration, SetPacketType and Syncword
    async fn init_lora(&mut self, sync_word: u8) -> Result<(), RadioError> {
        self.driver.calib_fe(&[]).await.map_err(|_| RadioError::OpError(0))?;
        self.driver.set_packet_type(PacketType::Lora).await.map_err(|_| RadioError::OpError(1))?;
        self.driver.set_lora_syncword(sync_word).await.map_err(|_| RadioError::OpError(2))
    }

    fn create_modulation_params(
        &self,
        spreading_factor: SpreadingFactor,
        bandwidth: Bandwidth,
        coding_rate: lora_phy::mod_params::CodingRate,
        frequency_in_hz: u32,
    ) -> Result<ModulationParams, RadioError> {
        let ldro_en = match bandwidth {
            Bandwidth::_125KHz => spreading_factor == SpreadingFactor::_11 || spreading_factor == SpreadingFactor::_12,
            Bandwidth::_250KHz => spreading_factor == SpreadingFactor::_12,
            _ => false
        };
        let low_data_rate_optimize = ldro_en as u8;
        Ok(ModulationParams {
            spreading_factor,
            bandwidth,
            coding_rate,
            low_data_rate_optimize,
            frequency_in_hz,
        })
    }

    fn create_packet_params(
        &self,
        mut preamble_length: u16,
        implicit_header: bool,
        payload_length: u8,
        crc_on: bool,
        iq_inverted: bool,
        modulation_params: &ModulationParams,
    ) -> Result<PacketParams, RadioError> {
        if ((modulation_params.spreading_factor == SpreadingFactor::_5)
            || (modulation_params.spreading_factor == SpreadingFactor::_6))
            && (preamble_length < 12)
        {
            preamble_length = 12;
        }

        Ok(PacketParams {
            preamble_length,
            implicit_header,
            payload_length,
            crc_on,
            iq_inverted,
        })
    }

    async fn reset(&mut self, _delay: &mut impl lora_phy::DelayNs) -> Result<(), RadioError> {
        self.driver.reset().await.map_err(|_| RadioError::Reset)
    }

    async fn ensure_ready(&mut self, mode: RadioMode) -> Result<(), RadioError> {
        match mode {
            RadioMode::Sleep => {
                self.driver.wake_up().await.map_err(|_| RadioError::DIO1)
            }
            _ => self.driver.wait_ready(Duration::from_nanos(0)).await.map_err(|_| RadioError::DIO1)
        }
    }

    async fn set_standby(&mut self) -> Result<(), RadioError> {
        self.driver.set_chip_mode(ChipMode::StandbyXosc)
            .await
            .map_err(|_| RadioError::SPI)
    }

    async fn set_sleep(&mut self, warm_start_if_possible: bool, _delay: &mut impl lora_phy::DelayNs) -> Result<(), RadioError> {
        let chip_mode = if warm_start_if_possible {ChipMode::DeepRetention} else {ChipMode::DeepSleep};
        self.driver.set_chip_mode(chip_mode)
            .await
            .map_err(|_| RadioError::SPI)
    }

    // Tx/Rx buffer are implemented as a FIFO -> nothing to do
    async fn set_tx_rx_buffer_base_address( &mut self, _tx_base_addr: usize, _rx_base_addr: usize,) -> Result<(), RadioError> {
        Ok(())
    }

    async fn set_tx_power_and_ramp_time(
        &mut self,
        output_power: i32,
        _mdltn_params: Option<&ModulationParams>,
        is_tx_prep: bool,
    ) -> Result<(), RadioError> {
        let ramp = if is_tx_prep {RampTime::Ramp32u} else {RampTime::Ramp128u};
        let pwr = output_power.clamp(-9, 22) as i8;
        self.driver.set_tx_params(pwr, ramp).await.map_err(|_| RadioError::InvalidConfiguration)
    }

    async fn set_modulation_params(&mut self, mdltn_params: &ModulationParams) -> Result<(), RadioError> {
        let sf = match mdltn_params.spreading_factor {
            SpreadingFactor::_5  => Sf::Sf5,
            SpreadingFactor::_6  => Sf::Sf6,
            SpreadingFactor::_7  => Sf::Sf7,
            SpreadingFactor::_8  => Sf::Sf8,
            SpreadingFactor::_9  => Sf::Sf9,
            SpreadingFactor::_10 => Sf::Sf10,
            SpreadingFactor::_11 => Sf::Sf11,
            SpreadingFactor::_12 => Sf::Sf12,
        };
        let bw = match mdltn_params.bandwidth {
            Bandwidth::_7KHz => LoraBw::Bw7,
            Bandwidth::_10KHz => LoraBw::Bw10,
            Bandwidth::_15KHz => LoraBw::Bw15,
            Bandwidth::_20KHz => LoraBw::Bw20,
            Bandwidth::_31KHz => LoraBw::Bw31,
            Bandwidth::_41KHz => LoraBw::Bw41,
            Bandwidth::_62KHz => LoraBw::Bw62,
            Bandwidth::_125KHz => LoraBw::Bw125,
            Bandwidth::_250KHz => LoraBw::Bw250,
            Bandwidth::_500KHz => LoraBw::Bw500,
        };
        let cr = match mdltn_params.coding_rate {
            lora_phy::mod_params::CodingRate::_4_5 => LoraCr::Cr1Ham45Si,
            lora_phy::mod_params::CodingRate::_4_6 => LoraCr::Cr2Ham23Si,
            lora_phy::mod_params::CodingRate::_4_7 => LoraCr::Cr3Ham47Si,
            lora_phy::mod_params::CodingRate::_4_8 => LoraCr::Cr4Ham12Si,
        };
        let ldro = if mdltn_params.low_data_rate_optimize!=0 {Ldro::On} else {Ldro::Off};
        let modulation = LoraModulationParams {sf,bw,cr,ldro};
        self.driver.set_lora_modulation(&modulation).await.map_err(|_| RadioError::InvalidConfiguration)
    }

    async fn set_packet_params(&mut self, pkt_params: &PacketParams) -> Result<(), RadioError> {
        let header_type = if pkt_params.implicit_header {HeaderType::Implicit} else {HeaderType::Explicit};
        let params = LoraPacketParams {
            pbl_len: pkt_params.preamble_length,
            payload_len: pkt_params.payload_length,
            header_type,
            crc_en: pkt_params.crc_on,
            invert_iq: pkt_params.iq_inverted
        };
        self.driver.set_lora_packet(&params).await.map_err(|_| RadioError::InvalidConfiguration)
    }

    async fn calibrate_image(&mut self, frequency_in_hz: u32) -> Result<(), RadioError> {
        // Calibration is done on a freqency multiple of 4MHz
        // Approximate by a right shift of 22 bits i.e. 4.194MHz
        let freq_4m = (frequency_in_hz >> 22) as u16;
        self.driver.calib_fe(&[freq_4m]).await.map_err(|_| RadioError::OpError(3))
    }

    async fn set_channel(&mut self, frequency_in_hz: u32) -> Result<(), RadioError> {
        self.driver.set_rf(frequency_in_hz).await.map_err(|_| RadioError::OpError(4))
    }

    async fn set_payload(&mut self, payload: &[u8]) -> Result<(), RadioError> {
        self.driver.wr_tx_fifo_from(payload).await.map_err(|_| RadioError::OpError(5))
    }

    async fn do_tx(&mut self) -> Result<(), RadioError> {
        self.driver.set_tx(0).await.map_err(|_| RadioError::OpError(6))
    }

    async fn do_rx(&mut self, rx_mode: lora_phy::RxMode) -> Result<(), RadioError> {
        if let RxMode::DutyCycle(params) = rx_mode {
            // Setting DRAM1-3 retention to 0: should only be needed if a patch RAM is set and none are required at the moment ...
            self.driver.set_rx_duty_cycle(params.rx_time, params.sleep_time, false, 0)
                .await.map_err(|_| RadioError::OpError(7))
        } else {
            let timeout = if let RxMode::Single(timeout) = rx_mode {timeout as u32} else {0xFFFFFFFF};
            self.driver.set_rx(timeout, true)
                .await.map_err(|_| RadioError::OpError(7))
        }
    }

    async fn get_rx_payload(&mut self, _params: &PacketParams, rx_buffer: &mut [u8]) -> Result<u8, RadioError> {
        let pkt_len = self.driver.get_rx_pkt_len().await.map_err(|_| RadioError::OpError(8))? as usize;
        match self.driver.rd_rx_fifo_to(rx_buffer).await {
            Ok(_) => Ok(pkt_len as u8), // Should be OK: Lora packets are < 256
            Err(_) => Err(RadioError::OpError(9)),
        }
    }

    async fn get_rx_packet_status(&mut self) -> Result<PacketStatus, RadioError> {
        let status = self.driver.get_lora_packet_status().await.map_err(|_| RadioError::OpError(10))?;
        let rssi_db = -((status.rssi_pkt()>>1) as i16);
        let snr_db = ((status.snr_pkt() + 2) >> 2 ) as i16;
        Ok(PacketStatus {
            rssi: rssi_db,
            snr: snr_db,
        })
    }

    async fn do_cad(&mut self, mdltn_params: &ModulationParams) -> Result<(), RadioError> {
        self.set_modulation_params(mdltn_params).await?;
        self.driver.set_lora_cad_params(4, false, 9, ExitMode::CadOnly, 0, None)
            .await.map_err(|_| RadioError::OpError(11))?;
        self.driver.set_lora_cad().await.map_err(|_| RadioError::OpError(12))
    }

    async fn set_tx_continuous_wave_mode(&mut self) -> Result<(), RadioError> {
        self.driver.set_tx_test(lr2021::radio::TestMode::Tone)
            .await
            .map_err(|_| RadioError::OpError(12))
    }

    async fn get_rssi(&mut self) -> Result<i16, RadioError> {
        let rssi = self.driver.get_rssi_inst().await.map_err(|_| RadioError::OpError(13))?;
        let rssi_db = -((rssi>>1) as i16);
        Ok(rssi_db)
    }

    async fn set_irq_params(&mut self, radio_mode: Option<lora_phy::mod_params::RadioMode>) -> Result<(), RadioError> {
        use lr2021::status::*;
        let intr = match radio_mode {
            Some(RadioMode::Standby)  => Intr::new(IRQ_MASK_LORA_TXRX),
            Some(RadioMode::Transmit) => Intr::new(IRQ_MASK_TX_DONE|IRQ_MASK_TIMEOUT),
            Some(RadioMode::Receive(_)) => Intr::new(IRQ_MASK_LORA_TXRX),
            Some(RadioMode::ChannelActivityDetection) => Intr::new(IRQ_MASK_CAD_DONE|IRQ_MASK_CAD_DETECTED),
            _ => Intr::new(0),
        };
        self.driver.set_dio_irq(self.dio_irq, intr).await.map_err(|_| RadioError::OpError(16))
    }

    async fn await_irq(&mut self) -> Result<(), RadioError> {
        self.irq.wait_for_rising_edge().await
            .map_err(|_| RadioError::Irq)
    }

    async fn get_irq_state(
        &mut self,
        radio_mode: RadioMode,
        cad_activity_detected: Option<&mut bool>,
    ) -> Result<Option<IrqState>, RadioError> {
        let (_,intr) = self.driver.get_status().await.map_err(|_| RadioError::OpError(15))?;
        if intr.timeout() {
            return Err(RadioError::TransmitTimeout);
        }
        // Add debug message like SX126x impl ?
        let irq_state = match radio_mode {
            RadioMode::Transmit => {
                if intr.tx_done() {Some(IrqState::Done)}
                else {None}
            },
            RadioMode::Receive(_) => {
                if intr.header_err() || intr.crc_error() {None}
                else if intr.rx_done() {Some(IrqState::Done)}
                else if intr.preamble_detected() || intr.header_valid() {Some(IrqState::PreambleReceived)}
                else {None}
            },
            RadioMode::ChannelActivityDetection => {
                if intr.cad_done() {
                    if let Some(detected) = cad_activity_detected {
                        *detected = intr.cad_detected();
                    }
                    Some(IrqState::Done)
                }
                else {None}
            },
            _ => {None},
        };
        Ok(irq_state)
    }

    async fn clear_irq_status(&mut self) -> Result<(), RadioError> {
        self.driver.clear_irqs(Intr::new(0xFFFFFFFF)).await.map_err(|_| RadioError::OpError(14))
    }

    // Process IRQ: just get and clear, no workaround to handle on LR2021
    async fn process_irq_event(
        &mut self,
        radio_mode: lora_phy::mod_params::RadioMode,
        cad_activity_detected: Option<&mut bool>,
        clear_interrupts: bool,
    ) -> Result<Option<lora_phy::mod_traits::IrqState>, RadioError> {
        let irq_state = self.get_irq_state(radio_mode,cad_activity_detected).await;
        if clear_interrupts {
            self.clear_irq_status().await?;
        }
        irq_state
    }
}