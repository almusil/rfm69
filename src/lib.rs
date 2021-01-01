//! A generic rust driver to support RFM69 family wireless chips. This crate uses
//! [`embedded_hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!
//!
//! ## Supported devices
//!
//!
//! ### RFM69W
//! - Low power variant
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69W.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20191105/RFM69%20Specification.pdf)
//!
//! ### RFM69CW
//! - Low power variant, pinout is compatible with popular RFM12B
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69C.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69CW-V1.1.pdf)
//!
//! ### RFM69HW
//! - High power variant
//! - [Product page](https://www.hoperf.com/%20modules/rf_transceiver/RFM69HW.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190306/RFM69HW-V1.3%20Datasheet.pdf)
//!
//! ### RFM69HCW
//! - High power variant, pinout is compatible with popular RFM12B
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69HCW.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69HCW-V1.1.pdf)

#![no_std]

#[cfg(test)]
#[macro_use]
extern crate std;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

use core::{cmp::min, convert::TryInto, marker::PhantomData};

use crate::registers::{
    ContinuousDagc, DataMode, DccCutoff, DioMapping, DioPin, FifoMode, InterPacketRxDelay,
    LnaConfig, LnaGain, LnaImpedance, Mode, Modulation, ModulationShaping, ModulationType,
    Pa13dBm1, Pa13dBm2, PacketConfig, PacketDc, PacketFiltering, PacketFormat, Registers, RxBw,
    RxBwFreq, RxBwFsk, SensitivityBoost,
};

pub mod registers;

const FOSC: f32 = 32_000_000.0;
const FSTEP: f32 = FOSC / 524_288.0; // FOSC/2^19

type Result<T, Ecs, Espi> = core::result::Result<T, Error<Ecs, Espi>>;

#[derive(Debug)]
pub enum Error<Ecs, Espi> {
    // Chip select pin error
    Cs(Ecs),
    // SPI bus error
    Spi(Espi),
    // Timeout exceeded
    Timeout,
    // Aes key size is too big
    AesKeySize,
    // Sync sequence is too long
    SyncSize,
}

/// Main struct to interact with RFM69 chip.
pub struct Rfm69<T, S, D> {
    spi: S,
    cs: T,
    delay: D,
    mode: Mode,
    dio: [Option<DioMapping>; 6],
    rssi: f32,
}

impl<T, S, D, Ecs, Espi> Rfm69<T, S, D>
where
    T: OutputPin<Error = Ecs>,
    S: Transfer<u8, Error = Espi>,
    S: Write<u8, Error = Espi>,
    D: DelayMs<u8>,
{
    /// Creates a new instance with everything set to default values after restart.
    pub fn new(spi: S, cs: T, delay: D) -> Self {
        Rfm69 {
            spi,
            cs,
            delay,
            mode: Mode::Standby,
            dio: [None; 6],
            rssi: 0.0,
        }
    }

    /// Reads content of all registers that are available.
    pub fn read_all_regs(&mut self) -> Result<[u8; 0x4f], Ecs, Espi> {
        let mut buffer = [0u8; 0x4f];
        self.read_many(Registers::OpMode, &mut buffer)?;
        Ok(buffer)
    }

    /// Sets the mode in corresponding register `RegOpMode (0x01)`.
    pub fn mode(&mut self, mode: Mode) -> Result<(), Ecs, Espi> {
        let val = mode as u8;
        self.update(Registers::OpMode, |r| (r & 0xe3) | val)?;
        self.mode = mode;
        self.dio()
    }

    /// Sets the modulation in corresponding register `RegDataModul (0x02)`.
    pub fn modulation(&mut self, modulation: Modulation) -> Result<(), Ecs, Espi> {
        self.write(Registers::DataModul, modulation.value())
    }

    /// Computes the bitrate, according to `Fosc / bit_rate` and stores it in
    /// `RegBitrateMsb (0x03), RegBitrateLsb (0x04)`.
    pub fn bit_rate(&mut self, bit_rate: f32) -> Result<(), Ecs, Espi> {
        let reg = (FOSC / bit_rate) as u16;
        self.write_many(Registers::BitrateMsb, &reg.to_be_bytes())
    }

    /// Computes the frequency deviation, according to `fdev / Fstep` and stores it in
    /// `RegFdevMsb (0x05), RegFdevLsb (0x06)`.
    pub fn fdev(&mut self, fdev: f32) -> Result<(), Ecs, Espi> {
        let reg = (fdev / FSTEP) as u16;
        self.write_many(Registers::FdevMsb, &reg.to_be_bytes())
    }

    /// Computes the radio frequency, according to `frequency / Fstep` and stores it in
    /// `RegFrfMsb (0x07), RegFrfMid (0x08), RegFrfLsb (0x09)`.
    pub fn frequency(&mut self, frequency: f32) -> Result<(), Ecs, Espi> {
        let reg = (frequency / FSTEP) as u32;
        self.write_many(Registers::FrfMsb, &reg.to_be_bytes()[1..])
    }

    /// Stores DIO mapping for different RFM69 modes. For DIO behavior between modes
    /// please refer to the corresponding table in RFM69 datasheet.
    pub fn dio_mapping(&mut self, mapping: DioMapping) -> Result<(), Ecs, Espi> {
        let pin = mapping.pin;
        let dio = Some(mapping);
        match pin {
            DioPin::Dio0 => self.dio[0] = dio,
            DioPin::Dio1 => self.dio[1] = dio,
            DioPin::Dio2 => self.dio[2] = dio,
            DioPin::Dio3 => self.dio[3] = dio,
            DioPin::Dio4 => self.dio[4] = dio,
            DioPin::Dio5 => self.dio[5] = dio,
        }
        self.dio()
    }

    /// Clears stored DIO mapping for specified pin.
    pub fn clear_dio(&mut self, pin: DioPin) -> Result<(), Ecs, Espi> {
        match pin {
            DioPin::Dio0 => self.dio[0] = None,
            DioPin::Dio1 => self.dio[1] = None,
            DioPin::Dio2 => self.dio[2] = None,
            DioPin::Dio3 => self.dio[3] = None,
            DioPin::Dio4 => self.dio[4] = None,
            DioPin::Dio5 => self.dio[5] = None,
        }
        self.dio()
    }

    /// Sets preamble length in corresponding registers `RegPreambleMsb (0x2C),
    /// RegPreambleLsb (0x2D)`.
    pub fn preamble(&mut self, reg: u16) -> Result<(), Ecs, Espi> {
        self.write_many(Registers::PreambleMsb, &reg.to_be_bytes())
    }

    /// Sets sync config and sync words in `RegSyncConfig (0x2E), RegSyncValue1-8(0x2F-0x36)`.
    /// Maximal sync length is 8, pass empty buffer to clear the sync flag.
    pub fn sync(&mut self, sync: &[u8]) -> Result<(), Ecs, Espi> {
        let len = sync.len();
        if len == 0 {
            return self.update(Registers::SyncConfig, |r| r & 0x7f);
        } else if len > 8 {
            return Err(Error::SyncSize);
        }
        let reg = 0x80 | ((len - 1) as u8) << 3;
        self.write(Registers::SyncConfig, reg)?;
        self.write_many(Registers::SyncValue1, sync)
    }

    /// Sets packet settings in corresponding registers `RegPacketConfig1 (0x37),
    /// RegPayloadLength (0x38), RegPacketConfig2 (0x3D)`.
    pub fn packet(&mut self, packet_config: PacketConfig) -> Result<(), Ecs, Espi> {
        let len: u8;
        let mut reg = 0x00;
        match packet_config.format {
            PacketFormat::Fixed(size) => len = size,
            PacketFormat::Variable(size) => {
                len = size;
                reg |= 0x80;
            }
        }
        reg |=
            packet_config.dc as u8 | packet_config.filtering as u8 | (packet_config.crc as u8) << 4;
        self.write_many(Registers::PacketConfig1, &[reg, len])?;
        reg = packet_config.interpacket_rx_delay as u8 | (packet_config.auto_rx_restart as u8) << 1;
        self.update(Registers::PacketConfig2, |r| r & 0x0d | reg)
    }

    /// Sets node address in corresponding register `RegNodeAdrs (0x39)`.
    pub fn node_address(&mut self, a: u8) -> Result<(), Ecs, Espi> {
        self.write(Registers::NodeAddrs, a)
    }

    /// Sets broadcast address in corresponding register `RegBroadcastAdrs (0x3A)`.
    pub fn broadcast_address(&mut self, a: u8) -> Result<(), Ecs, Espi> {
        self.write(Registers::BroadcastAddrs, a)
    }

    /// Sets FIFO mode in corresponding register `RegFifoThresh (0x3C)`.
    pub fn fifo_mode(&mut self, mode: FifoMode) -> Result<(), Ecs, Espi> {
        match mode {
            FifoMode::NotEmpty => self.update(Registers::FifoThresh, |r| r | 0x80),
            FifoMode::Level(level) => self.write(Registers::FifoThresh, level & 0x7f),
        }
    }

    /// Sets AES encryption in corresponding registers `RegPacketConfig2 (0x3D),
    /// RegAesKey1-16 (0x3E-0x4D)`. The key must be 16 bytes long, pass empty buffer to disable
    /// the AES encryption.
    pub fn aes(&mut self, key: &[u8]) -> Result<(), Ecs, Espi> {
        let len = key.len();
        if len == 0 {
            return self.update(Registers::PacketConfig2, |r| r & 0xfe);
        } else if len == 16 {
            self.update(Registers::PacketConfig2, |r| r | 0x01)?;
            return self.write_many(Registers::AesKey1, key);
        }
        Err(Error::AesKeySize)
    }

    /// Last RSSI value that was computed during receive.
    pub fn rssi(&self) -> f32 {
        self.rssi
    }

    /// Receive bytes from another RFM69. This call blocks until there are any
    /// bytes available. This can be combined with DIO interrupt for `PayloadReady`, calling
    /// `recv` immediately after the interrupt should not block.
    pub fn recv(&mut self, buffer: &mut [u8]) -> Result<(), Ecs, Espi> {
        if buffer.is_empty() {
            return Ok(());
        }

        self.mode(Mode::Receiver)?;
        self.wait_mode_ready()?;

        while !self.is_packet_ready()? {}

        self.mode(Mode::Standby)?;
        self.read_many(Registers::Fifo, buffer)?;
        self.rssi = self.read(Registers::RssiValue)? as f32 / -2.0;
        Ok(())
    }

    /// Receive bytes from another RFM69. This call blocks until there are any
    /// bytes available. This can be combined with DIO interrupt for `SyncAddressMatch`, calling
    /// `recv_large` immediately after the interrupt will not block waiting for packets. It will
    /// still block until all data are received.
    /// This function is designed to receive packets larger than the FIFO size by reading `chunk_size`
    /// bytes at a time as soon as the `FifoThreshold` IRQ flag is set.
    /// `CrcAutoClearOff` must be set, and `chunk_size` must not exceed `FifoThreshold + 1`.
    pub fn recv_large(&mut self, chunk_size: usize, buffer: &mut [u8]) -> Result<usize, Ecs, Espi> {
        if buffer.is_empty() {
            return Ok(0);
        }

        self.reset_fifo()?;

        self.mode(Mode::Receiver)?;
        self.wait_mode_ready()?;

        while !self.is_sync_address_match()? {}

        while self.is_fifo_empty()? {}
        let len: usize = self.read(Registers::Fifo)?.into();

        for chunk in buffer[0..len].chunks_mut(chunk_size) {
            self.wait_until_fifo_exceeds_threshold_or_payload_ready()?;
            self.read_many(Registers::Fifo, chunk)?;
        }

        self.mode(Mode::Standby)?;
        self.rssi = self.read(Registers::RssiValue)? as f32 / -2.0;
        Ok(len)
    }

    /// Send bytes to another RFM69. This can block until all data are send.
    pub fn send(&mut self, buffer: &[u8]) -> Result<(), Ecs, Espi> {
        if buffer.is_empty() {
            return Ok(());
        }

        self.mode(Mode::Standby)?;
        self.wait_mode_ready()?;

        self.reset_fifo()?;

        self.write_many(Registers::Fifo, buffer)?;
        self.mode(Mode::Transmitter)?;
        self.wait_packet_sent()?;

        self.mode(Mode::Standby)
    }

    /// Send bytes to another RFM69. This will block until all data are send.
    /// This function is designed to send packets larger than the FIFO size by writing `chunk_size`
    /// bytes at a time as soon as the `FifoThreshold` IRQ flag is cleared. The first write will have
    /// a size of either `prefill` or the length of the packet plus the length byte, whichever is smaller.
    /// `FifoThreshold + chunk_size` must not exceed the FIFO size (66 bytes).
    pub fn send_large(
        &mut self,
        prefill: usize,
        chunk_size: usize,
        buffer: &[u8],
    ) -> Result<(), Ecs, Espi> {
        if buffer.is_empty() {
            return Ok(());
        }

        self.mode(Mode::Standby)?;
        self.wait_mode_ready()?;

        self.reset_fifo()?;

        self.write(Registers::Fifo, buffer.len().try_into().unwrap())?;

        let first_chunk_size = min(prefill - 1, buffer.len());

        self.write_many(Registers::Fifo, &buffer[..first_chunk_size])?;
        self.mode(Mode::Transmitter)?;

        for i in buffer[first_chunk_size..].chunks(chunk_size) {
            self.wait_while_fifo_exceeds_threshold()?;
            self.write_many(Registers::Fifo, i)?;
        }

        self.wait_packet_sent()?;

        self.mode(Mode::Standby)
    }

    /// Check if IRQ flag SyncAddressMatch is set.
    pub fn is_sync_address_match(&mut self) -> Result<bool, Ecs, Espi> {
        Ok(self.read(Registers::IrqFlags1)? & 0x01 != 0)
    }

    /// Check if IRQ flag FifoNotEmpty is cleared.
    pub fn is_fifo_empty(&mut self) -> Result<bool, Ecs, Espi> {
        Ok(self.read(Registers::IrqFlags2)? & 0x40 == 0)
    }

    /// Check if IRQ flag PacketReady is set.
    pub fn is_packet_ready(&mut self) -> Result<bool, Ecs, Espi> {
        Ok(self.read(Registers::IrqFlags2)? & 0x04 != 0)
    }

    /// Configure LNA in corresponding register `RegLna (0x18)`.
    pub fn lna(&mut self, lna: LnaConfig) -> Result<(), Ecs, Espi> {
        let reg = (lna.zin as u8) | (lna.gain_select as u8);
        self.update(Registers::Lna, |r| (r & 0x78) | reg)
    }

    /// Configure RSSI Threshold in corresponding register `RegRssiThresh (0x29)`.
    pub fn rssi_threshold(&mut self, threshold: u8) -> Result<(), Ecs, Espi> {
        self.write(Registers::RssiThresh, threshold)
    }

    /// Configure Sensitivity Boost in corresponding register `RegTestLna (0x58)`.
    pub fn sensitivity_boost(&mut self, boost: SensitivityBoost) -> Result<(), Ecs, Espi> {
        self.write(Registers::TestLna, boost as u8)
    }

    /// Configure Pa13 dBm 1 in corresponding register `RegTestPa1 (0x5A)`.
    pub fn pa13_dbm1(&mut self, pa13: Pa13dBm1) -> Result<(), Ecs, Espi> {
        self.write(Registers::TestPa1, pa13 as u8)
    }

    /// Configure Pa13 dBm 2 in corresponding register `RegTestPa2 (0x5C)`.
    pub fn pa13_dbm2(&mut self, pa13: Pa13dBm2) -> Result<(), Ecs, Espi> {
        self.write(Registers::TestPa2, pa13 as u8)
    }

    /// Configure Continuous Dagc in corresponding register `RegTestDagc (0x6F)`.
    pub fn continuous_dagc(&mut self, cdagc: ContinuousDagc) -> Result<(), Ecs, Espi> {
        self.write(Registers::TestDagc, cdagc as u8)
    }

    /// Configure Rx Bandwidth in corresponding register `RegRxBw (0x19)`.
    pub fn rx_bw<RxBwT>(&mut self, rx_bw: RxBw<RxBwT>) -> Result<(), Ecs, Espi>
    where
        RxBwT: RxBwFreq,
    {
        self.write(
            Registers::RxBw,
            rx_bw.dcc_cutoff as u8 | rx_bw.rx_bw.value(),
        )
    }

    /// Configure Rx AFC Bandwidth in corresponding register `RegAfcBw (0x1A)`.
    pub fn rx_afc_bw<RxBwT>(&mut self, rx_bw: RxBw<RxBwT>) -> Result<(), Ecs, Espi>
    where
        RxBwT: RxBwFreq,
    {
        self.write(
            Registers::AfcBw,
            rx_bw.dcc_cutoff as u8 | rx_bw.rx_bw.value(),
        )
    }

    /// Direct write to RFM69 registers.
    pub fn write(&mut self, reg: Registers, val: u8) -> Result<(), Ecs, Espi> {
        self.write_many(reg, &[val])
    }

    /// Direct write to RFM69 registers.
    pub fn write_many(&mut self, reg: Registers, data: &[u8]) -> Result<(), Ecs, Espi> {
        let mut guard = CsGuard::new(&mut self.cs);
        guard.select()?;
        self.spi.write(&[reg.write()]).map_err(Error::Spi)?;
        self.spi.write(data).map_err(Error::Spi)?;
        Ok(())
    }

    /// Direct read from RFM69 registers.
    pub fn read(&mut self, reg: Registers) -> Result<u8, Ecs, Espi> {
        let mut buffer = [0u8; 1];
        self.read_many(reg, &mut buffer)?;
        Ok(buffer[0])
    }

    /// Direct read from RFM69 registers.
    pub fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> Result<(), Ecs, Espi> {
        let mut guard = CsGuard::new(&mut self.cs);
        guard.select()?;
        self.spi.write(&[reg.read()]).map_err(Error::Spi)?;
        self.spi.transfer(buffer).map_err(Error::Spi)?;
        Ok(())
    }

    fn dio(&mut self) -> Result<(), Ecs, Espi> {
        let mut reg = 0x07;
        for opt_mapping in self.dio.iter() {
            if let Some(mapping) = opt_mapping {
                if mapping.dio_mode.eq(self.mode) {
                    reg |= (mapping.dio_type as u16) << (mapping.pin as u16);
                }
            }
        }
        self.write_many(Registers::DioMapping1, &reg.to_be_bytes())
    }

    fn reset_fifo(&mut self) -> Result<(), Ecs, Espi> {
        self.write(Registers::IrqFlags2, 0x10)
    }

    fn wait_mode_ready(&mut self) -> Result<(), Ecs, Espi> {
        self.with_timeout(100, 5, |rfm| {
            Ok((rfm.read(Registers::IrqFlags1)? & 0x80) != 0)
        })
    }

    fn wait_packet_sent(&mut self) -> Result<(), Ecs, Espi> {
        self.with_timeout(100, 5, |rfm| {
            Ok((rfm.read(Registers::IrqFlags2)? & 0x08) != 0)
        })
    }

    fn wait_while_fifo_exceeds_threshold(&mut self) -> Result<(), Ecs, Espi> {
        for _ in 0..1000 {
            if self.read(Registers::IrqFlags2)? & 0x20 == 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    fn wait_until_fifo_exceeds_threshold_or_payload_ready(&mut self) -> Result<(), Ecs, Espi> {
        for _ in 0..1000 {
            if self.read(Registers::IrqFlags2)? & 0x24 != 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    fn with_timeout<F>(&mut self, timeout: u8, step: u8, func: F) -> Result<(), Ecs, Espi>
    where
        F: Fn(&mut Rfm69<T, S, D>) -> Result<bool, Ecs, Espi>,
    {
        let mut done = func(self)?;
        let mut count = 0;
        while !done && count < timeout {
            self.delay.delay_ms(step);
            count += step;
            done = func(self)?;
        }
        if !done {
            return Err(Error::Timeout);
        }
        Ok(())
    }

    fn update<F>(&mut self, reg: Registers, f: F) -> Result<(), Ecs, Espi>
    where
        F: FnOnce(u8) -> u8,
    {
        let val = self.read(reg)?;
        self.write(reg, f(val))
    }
}

struct CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    cs: &'a mut T,
    _phantom: PhantomData<Espi>,
}

impl<'a, T, Ecs, Espi> CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    fn new(pin: &'a mut T) -> Self {
        CsGuard {
            cs: pin,
            _phantom: PhantomData,
        }
    }

    fn select(&mut self) -> Result<(), Ecs, Espi> {
        self.cs.set_low().map_err(Error::Cs)
    }

    fn unselect(&mut self) -> Result<(), Ecs, Espi> {
        self.cs.set_high().map_err(Error::Cs)
    }
}

impl<'a, T, Ecs, Espi> Drop for CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    fn drop(&mut self) {
        if self.unselect().is_err() {
            panic!("Cannot clear CS guard");
        }
    }
}

/// Configures RFM69 according to [LowPowerLab](https://github.com/LowPowerLab/RFM69) Arduino
/// library
pub fn low_power_lab_defaults<T, S, D, Ecs, Espi>(
    mut rfm: Rfm69<T, S, D>,
    network_id: u8,
    frequency: f32,
) -> Result<Rfm69<T, S, D>, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
    S: Transfer<u8, Error = Espi>,
    S: Write<u8, Error = Espi>,
    D: DelayMs<u8>,
{
    rfm.mode(Mode::Standby)?;
    rfm.modulation(Modulation {
        data_mode: DataMode::Packet,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping00,
    })?;
    rfm.bit_rate(55_555.0)?;
    rfm.frequency(frequency)?;
    rfm.fdev(50_000.0)?;
    rfm.rx_bw(RxBw {
        dcc_cutoff: DccCutoff::Percent4,
        rx_bw: RxBwFsk::Khz125dot0,
    })?;
    rfm.preamble(3)?;
    rfm.sync(&[0x2d, network_id])?;
    rfm.packet(PacketConfig {
        format: PacketFormat::Variable(66),
        dc: PacketDc::None,
        filtering: PacketFiltering::None,
        crc: true,
        interpacket_rx_delay: InterPacketRxDelay::Delay2Bits,
        auto_rx_restart: true,
    })?;
    rfm.fifo_mode(FifoMode::NotEmpty)?;
    rfm.lna(LnaConfig {
        zin: LnaImpedance::Ohm50,
        gain_select: LnaGain::AgcLoop,
    })?;
    rfm.rssi_threshold(220)?;
    rfm.continuous_dagc(ContinuousDagc::ImprovedMarginAfcLowBetaOn0)?;
    Ok(rfm)
}

#[cfg(test)]
mod tests;
