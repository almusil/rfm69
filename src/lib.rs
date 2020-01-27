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

pub mod registers;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

use registers::{
    DataMode, DioMapping, DioPin, FifoMode, InterPacketRxDelay, Mode, Modulation,
    ModulationShaping, ModulationType, PacketConfig, PacketDc, PacketFiltering, PacketFormat,
    Registers,
};

const FOSC: f32 = 32_000_000.0;
const FSTEP: f32 = FOSC / 524_288.0; // FOSC/2^19

type Result<T> = core::result::Result<T, Error>;

pub struct Error(pub &'static str);

/// Main struct to interact with RFM69 chip.
pub struct Rfm69<T, S, D>
where
    T: OutputPin,
    S: Transfer<u8>,
    D: DelayMs<u8>,
{
    spi: S,
    cs: T,
    delay: D,
    mode: Mode,
    dio: [Option<DioMapping>; 6],
    rssi: f32,
}

impl<T, S, D> Rfm69<T, S, D>
where
    T: OutputPin,
    S: Transfer<u8>,
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
    pub fn read_all_regs(&mut self) -> Result<[u8; 0x4f]> {
        let mut buffer = [0u8; 0x4f];
        self.read_many(Registers::OpMode, &mut buffer)?;
        Ok(buffer)
    }

    /// Sets the mode in corresponding register `RegOpMode (0x01)`.
    pub fn mode(&mut self, mode: Mode) -> Result<()> {
        let val = mode as u8;
        self.update(Registers::OpMode, |r| (r & 0xe3) | val)?;
        self.mode = mode;
        self.dio()
    }

    /// Sets the modulation in corresponding register `RegDataModul (0x02)`.
    pub fn modulation(&mut self, modulation: Modulation) -> Result<()> {
        self.write(Registers::DataModul, modulation.value())
    }

    /// Computes the bitrate, according to `Fosc / bit_rate` and stores it in
    /// `RegBitrateMsb (0x03), RegBitrateLsb (0x04)`.
    pub fn bit_rate(&mut self, bit_rate: f32) -> Result<()> {
        let reg = (FOSC / bit_rate) as u16;
        let mut val = reg.to_be_bytes();
        self.write_many(Registers::BitrateMsb, &mut val)
    }

    /// Computes the frequency deviation, according to `fdev / Fstep` and stores it in
    /// `RegFdevMsb (0x05), RegFdevLsb (0x06)`.
    pub fn fdev(&mut self, fdev: f32) -> Result<()> {
        let reg = (fdev / FSTEP) as u16;
        let mut val = reg.to_be_bytes();
        self.write_many(Registers::FdevMsb, &mut val)
    }

    /// Computes the radio frequency, according to `frequency / Fstep` and stores it in
    /// `RegFrfMsb (0x07), RegFrfMid (0x08), RegFrfLsb (0x09)`.
    pub fn frequency(&mut self, frequency: f32) -> Result<()> {
        let reg = (frequency / FSTEP) as u32;
        let mut val = reg.to_be_bytes();
        self.write_many(Registers::FrfMsb, &mut val[1..])
    }

    /// Stores DIO mapping for different RFM69 modes. For DIO behavior between modes
    /// please refer to the corresponding table in RFM69 datasheet.
    pub fn dio_mapping(&mut self, mapping: DioMapping) -> Result<()> {
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
    pub fn clear_dio(&mut self, pin: DioPin) -> Result<()> {
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
    pub fn preamble(&mut self, reg: u16) -> Result<()> {
        let mut val = reg.to_be_bytes();
        self.write_many(Registers::PreambleMsb, &mut val)
    }

    /// Sets sync config and sync words in `RegSyncConfig (0x2E), RegSyncValue1-8(0x2F-0x36)`.
    /// Maximal sync length is 8, pass empty buffer to clear the sync flag.
    pub fn sync(&mut self, sync: &mut [u8]) -> Result<()> {
        let len = sync.len();
        if len == 0 {
            return self.update(Registers::SyncConfig, |r| r & 0x7f);
        } else if len > 8 {
            Err(Error("Too big sync size"))?;
        }
        let reg = 0x80 | ((len - 1) as u8) << 3;
        self.write(Registers::SyncConfig, reg)?;
        self.write_many(Registers::SyncValue1, sync)
    }

    /// Sets packet settings in corresponding registers `RegPacketConfig1 (0x37),
    /// RegPayloadLength (0x38), RegPacketConfig2 (0x3D)`.
    pub fn packet(&mut self, packet_config: PacketConfig) -> Result<()> {
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
        let mut val = [reg, len];
        self.write_many(Registers::PacketConfig1, &mut val)?;
        reg = packet_config.interpacket_rx_delay as u8 | (packet_config.auto_rx_restart as u8) << 1;
        self.update(Registers::PacketConfig2, |r| r & 0x0d | reg)
    }

    /// Sets node address in corresponding register `RegNodeAdrs (0x39)`.
    pub fn node_address(&mut self, a: u8) -> Result<()> {
        self.write(Registers::NodeAddrs, a)
    }

    /// Sets broadcast address in corresponding register `RegBroadcastAdrs (0x3A)`.
    pub fn broadcast_address(&mut self, a: u8) -> Result<()> {
        self.write(Registers::BroadcastAddrs, a)
    }

    /// Sets FIFO mode in corresponding register `RegFifoThresh (0x3C)`.
    pub fn fifo_mode(&mut self, mode: FifoMode) -> Result<()> {
        match mode {
            FifoMode::NotEmpty => self.update(Registers::FifoThresh, |r| r | 0x80),
            FifoMode::Level(level) => self.write(Registers::FifoThresh, level & 0x7f),
        }
    }

    /// Sets AES encryption in corresponding registers `RegPacketConfig2 (0x3D),
    /// RegAesKey1-16 (0x3E-0x4D)`. The key must be 16 bytes long, pass empty buffer to disable
    /// the AES encryption.
    pub fn aes(&mut self, key: &mut [u8]) -> Result<()> {
        let len = key.len();
        if len == 0 {
            return self.update(Registers::PacketConfig2, |r| r & 0xfe);
        } else if len == 16 {
            self.update(Registers::PacketConfig2, |r| r | 0x01)?;
            return self.write_many(Registers::AesKey1, key);
        }
        Err(Error("Invalid AES key size"))
    }

    /// Last RSSI value that was computed during receive.
    pub fn rssi(&self) -> f32 {
        self.rssi
    }

    /// Receive bytes from another RFM69. This call blocks until there are any
    /// bytes available. This can be combined with DIO interrupt for `PayloadReady`, calling
    /// `recv` immediately after the interrupt should not block.
    pub fn recv(&mut self, buffer: &mut [u8]) -> Result<()> {
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

    /// Send bytes to another RFM69. This can block until all data are send.
    pub fn send(&mut self, buffer: &mut [u8]) -> Result<()> {
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

    /// Check if IRQ flag PacketReady is set.
    pub fn is_packet_ready(&mut self) -> Result<bool> {
        Ok(self.read(Registers::IrqFlags2)? & 0x04 != 0)
    }

    fn dio(&mut self) -> Result<()> {
        let mut reg = 0x07;
        for opt_mapping in self.dio.iter() {
            if let Some(mapping) = opt_mapping {
                if mapping.dio_mode.eq(self.mode) {
                    reg |= (mapping.dio_type as u16) << (mapping.pin as u16);
                }
            }
        }
        let mut val = reg.to_be_bytes();
        self.write_many(Registers::DioMapping1, &mut val)
    }

    fn reset_fifo(&mut self) -> Result<()> {
        self.write(Registers::IrqFlags2, 0x10)
    }

    fn wait_mode_ready(&mut self) -> Result<()> {
        self.with_timeout(100, 5, |rfm| {
            Ok((rfm.read(Registers::IrqFlags1)? & 0x80) != 0)
        })
    }

    fn wait_packet_sent(&mut self) -> Result<()> {
        self.with_timeout(100, 5, |rfm| {
            Ok((rfm.read(Registers::IrqFlags2)? & 0x08) != 0)
        })
    }

    fn with_timeout<F>(&mut self, timeout: u8, step: u8, func: F) -> Result<()>
    where
        F: Fn(&mut Rfm69<T, S, D>) -> Result<bool>,
    {
        let mut done = func(self)?;
        let mut count = 0;
        while !done && count < timeout {
            self.delay.delay_ms(step);
            count += step;
            done = func(self)?;
        }
        if !done {
            Err(Error("Timeout"))?;
        }
        Ok(())
    }

    fn update<F>(&mut self, reg: Registers, f: F) -> Result<()>
    where
        F: FnOnce(u8) -> u8,
    {
        let val = self.read(reg)?;
        self.write(reg, f(val))
    }

    fn write(&mut self, reg: Registers, val: u8) -> Result<()> {
        self.write_many(reg, &mut [val])
    }

    fn write_many(&mut self, reg: Registers, data: &mut [u8]) -> Result<()> {
        let mut guard = CsGuard::new(&mut self.cs);
        guard.select()?;
        self.spi
            .transfer(&mut [reg.write()])
            .map_err(map_spi_err::<S>)?;
        self.spi.transfer(data).map_err(map_spi_err::<S>)?;
        Ok(())
    }

    fn read(&mut self, reg: Registers) -> Result<u8> {
        let mut buffer = [0u8; 1];
        self.read_many(reg, &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> Result<()> {
        let mut guard = CsGuard::new(&mut self.cs);
        guard.select()?;
        self.spi
            .transfer(&mut [reg.read()])
            .map_err(map_spi_err::<S>)?;
        self.spi.transfer(buffer).map_err(map_spi_err::<S>)?;
        Ok(())
    }
}

struct CsGuard<'a, T>
where
    T: OutputPin,
{
    cs: &'a mut T,
}

impl<'a, T> CsGuard<'a, T>
where
    T: OutputPin,
{
    fn new(pin: &'a mut T) -> Self {
        CsGuard { cs: pin }
    }

    fn select(&mut self) -> Result<()> {
        self.cs.set_low().map_err(map_output_pin_err::<T>)
    }

    fn unselect(&mut self) -> Result<()> {
        self.cs.set_high().map_err(map_output_pin_err::<T>)
    }
}

impl<'a, T> Drop for CsGuard<'a, T>
where
    T: OutputPin,
{
    fn drop(&mut self) {
        if self.unselect().is_err() {
            panic!("Cannot clear CS guard");
        }
    }
}

fn map_output_pin_err<T>(_: T::Error) -> Error
where
    T: OutputPin,
{
    Error { 0: "CS pin error" }
}

fn map_spi_err<T>(_: T::Error) -> Error
where
    T: Transfer<u8>,
{
    Error { 0: "SPI error" }
}

/// Configures RFM69 according to [LowPowerLab](https://github.com/LowPowerLab/RFM69) Arduino
/// library
pub fn low_power_lab_defaults<T, S, D>(
    mut rfm: Rfm69<T, S, D>,
    network_id: u8,
    frequency: f32,
) -> Result<Rfm69<T, S, D>>
where
    T: OutputPin,
    S: Transfer<u8>,
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
    rfm.write(Registers::RxBw, 0x42)?;
    rfm.preamble(3)?;
    rfm.sync(&mut [0x2d, network_id])?;
    rfm.packet(PacketConfig {
        format: PacketFormat::Variable(66),
        dc: PacketDc::None,
        filtering: PacketFiltering::None,
        crc: true,
        interpacket_rx_delay: InterPacketRxDelay::Delay2Bits,
        auto_rx_restart: true,
    })?;
    rfm.fifo_mode(FifoMode::NotEmpty)?;
    Ok(rfm)
}

#[cfg(test)]
mod tests;
